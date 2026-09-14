// Сборка: clang++ popo.cpp -std=c++17 -O2 -Wall -Wextra $(pkg-config --cflags --libs raylib) -o popo
#include "raylib.h"
#include <algorithm>
#include <cmath>
#include <numeric>
#include <stdexcept>
#include <vector>

// Физика рассчитывается в double; преобразование во float для raylib выполняется только при отрисовке.
struct Vec {
    double x = 0, y = 0;
    Vec operator+(Vec b) const { return {x+b.x, y+b.y}; }
    Vec operator-(Vec b) const { return {x-b.x, y-b.y}; }
    Vec operator*(double s) const { return {x*s, y*s}; }
    Vec operator/(double s) const { return {x/s, y/s}; }
    Vec& operator+=(Vec b) { x+=b.x; y+=b.y; return *this; }
    Vec& operator-=(Vec b) { x-=b.x; y-=b.y; return *this; }
};
double dot(Vec a, Vec b) { return a.x*b.x+a.y*b.y; }
double cross(Vec a, Vec b) { return a.x*b.y-a.y*b.x; }
double length(Vec a) { return std::sqrt(dot(a,a)); }
Vec spin(double w, Vec r) { return {-w*r.y, w*r.x}; }
Vector2 screen(Vec p) { return {float(p.x),float(p.y)}; }
Vec limited(Vec v, double maxLength) {
    double l=length(v); return l>maxLength ? v*(maxLength/l) : v;
}
struct PhysicsMaterial { double restitution, friction, density; Color color; const char* name; };
const PhysicsMaterial materials[] = {
    {0.28, 0.65, 1.0, {75,185,220,255}, "Wood"},
    {0.80, 0.85, 0.7, {243,171,77,255}, "Rubber"},
    {0.12, 0.30, 3.0, {169,180,208,255}, "Steel"}
};
struct Circle {
    Vec position, velocity;
    double radius=14, invMass=1, invInertia=1;
    double angle=0, angularVelocity=0;
    int material=0;
};
Circle makeCircle(Vec position, double radius, int material=0, bool fixed=false) {
    if(!std::isfinite(radius) || radius<8 || radius>50 || material<0 || material>2)
        throw std::invalid_argument("Invalid circle radius or material");
    Circle c; c.position=position; c.radius=radius; c.material=material;
    double mass=materials[material].density*radius*radius/196.0;
    c.invMass=fixed ? 0 : 1/mass;
    c.invInertia=fixed ? 0 : 2/(mass*radius*radius);
    return c;
}
struct Box { double x,y,w,h; };
// Индексы сохраняются при перераспределении памяти вектора. После добавления или удаления тел контакты пересоздаются.
struct Contact {
    int a, b; // b == -1: неподвижная стена или прямоугольное препятствие
    Vec normal, ra, rb;
    double penetration, restitution, friction;
    double normalImpulse=0, tangentImpulse=0, bounce=0;
};
struct World {
    static constexpr double dt=1.0/120.0;
    static constexpr int maxBodies=500;
    double width=1100, height=760, top=112;
    bool gravityEnabled=true;
    std::vector<Circle> bodies;
    std::vector<Box> boxes;
    std::vector<Contact> contacts;
    int dragged=-1;
    Vec mouse;
    int candidatePairs=0;

    void add(Circle c) {
        if(bodies.size()>=maxBodies) return;
        c.position.x=std::clamp(c.position.x,c.radius,width-c.radius);
        c.position.y=std::clamp(c.position.y,top+c.radius,height-c.radius);
        bodies.push_back(c); contacts.clear();
    }
    void erase(int index) {
        if(index<0 || index>=int(bodies.size())) return;
        bodies.erase(bodies.begin()+index); dragged=-1; contacts.clear();
    }
    int pick(Vec p) const {
        for(int i=int(bodies.size())-1;i>=0;--i)
            if(length(p-bodies[i].position)<=bodies[i].radius) return i;
        return -1;
    }
    void reset() {
        bodies.clear(); contacts.clear(); dragged=-1;
        boxes={{130,405,240,18},{730,405,240,18},{435,545,230,18}};
        for(int row=0;row<4;++row) for(int col=0;col<12;++col)
            add(makeCircle({180.0+col*43,165.0+row*39},14,col%3));
        add(makeCircle({550,385},35,2,true));
    }
    void contact(int a, int b, Vec normal, double penetration) {
        const Circle& ca=bodies[a];
        const PhysicsMaterial& ma=materials[ca.material];
        const PhysicsMaterial& mb=b>=0 ? materials[bodies[b].material] : materials[0];
        Vec ra=normal*ca.radius;
        Vec rb=b>=0 ? normal*(-bodies[b].radius) : Vec{};
        contacts.push_back({a,b,normal,ra,rb,penetration,
                           b>=0 ? std::min(ma.restitution,mb.restitution) : ma.restitution,
                           std::sqrt(ma.friction*mb.friction)});
    }
    void collidePair(int a, int b) {
        auto& ca=bodies[a]; auto& cb=bodies[b];
        if(ca.invMass+cb.invMass==0) return;
        Vec delta=cb.position-ca.position;
        double radii=ca.radius+cb.radius;
        if(std::abs(delta.y)>radii || dot(delta,delta)>radii*radii) return;
        double distance=length(delta);
        contact(a,b,distance>1e-9 ? delta/distance : Vec{1,0},radii-distance);
    }
    void collideBox(int i, const Box& b) {
        const auto& c=bodies[i];
        Vec q={std::clamp(c.position.x,b.x,b.x+b.w),
               std::clamp(c.position.y,b.y,b.y+b.h)};
        Vec toward=q-c.position; double distance=length(toward);
        if(distance>c.radius) return;
        if(distance>1e-9) { contact(i,-1,toward/distance,c.radius-distance); return; }
        // Если центр внутри прямоугольника или на его границе, выталкиваем шар через ближайшую сторону.
        double depths[]={c.position.x-b.x,b.x+b.w-c.position.x,
                         c.position.y-b.y,b.y+b.h-c.position.y};
        Vec normals[]={{1,0},{-1,0},{0,1},{0,-1}};
        int face=int(std::min_element(depths,depths+4)-depths);
        contact(i,-1,normals[face],c.radius+depths[face]);
    }
    void findContacts() {
        contacts.clear(); candidatePairs=0;
        // Проход по оси X: пропускаем пары, чьи границы по горизонтали не могут пересекаться.
        std::vector<int> order(bodies.size());
        std::iota(order.begin(),order.end(),0);
        std::stable_sort(order.begin(),order.end(),[&](int a,int b) {
            return bodies[a].position.x-bodies[a].radius < bodies[b].position.x-bodies[b].radius;
        });
        for(size_t a=0;a<order.size();++a) {
            int i=order[a]; const auto& c=bodies[i];
            for(size_t b=a+1;b<order.size();++b) {
                int j=order[b];
                if(bodies[j].position.x-bodies[j].radius>c.position.x+c.radius) break;
                ++candidatePairs; collidePair(i,j);
            }
            if(c.invMass==0) continue;
            if(c.position.x-c.radius<=0) contact(i,-1,{-1,0},c.radius-c.position.x);
            if(c.position.x+c.radius>=width) contact(i,-1,{1,0},c.position.x+c.radius-width);
            if(c.position.y-c.radius<=top) contact(i,-1,{0,-1},top+c.radius-c.position.y);
            if(c.position.y+c.radius>=height) contact(i,-1,{0,1},c.position.y+c.radius-height);
            for(const auto& box:boxes) collideBox(i,box);
        }
    }
    Vec relativeVelocity(const Contact& c) const {
        Vec va=bodies[c.a].velocity+spin(bodies[c.a].angularVelocity,c.ra);
        Vec vb=c.b>=0 ? bodies[c.b].velocity+spin(bodies[c.b].angularVelocity,c.rb) : Vec{};
        return vb-va;
    }
    double effectiveMass(const Contact& c, Vec axis) const {
        const auto& a=bodies[c.a];
        double ar=cross(c.ra,axis);
        double k=a.invMass+ar*ar*a.invInertia;
        if(c.b>=0) {
            const auto& b=bodies[c.b]; double br=cross(c.rb,axis);
            k+=b.invMass+br*br*b.invInertia;
        }
        return k;
    }
    void impulse(const Contact& c, Vec p) {
        auto& a=bodies[c.a];
        a.velocity-=p*a.invMass;
        a.angularVelocity-=cross(c.ra,p)*a.invInertia;
        if(c.b>=0) {
            auto& b=bodies[c.b]; b.velocity+=p*b.invMass;
            b.angularVelocity+=cross(c.rb,p)*b.invInertia;
        }
    }
    void solveVelocity() {
        for(auto& c:contacts) {
            double vn=dot(relativeVelocity(c),c.normal);
            // Подавляем слабые отскоки, вызванные гравитацией, для покоящихся тел в контакте.
            c.bounce=vn < -35 ? -c.restitution*vn : 0;
        }
        for(int iteration=0;iteration<16;++iteration) for(auto& c:contacts) {
            double k=effectiveMass(c,c.normal); if(k<=0) continue;
            double delta=(c.bounce-dot(relativeVelocity(c),c.normal))/k;
            double old=c.normalImpulse;
            c.normalImpulse=std::max(0.0,old+delta);
            impulse(c,c.normal*(c.normalImpulse-old));
            Vec tangent={-c.normal.y,c.normal.x};
            double kt=effectiveMass(c,tangent); if(kt<=0) continue;
            delta=-dot(relativeVelocity(c),tangent)/kt;
            double limit=c.friction*c.normalImpulse;
            old=c.tangentImpulse;
            c.tangentImpulse=std::clamp(old+delta,-limit,limit);
            impulse(c,tangent*(c.tangentImpulse-old));
        }
    }
    void solvePosition() {
        // Пересчитываем геометрию на каждом проходе, учитывая стены и центры внутри препятствий.
        for(int iteration=0;iteration<6;++iteration) {
            findContacts();
            for(const auto& c:contacts) {
                auto& a=bodies[c.a];
                double invB=c.b>=0 ? bodies[c.b].invMass : 0;
                double sum=a.invMass+invB; if(sum<=0) continue;
                Vec correction=c.normal*(0.65*std::max(c.penetration-0.02,0.0)/sum);
                a.position-=correction*a.invMass;
                if(c.b>=0) bodies[c.b].position+=correction*invB;
            }
        }
    }
    void step() {
        // Четыре подшага и ограничение скорости удерживают перемещение в пределах 4 пикселей за подшаг.
        // Это снижает риск пролёта сквозь объекты, но не заменяет непрерывное обнаружение столкновений.
        constexpr int substeps=4;
        constexpr double h=dt/substeps;
        for(int sub=0;sub<substeps;++sub) {
            for(int i=0;i<int(bodies.size());++i) {
                auto& c=bodies[i]; if(c.invMass==0) continue;
                c.velocity.y+=(gravityEnabled?800:0)*h;
                if(i==dragged) {
                    Vec acceleration=(mouse-c.position)*110.0-c.velocity*19.0;
                    c.velocity+=limited(acceleration,16000)*h;
                }
                c.velocity=limited(c.velocity*std::exp(-0.08*h),1800);
                c.angularVelocity=std::clamp(c.angularVelocity*std::exp(-0.04*h),-100.0,100.0);
                c.position+=c.velocity*h;
                c.angle=std::remainder(c.angle+c.angularVelocity*h,2*3.141592653589793);
            }
            findContacts(); solveVelocity(); solvePosition();
        }
        findContacts();
    }
};

#ifndef PHYSICS_NO_MAIN
int main() {
    SetConfigFlags(FLAG_MSAA_4X_HINT);
    InitWindow(1100,760,"Physics Lab");
    SetTargetFPS(120);
    World world; world.reset();
    bool paused=false, debug=false;
    int material=0;
    double radius=16, accumulator=0;
    const Color background={16,23,36,255}, panel={24,34,51,255}, muted={146,166,189,255};
    while(!WindowShouldClose()) {
        double elapsed=std::min(double(GetFrameTime()),0.1);
        auto mp=GetMousePosition(); world.mouse={mp.x,mp.y};
        if(IsKeyPressed(KEY_SPACE)) { paused=!paused; accumulator=0; }
        if(IsKeyPressed(KEY_TAB)) debug=!debug;
        if(IsKeyPressed(KEY_G)) world.gravityEnabled=!world.gravityEnabled;
        if(IsKeyPressed(KEY_R)) { world.reset(); accumulator=0; }
        if(IsKeyPressed(KEY_C)) { world.bodies.clear(); world.contacts.clear(); world.dragged=-1; }
        if(IsKeyPressed(KEY_ONE)) material=0;
        if(IsKeyPressed(KEY_TWO)) material=1;
        if(IsKeyPressed(KEY_THREE)) material=2;
        radius=std::clamp(radius+GetMouseWheelMove()*2,8.0,40.0);
        bool inScene=mp.y>world.top;
        if(inScene && IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
            world.add(makeCircle(world.mouse,radius,material));
        }
        if(inScene && IsMouseButtonPressed(MOUSE_BUTTON_RIGHT)) {
            int selected=world.pick(world.mouse);
            if(selected>=0 && world.bodies[selected].invMass>0) world.dragged=selected;
        }
        if(!IsMouseButtonDown(MOUSE_BUTTON_RIGHT)) world.dragged=-1;
        if(inScene && IsMouseButtonPressed(MOUSE_BUTTON_MIDDLE)) world.erase(world.pick(world.mouse));
        if(inScene && IsKeyPressed(KEY_B)) world.add(makeCircle(world.mouse,radius,material,true));
        if(paused) { if(IsKeyPressed(KEY_N)) world.step(); }
        else {
            accumulator+=elapsed;
            while(accumulator>=World::dt) { world.step(); accumulator-=World::dt; }
        }
        BeginDrawing(); ClearBackground(background);
        for(int x=0;x<1100;x+=40) DrawLine(x,int(world.top),x,760,Fade(muted,0.07f));
        for(int y=int(world.top);y<760;y+=40) DrawLine(0,y,1100,y,Fade(muted,0.07f));
        for(const auto& b:world.boxes) {
            DrawRectangleRounded({float(b.x),float(b.y),float(b.w),float(b.h)},0.3f,4,{59,76,99,255});
            DrawLine(int(b.x+5),int(b.y),int(b.x+b.w-5),int(b.y),muted);
        }
        for(const auto& c:world.bodies) {
            Color color=materials[c.material].color;
            if(c.invMass==0) {
                DrawCircleV(screen(c.position),float(c.radius),{51,64,83,255});
                DrawCircleLinesV(screen(c.position),float(c.radius),muted);
                DrawLineV(screen(c.position-Vec{5,0}),screen(c.position+Vec{5,0}),muted);
                DrawLineV(screen(c.position-Vec{0,5}),screen(c.position+Vec{0,5}),muted);
            } else {
                DrawCircleV(screen(c.position+Vec{2,3}),float(c.radius),Fade(BLACK,0.25f));
                DrawCircleV(screen(c.position),float(c.radius),color);
                Vec mark={std::cos(c.angle)*c.radius*0.8,std::sin(c.angle)*c.radius*0.8};
                DrawLineEx(screen(c.position),screen(c.position+mark),2,Fade(background,0.6f));
            }
            if(debug) {
                DrawRectangleLinesEx({float(c.position.x-c.radius),float(c.position.y-c.radius),
                                     float(2*c.radius),float(2*c.radius)},1,Fade(muted,0.45f));
                DrawLineV(screen(c.position),screen(c.position+c.velocity*0.08),GREEN);
            }
        }
        if(debug) for(const auto& c:world.contacts) {
            Vec p=world.bodies[c.a].position+c.ra;
            DrawCircleV(screen(p),2,RED);
            DrawLineV(screen(p),screen(p+c.normal*18),RED);
        }
        if(world.dragged>=0)
            DrawLineEx(screen(world.bodies[world.dragged].position),mp,2,materials[material].color);
        if(inScene) DrawCircleLinesV(mp,float(radius),Fade(materials[material].color,0.5f));
        DrawRectangle(0,0,1100,int(world.top),panel);
        DrawText("PHYSICS LAB",20,14,24,RAYWHITE);
        DrawText(paused?"PAUSED":"RUNNING",222,19,16,paused?ORANGE:GREEN);
        DrawText(TextFormat("%s  |  Radius %.0f  |  Gravity %s",materials[material].name,radius,
                            world.gravityEnabled?"ON":"OFF"),345,19,16,materials[material].color);
        DrawText(TextFormat("%d / %d bodies   %d contacts   %d FPS",int(world.bodies.size()),
                            World::maxBodies,int(world.contacts.size()),GetFPS()),745,20,14,muted);
        DrawText("LMB: add   RMB: grab / throw   MMB: delete   Wheel: size   1/2/3: wood / rubber / steel",20,52,16,muted);
        DrawText("Space: pause   N: step   G: gravity   B: fixed ball   R: reset   C: clear balls   Tab: debug",20,79,16,muted);
        EndDrawing();
    }
    CloseWindow();
}
#endif
