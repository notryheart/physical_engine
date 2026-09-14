#define PHYSICS_NO_MAIN
#include "../popo.cpp"
#include <iostream>
#include <cstdlib>
void check(bool ok, const char* name) {
    if(!ok) { std::cerr<<"FAIL: "<<name<<'\n'; std::exit(1); }
    std::cout<<"PASS: "<<name<<'\n';
}
int main() {
    {
        World w; w.gravityEnabled=false;
        w.add(makeCircle({400,300},14)); w.add(makeCircle({428,300},14));
        w.bodies[0].velocity={100,0}; w.bodies[1].velocity={-100,0};
        w.findContacts(); w.solveVelocity();
        check(std::abs(w.bodies[0].velocity.x+28)<1e-8 &&
              std::abs(w.bodies[1].velocity.x-28)<1e-8,"head-on restitution and momentum");
    }
    {
        World w; w.gravityEnabled=false;
        w.add(makeCircle({400,300},14)); w.add(makeCircle({400,300},14));
        w.step();
        check(length(w.bodies[0].position-w.bodies[1].position)>27.9,"coincident circles separate");
    }
    {
        World w; w.gravityEnabled=false; w.boxes={{400,300,200,20}};
        w.add(makeCircle({500,310},14)); w.step();
        check(w.bodies[0].position.y<286.1,"circle embedded in platform escapes");
    }
    {
        World w;
        w.add(makeCircle({400,300},25,2,true));
        w.add(makeCircle({400,270},14));
        for(int n=0;n<120;++n) w.step();
        check(w.bodies[0].position.x==400 && w.bodies[0].position.y==300 &&
              length(w.bodies[0].velocity)==0,"fixed body remains immovable");
    }
    {
        World w; w.add(makeCircle({400,746},14));
        w.bodies[0].velocity={200,0};
        for(int n=0;n<120;++n) w.step();
        check(std::abs(w.bodies[0].angularVelocity)>1 && w.bodies[0].velocity.x<190,
              "floor friction produces rolling");
    }
    {
        World w;
        for(int i=0;i<8;++i) w.add(makeCircle({500,746.0-i*28},14,2));
        for(int n=0;n<1200;++n) w.step();
        double speed=0, penetration=0;
        for(auto& b:w.bodies) speed=std::max(speed,length(b.velocity));
        for(auto& c:w.contacts) penetration=std::max(penetration,c.penetration);
        std::cout<<"Stack speed="<<speed<<" penetration="<<penetration<<'\n';
        check(speed<3 && penetration<0.5,"eight-ball stack settles");
    }
    {
        World w; w.gravityEnabled=false; w.boxes={{400,400,200,18}};
        w.add(makeCircle({500,350},8)); w.bodies[0].velocity={0,1800};
        for(int n=0;n<30;++n) w.step();
        check(w.bodies[0].position.y<400,"fast small ball does not cross platform");
    }
    {
        World w; w.reset();
        for(int n=0;n<600;++n) w.step();
        bool finite=true;
        for(auto& b:w.bodies) finite &= std::isfinite(b.position.x) && std::isfinite(b.position.y)
            && std::isfinite(b.angularVelocity) && b.position.y<=w.height-b.radius+0.5;
        check(finite,"default scene remains finite and contained for five seconds");
        w.dragged=0; w.mouse={1000,200};
        for(int n=0;n<60;++n) w.step();
        check(length(w.bodies[0].velocity)<=1800.001,"mouse spring respects speed limit");
        w.erase(0);
        check(w.dragged==-1 && w.contacts.empty(),"deletion clears selection and contacts");
        for(int n=0;n<600;++n) w.add(makeCircle({300,200},8));
        check(w.bodies.size()==World::maxBodies,"body count is bounded");
    }
}
