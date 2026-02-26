#include <iostream>
#include <vector>
#include <stdexcept>
#include <cmath>
#include <algorithm>

using namespace std;

//делаем axis aligned bounding boxes
struct AABB{
    vector<double> max_position;
    vector<double> min_position;
};

struct Circle{
    AABB aabb;
    double radius;
    vector<double> position;
    vector<double> velocity;
    double soprtivlenie; // по смыслу это restitution (упругость)
    double mass;
    double inv_mass;
};

//сейчас прописываем функцию обнаружения столкновений в реальном времени
bool detect_collision(const AABB& target_1, const AABB& target_2){
    if(target_1.min_position.size()!=2 || target_1.max_position.size()!=2 || target_2.min_position.size()!=2 || target_2.max_position.size()!=2)
        throw invalid_argument("Ваш вектор не заполнен");
    if(target_1.max_position[0] < target_2.min_position[0] || target_1.min_position[0] > target_2.max_position[0]) return false;
    if(target_1.max_position[1] < target_2.min_position[1] || target_1.min_position[1] > target_2.max_position[1]) return false;
    return true;
}

//функция подсчета расстояния между кругами
double distance_squared(const Circle& target_1, const Circle& target_2){
    if(target_1.position.size()!=2 || target_2.position.size()!=2) throw invalid_argument("Ваш вектор не заполнен");
    const double dx = target_1.position[0] - target_2.position[0];
    const double dy = target_1.position[1] - target_2.position[1];
    return dx*dx + dy*dy;
}

bool circles_detect_collision(const Circle& target_1, const Circle& target_2){
    double r = target_1.radius + target_2.radius;
    r *= r;
    return r >= distance_squared(target_1, target_2);
}

double dotproduct(const vector<double> &vec_1, const vector<double> &vec_2){
    if(vec_1.size()  != 2 || vec_2.size() != 2) throw invalid_argument("Ваш вектор не заполнен");
    return vec_1[0] * vec_2[0] + vec_1[1] * vec_2[1];
}

void process_collision(Circle& target_1, Circle& target_2){
    if(target_1.mass <= 0 || target_2.mass <= 0) throw invalid_argument("Масса должна быть > 0");
    if(target_1.position.size()!=2 || target_2.position.size()!=2 || target_1.velocity.size()!=2 || target_2.velocity.size()!=2)
        throw invalid_argument("Ваш вектор не заполнен");
    vector<double> personal_velocity = {
        target_2.velocity[0] - target_1.velocity[0],
        target_2.velocity[1] - target_1.velocity[1]
    };

    const double dx = target_2.position[0] - target_1.position[0];
    const double dy = target_2.position[1] - target_1.position[1];
    const double len_2 = dx*dx + dy*dy;
    if (len_2 <= 1e-12) return;
    const double len = sqrt(len_2);
    vector<double> normal = {dx/len, dy/len};
    const double velocity_align_normal = dotproduct(personal_velocity, normal);
    if(velocity_align_normal > 0) return;
    const double uprugost = min(target_1.soprtivlenie, target_2.soprtivlenie);
    double impuls_power = -(1.0 + uprugost) * velocity_align_normal;
    impuls_power /= (1.0 / target_1.mass) + (1.0 / target_2.mass);
    vector<double> impulse = { normal[0] * impuls_power, normal[1] * impuls_power };
    target_1.velocity[0] -= (1.0 / target_1.mass) * impulse[0];
    target_1.velocity[1] -= (1.0 / target_1.mass) * impulse[1];
    target_2.velocity[0] += (1.0 / target_2.mass) * impulse[0];
    target_2.velocity[1] += (1.0 / target_2.mass) * impulse[1];
}

void inverting_mass(Circle& target){
    if(target.mass == 0)
        target.inv_mass = 0;
    else
        target.inv_mass = 1.0 / target.mass;
}

double compute_penetration(const Circle& target_1, const Circle& target_2){
    const double dist = sqrt(distance_squared(target_1, target_2));
    return max(0.0, (target_1.radius + target_2.radius) - dist);
}

void positional_correction(Circle& target_1, Circle& target_2){
    const double precent = 0.2;
    const double slope = 0.01; 
    inverting_mass(target_1);
    inverting_mass(target_2);
    const double dx = target_2.position[0] - target_1.position[0];
    const double dy = target_2.position[1] - target_1.position[1];
    const double len_2 = dx*dx + dy*dy;
    if (len_2 <= 1e-12) return;
    const double len = sqrt(len_2);
    vector<double> normal = {dx/len, dy/len};
    const double penetration = compute_penetration(target_1, target_2);
    const double corr = max(penetration - slope, 0.0);
    if ((target_1.inv_mass + target_2.inv_mass) <= 0.0) return;
    vector<double> correction = {
        (corr / (target_1.inv_mass + target_2.inv_mass) * precent) * normal[0],
        (corr / (target_1.inv_mass + target_2.inv_mass) * precent) * normal[1]
    };
    target_1.position[0] -= correction[0] * target_1.inv_mass;
    target_1.position[1] -= correction[1] * target_1.inv_mass;
    target_2.position[0] += correction[0] * target_2.inv_mass;
    target_2.position[1] += correction[1] * target_2.inv_mass;
}

struct manifold{
    Circle *target_1;
    Circle *target_2;
    float penetration;
    vector<double> normal;
};

double LengthSquared(const vector<double> &vec){
    if (vec.size() != 2) throw invalid_argument("Ваш вектор не заполнен");
    return vec[0] * vec[0] + vec[1] * vec[1];
}

bool circle_vs_circle(manifold *m){
    if (!m || !m->target_1 || !m->target_2) throw invalid_argument("Ошибка");
    Circle *target_1 = m->target_1;
    Circle *target_2 = m->target_2;
    if(target_1->position.size()!=2 || target_2->position.size()!=2) throw invalid_argument("Ваш вектор не заполнен");
    const double nx = target_2->position[0] - target_1->position[0];
    const double ny = target_2->position[1] - target_1->position[1];
    const double sumR = target_1->radius + target_2->radius;
    const double dist2 = nx*nx + ny*ny;
    if (dist2 > sumR * sumR)
        return false;
    const double dist = sqrt(dist2);
    if (dist > 1e-12) {
        m->penetration = static_cast<float>(sumR - dist);
        m->normal = { nx / dist, ny / dist };
    } else {
        m->penetration = static_cast<float>(target_1->radius);
        m->normal = { 1.0, 0.0 };
    }
    return true;
}

bool AABBvsAABB(manifold *m){
    if (!m || !m -> target_1 || !m -> target_2) throw invalid_argument("ERRROR");
    Circle *target_1 = m -> target_1;
    Circle *target_2 = m -> target_2;
    if (target_1->position.size() != 2 || target_2->position.size() != 2)
        throw invalid_argument("вектор не заполнен");
    if (target_1->aabb.min_position.size() != 2 || target_1->aabb.max_position.size() != 2 || target_2->aabb.min_position.size() != 2 || target_2->aabb.max_position.size() != 2)
        throw invalid_argument("вектор не заполнен");
    vector<double> n = {target_2->position[0] - target_1->position[0], target_2->position[1] - target_1->position[1]};
    AABB abox = target_1 -> aabb;
    AABB bbox = target_2 -> aabb;
    //Считаем наложение для координат по x 
    double a_extent = (abox.max_position[0] - abox.min_position[0]) / 2;
    double b_extent = (bbox.max_position[0] - bbox.min_position[0]) / 2;
    double x_overlap = a_extent + b_extent - abs(n[0]);
    //Делаем аналогичнно для координат по Y
    if (x_overlap > 0){
        double a_extent_y = (abox.max_position[1] - abox.min_position[1]) / 2;
        double b_extent_y = (bbox.max_position[1] - bbox.min_position[1   ]) / 2;
        double y_overlap = a_extent + b_extent - abs(n[1]);
        if (y_overlap > 0){
            if (x_overlap > y_overlap){
                if(n[0] < 0)
                    m -> normal = {-1, 0};
                else
                    m -> normal = {1, 0};
                m -> penetration = x_overlap;
                return true;
            }
            else{
                if(n[1] < 0)
                    m -> normal = {0, -1};
                else
                    m -> normal = {0, 1};
                m -> penetration = y_overlap;
                return true;
            }
        }
    }
    return false;
}

bool AABBvsCircle(manifold *m){
    if (!m || !m -> target_1 || !m -> target_2) throw invalid_argument("ERRROR");
    Circle *target_1 = m -> target_1;
    Circle *target_2 = m -> target_2;
    if (target_1->position.size() != 2 || target_2->position.size() != 2)
        throw invalid_argument("вектор не заполнен");
    if (target_1->aabb.min_position.size() != 2 || target_1->aabb.max_position.size() != 2 || target_2->aabb.min_position.size() != 2 || target_2->aabb.max_position.size() != 2)
        throw invalid_argument("вектор не заполнен");
    vector<double> n = {target_2->position[0] - target_1->position[0], target_2->position[1] - target_1->position[1]};
    vector<double> closest = n;
    AABB abox = target_1 -> aabb;
    AABB bbox = target_2 -> aabb;
    double a_extent = (abox.max_position[0] - abox.min_position[0]) / 2;
    double b_extent = (bbox.max_position[0] - bbox.min_position[0]) / 2;
    closest[0] = max(-a_extent, min(a_extent, closest[0]));
    closest[1] = max(-b_extent, min(b_extent, closest[1]));
    bool inside = false;
    if(n == closest){
        inside = true;
        if(abs(n[0]) > abs(n[1])){
            if(closest[0] > 0)
                closest[0] = a_extent;
            else 
                closest[1] = -a_extent;
        }
        else{
            if(closest[1] > 0)
                closest[1] = b_extent;
            else
                closest[1] = -b_extent;
        }
    }
    vector<double> normal = {n[0] - closest[0], n[1] - closest[1]};
    double d = LengthSquared(normal);
    double r = target_2->radius;
    if(d > r * r && !inside)
        return false;
    d = sqrt(d);
    if(inside){
        m -> normal = {-n[0], -n[1]};
        m -> penetration = r - d;
    }
    else{
        m -> normal = n;
        m -> penetration = r - d;
    }
    return true;
}

int main(){
    return 0;
}
