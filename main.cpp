#include <iostream>
#include <vector>
#include <stdexcept>
#include <cmath>

using namespace std;

//делаем axis aligned bouncing boxes
struct AABB{
    vector<double> max_position;
    vector<double> min_position;
};

struct Circle{
    double radius;
    vector<double> position;
    vector<double> velocity;
    double soprtivlenie;
    double mass;
};

//сейчас прописываем функцию обнаружения столкновений в реальном времени
bool detect_collision(const AABB& target_1, const AABB& target_2){
    if(target_1.min_position.size()!=2 || target_1.max_position.size()!=2 || target_2.min_position.size()!=2 || target_2.max_position.size()!=2) throw invalid_argument("Ваш вектор не заполнен");
    if(target_1.max_position[0] < target_2.min_position[0] || target_1.min_position[0] > target_2.max_position[0]) return false;
    if(target_1.max_position[1] < target_2.min_position[1] || target_1.min_position[1] > target_2.max_position[1]) return false;
    return true;
}

//функция подсчета расстояния между кругами
double distance_squared(const Circle& target_1, const Circle& target_2){
    if(target_1.position.size()!=2 || target_2.position.size()!=2) throw invalid_argument("Ваш вектор не заполнен");
    return pow(target_1.position[0] - target_2.position[0], 2) + pow(target_1.position[1] - target_2.position[1], 2);
}

//функция 
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
    vector<double> personal_velocity;
    if(target_1.mass <= 0 || target_2.mass <= 0) throw invalid_argument("Масса должна быть > 0");
    if(target_1.position.size()!=2 || target_2.position.size()!=2 || target_1.velocity.size()!=2 || target_2.velocity.size()!=2) throw invalid_argument("Ваш вектор не заполнен");
    personal_velocity.push_back(target_2.velocity[0] - target_1.velocity[0]);
    personal_velocity.push_back(target_2.velocity[1] - target_1.velocity[1]);
    double dx = target_2.position[0] - target_1.position[0];
    double dy = target_2.position[1] - target_1.position[1];
    double len_2 = dx*dx + dy * dy;
    if (len_2 == 0.0) return;
    double len = sqrt(len_2);
    vector<double> normal = {dx/len, dy/len};
    double velocity_align_normal = dotproduct(personal_velocity, normal);
    if(velocity_align_normal > 0) return;
    double uprugost = min(target_1.soprtivlenie, target_2.soprtivlenie);
    double impuls_power = -(1 + uprugost) * velocity_align_normal;
    impuls_power /= 1 / target_1.mass + 1 / target_2.mass;
    vector<double> impulse;
    impulse.push_back(normal[0] * impuls_power);
    impulse.push_back(normal[1] * impuls_power);
    target_1.velocity[0] -= (1.0 / target_1.mass) * impulse[0];
    target_1.velocity[1] -= (1.0 / target_1.mass) * impulse[1];
    target_2.velocity[0] += (1.0 / target_2.mass) * impulse[0];
    target_2.velocity[1] += (1.0 / target_2.mass) * impulse[1];
}

int main(){
    return 0;
}
