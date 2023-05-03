#include <kinematics.h>

ForwardKinematics::ForwardKinematics(const float a1, const float a2, const float a3) :
    _a1{a1}, _a2{a2}, _a3{a3} {}

Vector5f ForwardKinematics::evaluate(const Eigen::Vector3f configuration) {
    float th1 = deg_to_rad(configuration[0]);
    float th2 = deg_to_rad(configuration[1]);
    float th23 = deg_to_rad(configuration[1] + configuration[2]);

    float c1 = cos(th1);
    float s1 = sin(th1);

    float c2 = cos(th2);
    float s2 = sin(th2);

    float c23 = cos(th23);
    float s23 = sin(th23);

    float aux1 = (_a3 * c23 + _a2 * c2 + _a1);

    Vector5f pose;
    pose[0] = c1 * aux1;
    pose[1] = s1 * aux1;
    pose[2] = _a3 * s23 + _a2 * s2;
    pose[3] = rad_to_deg(th1);
    pose[4] = rad_to_deg(th23);

    return pose;
}