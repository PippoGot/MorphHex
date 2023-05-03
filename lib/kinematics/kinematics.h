#pragma once

#include <Eigen/Dense>
#include <cmath>

typedef Eigen::Matrix<float, 5, 1> Vector5f;

#define deg_to_rad(angle_deg) ((angle_deg)*M_PI / 180.0)
#define rad_to_deg(angle_rad) ((angle_rad)*180.0 / M_PI)

class ForwardKinematics {
   protected:
    const float _a1;
    const float _a2;
    const float _a3;

   public:
    ForwardKinematics(const float a1, const float a2, const float a3);

    Vector5f evaluate(const Eigen::Vector3f configuration);
};