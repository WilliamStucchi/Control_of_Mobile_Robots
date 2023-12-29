#include "car_dyn_fblin.h"

#include <cmath>
#include <stdexcept>


car_dyn_fblin::car_dyn_fblin(double P_dist)
{
    this->P_dist  = P_dist;

    // Initialize robot position
    x = y = theta = 0.0;
}

car_dyn_fblin::~car_dyn_fblin()
{
    // Do nothing
}

void car_dyn_fblin::control_transformation(double vPx, double vPy, double& v, double& phi)
{
    v = vPx*std::cos(theta)+vPy*std::sin(theta);

    if (std::fabs(v)>0.01)
    {
        phi = std::atan(l/(v*P_dist)*(vPy*std::cos(theta)-vPx*std::sin(theta)));
    }
    else
    {
        phi = std::atan(l/(0.01*P_dist)*(vPy*std::cos(theta)-vPx*std::sin(theta)));
    }
}

void car_dyn_fblin::output_transformation(double& xP, double& yP)
{
    xP = x + P_dist*std::cos(theta);
    yP = y + P_dist*std::sin(theta);
}

void car_dyn_fblin::reference_transformation(double xref, double yref, double& xPref, double& yPref)
{
    xPref = xref + P_dist*std::cos(theta);
    yPref = yref + P_dist*std::sin(theta);
}