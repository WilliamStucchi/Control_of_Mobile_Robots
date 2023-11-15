#include "unicycle_dyn_ode.h"

#include <boost/math/special_functions/sign.hpp>

unicycle_dyn_ode::unicycle_dyn_ode(double deltaT) : dt(deltaT), t(0.0), state(5), tau1(0.0), tau2(0.0),
    vehicleParams_set(false)
{
    // state = [ r, beta, x, y, psi ]

    // Initial state values
    state[0] = 0.0;
    state[1] = 0.0;
    state[2] = 0.0;
    state[3] = 0.0;
    state[4] = 0.0;
}

void unicycle_dyn_ode::setInitialState(double x0, double y0, double theta0, double v0, double omega0)
{
    // Initial state values
    state[0] = x0;
    state[1] = y0;
    state[2] = theta0;
    state[3] = v0;
    state[4] = omega0;
}

void unicycle_dyn_ode::setVehicleParams(double m, double I)
{
    // Initialize vehicle parameters
    this->m = m;
    this->I = I;

    vehicleParams_set = true;
}

void unicycle_dyn_ode::setReferenceCommands(double force, double torque)
{
    tau1 = force;
    tau2 = torque;
}

void unicycle_dyn_ode::integrate()
{
    // Check vehicle parameters are set
    if (!vehicleParams_set) {
        throw std::invalid_argument( "Vehicle parameters not set!" );
    }

    // Integrate for one step ahead
    using namespace std::placeholders;
    stepper.do_step(std::bind(&unicycle_dyn_ode::vehicle_ode, this, _1, _2, _3), state, t, dt);

    // Update time and steering
    t += dt;
}

void unicycle_dyn_ode::vehicle_ode(const state_type &state, state_type &dstate, double t)
{
    // Actual state
    const double x     = state[0];
    const double y     = state[1];
    const double theta = state[2];
    const double v     = state[3];
    const double omega = state[4];

    // Vehicle equations
    dstate[0] = v*std::cos(theta);      // dx
    dstate[1] = v*std::sin(theta);      // dy
    dstate[2] = omega;                  // dtheta
    dstate[3] = tau1/m;                 // dv
    dstate[4] = tau2/I;                 // domega
}
