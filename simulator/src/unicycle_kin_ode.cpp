#include "unicycle_kin_ode.h"

#include <boost/math/special_functions/sign.hpp>

unicycle_kin_ode::unicycle_kin_ode(double deltaT) : dt(deltaT), t(0.0), state(3), act_state(3), V(0.0), omega(0.0)
{
    // state = [ x, y, theta ]

    // Initial state values
    state[0] = 0.0;
    state[1] = 0.0;
    state[2] = 0.0;

    act_state[0] = 0.0;
    act_state[1] = 0.0;
    act_state[2] = 0.0;
}

void unicycle_kin_ode::setInitialState(double x0, double y0, double theta0)
{
    // Initial state values
    state[0] = x0;
    state[1] = y0;
    state[2] = theta0;

    act_state[0] = x0;
    act_state[1] = y0;
    act_state[2] = theta0;
}

void unicycle_kin_ode::setReferenceCommands(double velocity, double angular_velocity)
{
    V     = velocity;
    omega = angular_velocity;
}

void unicycle_kin_ode::integrate()
{
    // Integrate for one step ahead
    using namespace std::placeholders;
    stepper.do_step(std::bind(&unicycle_kin_ode::vehicle_ode, this, _1, _2, _3), state, t, dt);
    act_stepper.do_step(std::bind(&unicycle_kin_ode::vehicle_ode_act, this, _1, _2, _3), act_state, t, dt);


    // Update time and steering
    t += dt;
}

void unicycle_kin_ode::vehicle_ode(const state_type &state, state_type &dstate, double t)
{
    using namespace boost::math;

    // Actual state
    const double x     = state[0];
    const double y     = state[1];
    const double theta = state[2];
    
    // Vehicle equations
    dstate[0] = V*std::cos(theta);    // dx
    dstate[1] = V*std::sin(theta);    // dy
    dstate[2] = omega;                // dtheta
    
}


void unicycle_kin_ode::vehicle_ode_act(const state_type &act_state, state_type &act_dstate, double t)
{
    using namespace boost::math;

    // Actual state
    const double x     = act_state[0];
    const double y     = act_state[1];
    const double theta = act_state[2];

    const double V_actuator = 1;
    double omega_actuator = omega;

    double denominator = (135 * omega + 8100 - std::pow(omega, 2));
    if(denominator != 0)
        omega_actuator = 8100 / denominator;

    // Actuated vehicle equations
    act_dstate[0] = V_actuator * V * std::cos(theta);    // dx
    act_dstate[1] = V_actuator * V * std::sin(theta);    // dy
    act_dstate[2] = omega_actuator * omega;              // dtheta
}