#include <queue>
#include <boost/numeric/odeint.hpp>

using namespace boost::numeric::odeint;

typedef std::vector<double> state_type;

// Simulation of a unicycle dynamic model
// Control inputs: longitudinal force, steering torque

class unicycle_dyn_ode
{
public:
    unicycle_dyn_ode(double deltaT);

    void setInitialState(double x0, double y0, double theta0, double v0, double omega0);
    void setVehicleParams(double m, double I);

    void integrate();
    
    void setReferenceCommands(double force, double torque);
    
    void getPose(double &x, double &y, double &theta) { x = state[0]; y = state[1]; theta = state[2]; };
    void getVelocity(double &linear_velocity, double &angular_velocity) { linear_velocity = state[3]; angular_velocity = state[4]; }
    void getCommands(double &force, double &torque) { force = tau1; torque = tau2; };
    void getTime(double &time) { time = t; };

private:
    // Simulator and integrator variables
    double t, dt;
    double tau1, tau2;
    double m, I;

    bool vehicleParams_set;

    state_type state;
    runge_kutta_dopri5 < state_type > stepper;

    // ODE function
    void vehicle_ode(const state_type &state, state_type &dstate, double t);
};
