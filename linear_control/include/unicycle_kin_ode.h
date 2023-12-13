#include <queue>
#include <boost/numeric/odeint.hpp>

using namespace boost::numeric::odeint;

typedef std::vector<double> state_type;

// Simulation of a unicycle kiematic model
// Control inputs: linear velocity, angular velocity

class unicycle_kin_ode
{
public:
    unicycle_kin_ode(double deltaT);

    void setInitialState(double x0, double y0, double theta0);

    void integrate();
    
    void setReferenceCommands(double linear_velocity, double angular_velocity);
    
    void getPose(double &x, double &y, double &theta) { x = state[0]; y = state[1]; theta = state[2]; };
    //void getActPose(double &x, double &y, double &theta) { x = act_state[0]; y = act_state[1]; theta = act_state[2]; };
    void getCommands(double &linear_velocity, double &angular_velocity) { linear_velocity = V; angular_velocity = omega; };
    void getTime(double &time) { time = t; };

private:
    // Simulator and integrator variables
    double t, dt;
    double V, omega;

    state_type state;
    //state_type act_state;
    runge_kutta_dopri5 < state_type > stepper;
    //runge_kutta_dopri5 < state_type > act_stepper;

    // ODE function
    void vehicle_ode(const state_type &state, state_type &dstate, double t);
   // void vehicle_ode_act(const state_type &act_state, state_type &act_dstate, double t);
};
