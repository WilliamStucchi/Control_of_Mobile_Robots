#ifndef CAR_DYN_TRAJCTRL_H_
#define CAR_DYN_TRAJCTRL_H_

#include "ros/ros.h"
#include <std_msgs/Float64MultiArray.h>
#include <car_dyn_fblin.h>

#define NAME_OF_THIS_NODE "car_dyn_trajctrl"


class car_dyn_trajctrl
{
  private: 
    ros::NodeHandle Handle;

    /* ROS topics */
    ros::Subscriber vehicleState_subscriber;
    ros::Publisher vehicleCommand_publisher, controllerState_publisher;

    /* Parameters from ROS parameter server */
    int traj_sel;
    double P_dist, l, Kpx, Kpy;
    double T, Tix, Tiy, Ts;

    /* ROS topic callbacks */
    void vehicleState_MessageCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);

    /* Node periodic task */
    void PeriodicTask(void);

    void compute_trajectory(double& xref, double& dxref, double& yref, double& dyref);
    void pi_controller(double& vP, double Pref, double P, double Ts, double Ti, double& saved_I, double dref, double Kp, double& old_ref, double& old);
    void compute_max_error(double& maxError, double error);

    /* Node state variables */
    car_dyn_fblin* controller;
    double xref, yref, dxref, dyref;
    double xP, yP, xPref, yPref, x_old_ref, y_old_ref, x_old, y_old;
    double vPx, vPy, v, phi;
    double saved_I_x, saved_I_y;
    double x_max_error, y_max_error, print_error;

  public:
    float RunPeriod;

    void Prepare(void);
    
    void RunPeriodically(float Period);
    
    void Shutdown(void);
};

#endif /* CAR_DYN_TRAJCTRL_H_ */
