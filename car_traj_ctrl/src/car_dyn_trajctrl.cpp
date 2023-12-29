#include "car_traj_ctrl/car_dyn_trajctrl.h"

#include <unistd.h>
#include <cmath>


void car_dyn_trajctrl::Prepare(void)
{
    /* Retrieve parameters from ROS parameter server */
    std::string FullParamName;

    // run_period
    FullParamName = ros::this_node::getName()+"/run_period";
    if (false == Handle.getParam(FullParamName, RunPeriod))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    // Controller parameters
    FullParamName = ros::this_node::getName()+"/P_dist";
    if (false == Handle.getParam(FullParamName, P_dist))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    FullParamName = ros::this_node::getName()+"/l";
    if (false == Handle.getParam(FullParamName, l))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    FullParamName = ros::this_node::getName()+"/Kpx";
    if (false == Handle.getParam(FullParamName, Kpx))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    FullParamName = ros::this_node::getName()+"/Kpy";
    if (false == Handle.getParam(FullParamName, Kpy))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    FullParamName = ros::this_node::getName()+"/x_old";
    if (false == Handle.getParam(FullParamName, x_old))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    FullParamName = ros::this_node::getName()+"/y_old";
    if (false == Handle.getParam(FullParamName, y_old))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());
    
    FullParamName = ros::this_node::getName()+"/x_old_ref";
    if (false == Handle.getParam(FullParamName, x_old_ref))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());
    
    FullParamName = ros::this_node::getName()+"/y_old_ref";
    if (false == Handle.getParam(FullParamName, y_old_ref))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    FullParamName = ros::this_node::getName()+"/T";
    if (false == Handle.getParam(FullParamName, T))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());
    
    FullParamName = ros::this_node::getName()+"/Tix";
    if (false == Handle.getParam(FullParamName, Tix))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    FullParamName = ros::this_node::getName()+"/Tiy";
    if (false == Handle.getParam(FullParamName, Tiy))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    FullParamName = ros::this_node::getName()+"/Ts";
    if (false == Handle.getParam(FullParamName, Ts))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    FullParamName = ros::this_node::getName()+"/saved_I_x";
    if (false == Handle.getParam(FullParamName, saved_I_x))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    FullParamName = ros::this_node::getName()+"/saved_I_y";
    if (false == Handle.getParam(FullParamName, saved_I_y))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    FullParamName = ros::this_node::getName()+"/x_maxError";
    if (false == Handle.getParam(FullParamName, x_maxError))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());
    
    FullParamName = ros::this_node::getName()+"/y_maxError";
    if (false == Handle.getParam(FullParamName, y_maxError))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    /* ROS topics */
    vehicleState_subscriber = Handle.subscribe("/car_state", 1, &car_dyn_trajctrl::vehicleState_MessageCallback, this);
    vehicleCommand_publisher = Handle.advertise<std_msgs::Float64MultiArray>("/car_input", 1);
    controllerState_publisher = Handle.advertise<std_msgs::Float64MultiArray>("/controller_state", 1);

    /* Create controller class */
    controller = new car_dyn_fblin(P_dist);

    // Initialize controller parameters
    controller->set_carParam(l);

    ROS_INFO("Node %s ready to run.", ros::this_node::getName().c_str());
}

void car_dyn_trajctrl::RunPeriodically(float Period)
{
    ros::Rate LoopRate(1.0/Period);

    ROS_INFO("Node %s running periodically (T=%.2fs, f=%.2fHz).", ros::this_node::getName().c_str(), Period, 1.0/Period);

    while (ros::ok())
    {
        PeriodicTask();

        ros::spinOnce();

        LoopRate.sleep();
    }
}

void car_dyn_trajctrl::Shutdown(void)
{
    // Delete controller object
    delete controller;

    ROS_INFO("Node %s shutting down.", ros::this_node::getName().c_str());
}

void car_dyn_trajctrl::vehicleState_MessageCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    // Input command: t, msg->data[0]; x, msg->data[1]; y, msg->data[2]; theta, msg->data[3];
    // linear velocity, msg->data[4]; angular velocity, msg->data[5]
    /*  Set vehicle state */
    controller->set_carState(msg->data.at(1), msg->data.at(2), msg->data.at(3));
}

void car_dyn_trajctrl::PeriodicTask(void)
{
    // Trajectory computation
    compute_trajectory(xref, dxref, yref, dyref);

    /*  Compute the control action */
    // Transform trajectory to point P
    controller->reference_transformation(xref, yref, xPref, yPref);
    controller->output_transformation(xP, yP);

    pi_controller(vPx, xPref, xP, Ts, Tix, saved_I_x, dxref, Kpx, x_old_ref, x_old);
    compute_max_error(x_maxError, xPref-xP);
    ROS_INFO("Max Error X: %f", x_maxError);

    pi_controller(vPy, yPref, yP, Ts, Tiy, saved_I_y, dyref, Kpy, y_old_ref, y_old);
    compute_max_error(y_maxError, yPref-yP);
    ROS_INFO("Max Error Y: %f", y_maxError);

    // Linearization law
    controller->control_transformation(vPx, vPy, v, phi);

    /*  Publish vehicle commands */
    std_msgs::Float64MultiArray vehicleCommandMsg;
    vehicleCommandMsg.data.push_back(ros::Time::now().toSec());
    vehicleCommandMsg.data.push_back(v);
    vehicleCommandMsg.data.push_back(phi);
    vehicleCommand_publisher.publish(vehicleCommandMsg);

    /*  Publish controller state */
    std_msgs::Float64MultiArray controllerStateMsg;
    controllerStateMsg.data.push_back(ros::Time::now().toSec());
    controllerStateMsg.data.push_back(xref);
    controllerStateMsg.data.push_back(yref);
    controllerStateMsg.data.push_back(xPref);
    controllerStateMsg.data.push_back(yPref);
    controllerStateMsg.data.push_back(xP);
    controllerStateMsg.data.push_back(yP);
    controllerStateMsg.data.push_back(vPx);
    controllerStateMsg.data.push_back(vPy);
    controllerStateMsg.data.push_back(v);
    controllerStateMsg.data.push_back(phi);
    controllerState_publisher.publish(controllerStateMsg);
}


void car_dyn_trajctrl::compute_trajectory(double& xref, double& dxref, double& yref, double& dyref) {
     /* 8-shaped trajectory generation */
    // Trajectory parameters (these parameters should be moved to the parameter server)
    const double a = 2.0;
    const double w = (2 * M_PI) / T;
        
    xref    = a*std::sin(w*ros::Time::now().toSec());
    dxref   = w*a*std::cos(w*ros::Time::now().toSec());
    yref    = a*std::sin(w*ros::Time::now().toSec())*std::cos(w*ros::Time::now().toSec());
    dyref   = w*a*(std::pow(std::cos(w*ros::Time::now().toSec()),2.0)-std::pow(std::sin(w*ros::Time::now().toSec()),2.0));
    
}

void car_dyn_trajctrl::pi_controller(
    double& vP, double Pref, double P, double Ts, double Ti, double& saved_I, double dref, double Kp, double& old_ref, double& old) {

    // Compute error
    double err_P = (Pref-P);//-(old_ref-old);

    // Update the integral error, with Forward Euler
    saved_I += err_P * Ts / Ti;

    // PI + FF control action
    vP = dref + Kp*(err_P + saved_I);

    old_ref = Pref;
    old = P;
}

void car_dyn_trajctrl::compute_max_error(double& maxError, double error) {
    if(error > maxError) {
        maxError = error;
    }
}