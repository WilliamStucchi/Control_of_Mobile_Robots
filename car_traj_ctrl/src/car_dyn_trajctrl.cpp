#include "car_traj_ctrl/car_dyn_trajctrl.h"

#include <unistd.h>


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

    FullParamName = ros::this_node::getName()+"/x_max_error";
    if (false == Handle.getParam(FullParamName, x_max_error))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());
    
    FullParamName = ros::this_node::getName()+"/y_max_error";
    if (false == Handle.getParam(FullParamName, y_max_error))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    FullParamName = ros::this_node::getName()+"/print_error";
    if (false == Handle.getParam(FullParamName, print_error))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    // Trajectory selector
    FullParamName = ros::this_node::getName()+"/traj_sel";
    if (false == Handle.getParam(FullParamName, traj_sel))
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
    compute_max_error(x_max_error, xPref-xP);

    pi_controller(vPy, yPref, yP, Ts, Tiy, saved_I_y, dyref, Kpy, y_old_ref, y_old);
    compute_max_error(y_max_error, yPref-yP);

    if(print_error > 0.0) {
        ROS_INFO("Max Error X: %f", x_max_error);
        ROS_INFO("Max Error Y: %f", y_max_error);
        print_error = 0.0;
    }
    
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
    double t = ros::Time::now().toSec();

    if(traj_sel == 1) {
        if (t <= 3.0) {
            xref = t;
            dxref = 1.0;
            yref = 0.0;
            dyref = 0.0;
        } else {
            xref = 3*t;
            dxref = 3.0;
            yref = 0.0;
            dyref = 0.0;
        }
        
    } else if(traj_sel == 2){
         if (t <= 3.0) {
            xref = t;
            dxref = 1.0;
            yref = 0.0;
            dyref = 0.0;
        } else {
            xref = t;
            dxref = 1.0;
            yref = 1.0;
            dyref = 0.0;
        }

    } else {
        /* 8-shaped trajectory generation */
        // Trajectory parameters 
        const double a = 2.0;
        const double w = (2 * M_PI) / T;
            
        xref    = a*std::sin(w*t);
        dxref   = w*a*std::cos(w*t);
        yref    = a*std::sin(w*t)*std::cos(w*t);
        dyref   = w*a*(std::pow(std::cos(w*t),2.0)-std::pow(std::sin(w*t),2.0));
    }
    
}

void car_dyn_trajctrl::pi_controller(
    double& vP, double Pref, double P, double Ts, double Ti, double& saved_I, double dref, double Kp, double& old_ref, double& old) {

    // Compute error
    double err_P = (Pref-P);//-(old_ref-old);

    // Update the integral error, with Forward Euler
    saved_I += err_P * Ts / Ti;

    // FF + PI control action
    vP = dref + Kp*(err_P + saved_I);

    old_ref = Pref;
    old = P;
}

void car_dyn_trajctrl::compute_max_error(double& maxError, double error) {
    if(std::abs(error) > std::abs(maxError)) {
        maxError = error;
        print_error = 1.0;
    }
}