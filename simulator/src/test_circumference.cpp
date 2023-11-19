#include "unicycle_kin_sim/test_circumference.h"

void test_circumference::Prepare(void)
{
    /* ROS topics */
    vehicleCommand_publisher = Handle.advertise<std_msgs::Float64MultiArray>("/cmd", 1);

    /* Retrieve parameters from ROS parameter server */
    std::string FullParamName;

    // Trajectory parameters
    FullParamName = ros::this_node::getName()+"/period";
    if (false == Handle.getParam(FullParamName, period))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    FullParamName = ros::this_node::getName()+"/radius";
    if (false == Handle.getParam(FullParamName, radius))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    /* Initialize node state */
    RunPeriod = RUN_PERIOD_DEFAULT;

    linear_velocity = angular_velocity = 0.0;

    ROS_INFO("Node %s ready to run.", ros::this_node::getName().c_str());
}

void test_circumference::RunPeriodically(float Period)
{
    ros::Rate LoopRate(1.0/Period);

    // Wait other nodes start
    sleep(1.5);

    ROS_INFO("Node %s running periodically (T=%.2fs, f=%.2fHz).", ros::this_node::getName().c_str(), Period, 1.0/Period);

    while (ros::ok())
    {
        PeriodicTask();

        ros::spinOnce();

        LoopRate.sleep();
    }
}

void test_circumference::Shutdown(void)
{
    ROS_INFO("Node %s shutting down.", ros::this_node::getName().c_str());
}

void test_circumference::PeriodicTask(void)
{
    // Circular trajectory computation 
    const double w = 2 * M_PI / period;
    const double R = radius;
    xref    = R*(std::cos(w*ros::Time::now().toSec())-1);
    dxref   = -R*w*std::sin(w*ros::Time::now().toSec());
    ddxref  = -R*std::pow(w,2.0)*std::cos(w*ros::Time::now().toSec());
    yref    = R*std::sin(w*ros::Time::now().toSec());
    dyref   = R*w*std::cos(w*ros::Time::now().toSec());
    ddyref  = -R*std::pow(w,2.0)*std::sin(w*ros::Time::now().toSec());
    

    /* Vehicle commands */
    // Flatness transformation
    linear_velocity  = std::sqrt(std::pow(dxref,2.0)+std::pow(dyref,2.0));
    angular_velocity = (dxref*ddyref-dyref*ddxref)/(std::pow(dxref,2.0)+std::pow(dyref,2.0));

    /* Publishing vehicle commands (t, msg->data[0]; velocity, msg->data[1]; steer, msg->data[2]) */
    std_msgs::Float64MultiArray msg;
    msg.data.push_back(ros::Time::now().toSec());
    msg.data.push_back(linear_velocity);
    msg.data.push_back(angular_velocity);
    vehicleCommand_publisher.publish(msg);
}
