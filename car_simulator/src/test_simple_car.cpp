#include "car_simulator/test_simple_car.h"

void test_simple_car::Prepare(void)
{
    /* ROS topics */
    vehicleCommand_publisher = Handle.advertise<std_msgs::Float64MultiArray>("/car_input", 1);

    /* Initialize node state */
    RunPeriod = RUN_PERIOD_DEFAULT;

    steer = speed = 0.0;

    ROS_INFO("Node %s ready to run.", ros::this_node::getName().c_str());
}

void test_simple_car::RunPeriodically(float Period)
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

void test_simple_car::Shutdown(void)
{
    ROS_INFO("Node %s shutting down.", ros::this_node::getName().c_str());
}

void test_simple_car::PeriodicTask(void)
{
    /* Vehicle commands */
    if (ros::Time::now().toSec()<=3.0)
    {
        speed = 1.0;
        steer = 0.0;
    }
    else if (ros::Time::now().toSec()>3.0 && ros::Time::now().toSec()<=3.5)
    {
        speed = 1.0;
        steer = 0.05;
    }
    else if (ros::Time::now().toSec()>3.5 && ros::Time::now().toSec()<=4)
    {
        speed = 1.0;
        steer = -0.05;
    }
    else {
        speed = 1.0;
        steer = 0.0;
    }

    /* Publishing vehicle commands (t, msg->data[0]; velocity, msg->data[1]; steer, msg->data[2]) */
    std_msgs::Float64MultiArray msg;
    msg.data.push_back(ros::Time::now().toSec());
    msg.data.push_back(speed);
    msg.data.push_back(steer);
    vehicleCommand_publisher.publish(msg);
}
