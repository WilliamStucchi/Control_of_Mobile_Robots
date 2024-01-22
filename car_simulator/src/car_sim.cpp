#include "car_simulator/car_sim.h"

#include <unistd.h>


void car_sim::Prepare(void)
{
    /* Retrieve parameters from ROS parameter server */
    std::string FullParamName;

    // Simulator parameters
    FullParamName = ros::this_node::getName()+"/tyre_model";
    if (false == Handle.getParam(FullParamName, tyre_model))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    FullParamName = ros::this_node::getName()+"/dt";
    if (false == Handle.getParam(FullParamName, dt))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    // Vehicle parameters
    FullParamName = ros::this_node::getName()+"/m";
    if (false == Handle.getParam(FullParamName, m))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    FullParamName = ros::this_node::getName()+"/cog_a";
    if (false == Handle.getParam(FullParamName, cog_a))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    FullParamName = ros::this_node::getName()+"/cog_b";
    if (false == Handle.getParam(FullParamName, cog_b))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    FullParamName = ros::this_node::getName()+"/mu";
    if (false == Handle.getParam(FullParamName, mu))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    FullParamName = ros::this_node::getName()+"/Cf";
    if (false == Handle.getParam(FullParamName, Cf))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    FullParamName = ros::this_node::getName()+"/Cr";
    if (false == Handle.getParam(FullParamName, Cr))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    FullParamName = ros::this_node::getName()+"/Iz";
    if (false == Handle.getParam(FullParamName, Iz))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    // Vehicle initial state
    FullParamName = ros::this_node::getName()+"/r0";
    if (false == Handle.getParam(FullParamName, r0))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    FullParamName = ros::this_node::getName()+"/beta0";
    if (false == Handle.getParam(FullParamName, beta0))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    FullParamName = ros::this_node::getName()+"/x0";
    if (false == Handle.getParam(FullParamName, x0))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    FullParamName = ros::this_node::getName()+"/y0";
    if (false == Handle.getParam(FullParamName, y0))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    FullParamName = ros::this_node::getName()+"/psi0";
    if (false == Handle.getParam(FullParamName, psi0))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    FullParamName = ros::this_node::getName()+"/Fyf_max";
    if (false == Handle.getParam(FullParamName, Fyf_max))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    FullParamName = ros::this_node::getName()+"/Fyr_max";
    if (false == Handle.getParam(FullParamName, Fyr_max))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    /* ROS topics */
    vehicleCommand_subscriber = Handle.subscribe("/car_input", 1, &car_sim::vehicleCommand_MessageCallback, this);
    vehicleState_publisher = Handle.advertise<std_msgs::Float64MultiArray>("/car_state", 1);
    clock_publisher = Handle.advertise<rosgraph_msgs::Clock>("/clock", 1);

    /* Create simulator class */
    switch (tyre_model)
    {
        case 0: // linear
            simulator = new car_ode(dt, car_ode::LINEAR);
            break;

        case 1: // fiala with saturation
            simulator = new car_ode(dt, car_ode::FIALA_WITH_SATURATION);
            break;

        case 2: // fiala without saturation
            simulator = new car_ode(dt, car_ode::FIALA_WITHOUT_SATURATION);
            break;

        default:
            simulator = new car_ode(dt, car_ode::LINEAR);
            break;
    }

    /* Initialize simulator class */
    simulator->setInitialState(r0, beta0, x0, y0, psi0);
    simulator->setVehicleParams(m, cog_a, cog_b, Cf, Cr, mu, Iz);

    ROS_INFO("Node %s ready to run.", ros::this_node::getName().c_str());
}

void car_sim::RunPeriodically(void)
{
    ROS_INFO("Node %s running.", ros::this_node::getName().c_str());

    // Wait other nodes start
    sleep(1.0);

    while (ros::ok())
    {
        PeriodicTask();

        ros::spinOnce();

        usleep(1000);
    }
}

void car_sim::Shutdown(void)
{
    // Delete ode object
    delete simulator;

    ROS_INFO("Node %s shutting down.", ros::this_node::getName().c_str());
}

void car_sim::vehicleCommand_MessageCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    // Input command: t, msg->data[0]; velocity, msg->data[1]; steer, msg->data[2]
    /*  Set vehicle commands */
    simulator->setReferenceCommands(msg->data.at(1), msg->data.at(2));
}

void car_sim::PeriodicTask(void)
{
    /*  Integrate the model */
    simulator->integrate();

    /*  Extract measurement from simulator */
    double x, y, theta;
    simulator->getPose(x, y, theta);

    double ay, yawrate, vy;
    simulator->getLateralDynamics(ay, yawrate, vy);

    double sideslip;
    simulator->getSideslip(sideslip);

    double slip_front, slip_rear;
    simulator->getSlip(slip_front, slip_rear);

    double force_front, force_rear;
    simulator->getLateralForce(force_front, force_rear);

    if(force_front > Fyf_max) {
        Fyf_max = force_front;
        ROS_INFO("Fyf MAX: %f", Fyf_max);
        ROS_INFO("Fyr MAX: %f", Fyr_max);
    }
    if(force_rear > Fyr_max) {
        Fyr_max = force_rear;
        ROS_INFO("Fyf MAX: %f", Fyf_max);
        ROS_INFO("Fyr MAX: %f", Fyr_max);
    }


    double velocity_act, steer_act;
    simulator->getCommands(velocity_act, steer_act);

    double time;
    simulator->getTime(time);

    /*  Print simulation time every 5 sec */
    if (std::fabs(std::fmod(time,5.0)) < 1.0e-3)
    {
        ROS_INFO("Simulator time: %d seconds", (int) time);
    }

    /*  Publish vehicle state */
    std_msgs::Float64MultiArray vehicleStateMsg;
    vehicleStateMsg.data.push_back(time);
    vehicleStateMsg.data.push_back(x);
    vehicleStateMsg.data.push_back(y);
    vehicleStateMsg.data.push_back(theta);
    vehicleStateMsg.data.push_back(yawrate);
    vehicleStateMsg.data.push_back(vy);
    vehicleStateMsg.data.push_back(ay);
    vehicleStateMsg.data.push_back(sideslip);
    vehicleStateMsg.data.push_back(slip_front);
    vehicleStateMsg.data.push_back(slip_rear);
    vehicleStateMsg.data.push_back(force_front);
    vehicleStateMsg.data.push_back(force_rear);
    vehicleStateMsg.data.push_back(velocity_act);
    vehicleStateMsg.data.push_back(steer_act);
    vehicleState_publisher.publish(vehicleStateMsg);

    /*  Publish clock */
    rosgraph_msgs::Clock clockMsg;
    clockMsg.clock = ros::Time(time);
    clock_publisher.publish(clockMsg);
}
