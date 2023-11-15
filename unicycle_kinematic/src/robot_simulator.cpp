#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"

class Robot_simulator {
    public: 
        Robot_simulator(void);

        void execute_Inputs_Callback(const std_msgs::Float64MultiArray::ConstPtr& msg);

        void run();

    private:
        ros::NodeHandle nh;
        ros::Subscriber sub;
        ros::Publisher pub;
};

Robot_simulator::Robot_simulator() {
    this->sub = this->nh.subscribe("values", 1, &Robot_simulator::execute_Inputs_Callback, this);
    this->pub = this->nh.advertise<std_msgs::Float64MultiArray>("state", 1);
}

void Robot_simulator::execute_Inputs_Callback(const std_msgs::Float64MultiArray::ConstPtr& msg) {
    
}
