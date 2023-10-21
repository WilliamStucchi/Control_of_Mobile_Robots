/*  Author: William Stucchi
    Version: 0.0.0
    Date: 21/10/2023
*/

#include "ros/ros.h"
#include "std_msgs/Int32.h"

class Sumup {
  public:

    // constructor
    Sumup(void);

    // this function is called when a message is published on topic /value
    void execute_Value_Callback(const std_msgs::Int32::ConstPtr &msg);

    void run(void);

  private:
    ros::NodeHandle nh;
    ros::Subscriber smp_subscriber;
    ros::Publisher smp_publisher;

    // parameters on the problem
    int ctr;        // how many nodes have submitted their value
    int summation;  // summation of the values read from the topic /value
    int N;          // number of different values of the counter nodes we are waiting
    int alfa;       // parameter of the problem definition
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "sumup");

  // start operations
  Sumup sumup_node;
  sumup_node.run();

  return 0;
}

Sumup::Sumup() {
  // setup publisher and subscriber 
  this->smp_publisher = this->nh.advertise<std_msgs::Int32>("sum", 100);
  this->smp_subscriber = this->nh.subscribe("value", 100, &Sumup::execute_Value_Callback, this);

  this->ctr = 0;
  this->summation = 0;
  this->N = 2;
  this->alfa = 1;
}

void Sumup::execute_Value_Callback(const std_msgs::Int32::ConstPtr &msg) {
  // increase counter (to check how many values this node has received)
  this->ctr++;
  // keeps count of the values received
  this->summation += msg->data;

  // when N values have been received, publish final computations
  if(this->ctr == this->N) {
      int multiplication = this->alfa * this->summation;
      
      std_msgs::Int32 message_value;
      message_value.data = multiplication;

      this->smp_publisher.publish(message_value);
      ROS_INFO("The result is %d", multiplication);
      // std::cout << "The result is " << multiplication;
  }
}

void Sumup::run() {
  ros::spin();
}