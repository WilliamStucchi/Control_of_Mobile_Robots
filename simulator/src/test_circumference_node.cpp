#include "unicycle_kin_sim/test_circumference.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, NAME_OF_THIS_NODE);
  
  test_circumference test_circumference_node;
   
  test_circumference_node.Prepare();
  
  test_circumference_node.RunPeriodically(test_circumference_node.RunPeriod);
  
  test_circumference_node.Shutdown();
  
  return (0);
}

