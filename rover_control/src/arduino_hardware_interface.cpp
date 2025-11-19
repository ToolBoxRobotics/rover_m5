// ros_control hardware interface skeleton
#include <ros/ros.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>

class RoverHW : public hardware_interface::RobotHW {
public:
  RoverHW() {}
  void init() {}
  void read() {}
  void write() {}
};

int main(int argc, char** argv){
  ros::init(argc, argv, "rover_hw_interface");
  ros::NodeHandle nh;
  RoverHW robot;
  robot.init();
  controller_manager::ControllerManager cm(&robot, nh);
  ros::Rate rate(50);
  while(ros::ok()){
    robot.read();
    cm.update(ros::Time::now(), ros::Duration(0.02));
    robot.write();
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}
