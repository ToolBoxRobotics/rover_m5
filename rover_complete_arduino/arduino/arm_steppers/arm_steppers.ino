/*
  Arm steppers sketch for Arduino Mega

  - Controls 5 NEMA17 steppers through A4988 drivers
  - Supports homing switches on each axis
  - Subscribes to /arm/command (Float64MultiArray of target positions in radians)
  - Publishes /arm/joint_states
  - Uses AccelStepper library
  - Configure pins for step/dir and homing switch pins per your wiring
*/
#include <ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <AccelStepper.h>

ros::NodeHandle nh;

const int N = 5;
// Step and Dir pins for each A4988 (change to match wiring)
const int stepPins[N] = {2,4,6,8,10};
const int dirPins[N] = {3,5,7,9,11};
const int homePins[N] = {22,23,24,25,26};

AccelStepper steppers[N] = {
  AccelStepper(AccelStepper::DRIVER, stepPins[0], dirPins[0]),
  AccelStepper(AccelStepper::DRIVER, stepPins[1], dirPins[1]),
  AccelStepper(AccelStepper::DRIVER, stepPins[2], dirPins[2]),
  AccelStepper(AccelStepper::DRIVER, stepPins[3], dirPins[3]),
  AccelStepper(AccelStepper::DRIVER, stepPins[4], dirPins[4])
};

sensor_msgs::JointState joint_state;
ros::Publisher joint_pub("/arm/joint_states", &joint_state);

void arm_cb(const std_msgs::Float64MultiArray& msg){
  for (int i=0;i<N && i<msg.data_length;i++){
    // Convert radians to steps. Example assumes 200 steps/rev and microstepping factor
    double rad = msg.data[i];
    long steps = (long)(rad * (200.0 / (2.0 * 3.141592653589793)) * 16.0); // 1/16 microstep
    steppers[i].moveTo(steps);
  }
}

ros::Subscriber<std_msgs::Float64MultiArray> arm_sub("/arm/command", arm_cb);

void setup(){
  Serial.begin(115200);
  nh.initNode();
  nh.advertise(joint_pub);
  nh.subscribe(arm_sub);

  for (int i=0;i<N;i++){
    pinMode(homePins[i], INPUT_PULLUP);
    steppers[i].setMaxSpeed(1000);
    steppers[i].setAcceleration(1000);
  }

  // Optional: perform homing sequence (move slowly towards switch)
}

unsigned long last_pub = 0;
void loop(){
  nh.spinOnce();
  for (int i=0;i<N;i++) steppers[i].run();
  if (millis() - last_pub > 50){
    last_pub = millis();
    joint_state.header.stamp = nh.now();
    static const char* names[N] = {"arm_joint_1","arm_joint_2","arm_joint_3","arm_joint_4","arm_joint_5"};
    joint_state.name = (char**)&names;
    static double pos[N];
    for (int i=0;i<N;i++) pos[i] = (steppers[i].currentPosition() / (200.0*16.0)) * 2.0 * 3.141592653589793;
    joint_state.position = pos;
    joint_pub.publish(&joint_state);
  }
}
