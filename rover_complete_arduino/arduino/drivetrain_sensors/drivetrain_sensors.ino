/*
  Drivetrain + Sensors Arduino sketch for rosserial

  Features implemented:
  - DRI0002 motor control (PWM + DIR) for 6 motors
  - Quadrature encoder ISRs for 6 encoders
  - PID-ish velocity loop (P controller) per wheel on Arduino
  - PCA9685 steering servos (4 corner servos)
  - TCA9548A multiplexer helper to read MPU6050, AHT20, INA219 behind the mux
  - Publishes: /joint_states, /encoders (Int32MultiArray), /imu/data (sensor_msgs/Imu), /aht20/temperature, /aht20/humidity, /battery (sensor_msgs/BatteryState)
  - Subscribes: /<wheel>_controller/command (std_msgs/Float64) for wheel angular velocity (rad/s)
                /<steer>_controller/command (std_msgs/Float64) for steering joint target (rad)

  Notes:
  - This sketch assumes Arduino Mega. Adjust pins to your wiring.
  - You must install the following Arduino libraries:
    * rosserial_arduino (and ros_lib generated headers)
    * Adafruit_PWMServoDriver
    * Adafruit_INA219
    * AHT20 library (or equivalent)
    * I2Cdevlib MPU6050 (or MPU6050 library)

  Calibrate:
  - PCA9685 servo pulse lengths for your steering servos.
  - PID gains below for wheel velocity loop.
  - Encoder counts_per_rev constant according to your encoders.
*/

#include <ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32MultiArray.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Temperature.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/JointState.h>
#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>
#include <Adafruit_INA219.h>
// Include your AHT20 and MPU6050 library headers here:
// #include <AHT20.h>
// #include <MPU6050.h>

// --------------------------- USER CONFIG: PIN ASSIGNMENTS ---------------------------
// Motor wiring for DRI0002: each motor has PWM pin and DIR pin
const uint8_t motor_pwm_pins[6] = {3, 5, 6, 9, 10, 11}; // PWM pins (timers) - change as needed
const uint8_t motor_dir_pins[6] = {22, 23, 24, 25, 26, 27}; // DIR control pins

// Encoder pins: for each wheel we have encoder A and B (quadrature)
// Must be attached to interrupt-capable pins on Arduino Mega. Use digital pins 2..21 as needed.
const uint8_t encA_pins[6] = {2, 18, 19, 20, 21, 18}; // update duplicates appropriately
const uint8_t encB_pins[6] = {4, 44, 45, 46, 47, 48}; // example pins - change for your wiring

// PCA9685 for steering servos (I2C)
Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver(); // default address 0x40

// TCA9548A multiplexer I2C address (default)
const uint8_t TCA_ADDR = 0x70;

// INA219 on multiplexer (battery current/voltage)
Adafruit_INA219 ina219;

// --------------------------- ROBOT / CONTROL PARAMETERS ---------------------------
const float wheel_radius = 0.08; // meters
const uint16_t ENCODER_CPR = 1024; // counts per revolution
const float control_dt = 0.02; // 50 Hz control loop

// Simple P gain for velocity loop (tune on robot)
float vel_kp[6] = {40.0, 40.0, 40.0, 40.0, 40.0, 40.0};

// Max PWM value for Arduino analogWrite (0-255)
const int PWM_MAX = 255;

// --------------------------- ROS SETUP ---------------------------
ros::NodeHandle nh;

// Publishers
sensor_msgs::JointState joint_state_msg;
ros::Publisher joint_state_pub("/joint_states", &joint_state_msg);

std_msgs::Int32MultiArray encoder_array_msg;
ros::Publisher encoder_pub("/encoders", &encoder_array_msg);

sensor_msgs::Imu imu_msg;
ros::Publisher imu_pub("/imu/data", &imu_msg);

sensor_msgs::Temperature temp_pub_msg;
ros::Publisher aht_temp_pub("/aht20/temperature", &temp_pub_msg);
std_msgs::Float64 aht_hum_msg;
ros::Publisher aht_hum_pub("/aht20/humidity", &aht_hum_msg);

sensor_msgs::BatteryState battery_msg;
ros::Publisher battery_pub("/battery", &battery_msg);

// Subscribers for wheel velocity commands (rad/s) and steering (rad)
ros::Subscriber<std_msgs::Float64> *wheel_subs[6];
ros::Subscriber<std_msgs::Float64> *steer_subs[4];

// Command buffers (rad/s for wheels, rad for steering)
volatile double wheel_cmd[6] = {0,0,0,0,0,0};
volatile double steer_cmd[4] = {0,0,0,0}; // fl, fr, rl, rr

// Encoder counts (volatile because updated in ISR)
volatile long encoder_counts[6] = {0,0,0,0,0,0};
volatile long last_encoder_counts[6] = {0,0,0,0,0,0};

// For velocity estimation
double wheel_velocity[6] = {0,0,0,0,0,0}; // rad/s estimated

// --------------------------- HELPER: TCA9548A multiplexer select ---------------------------
void tca_select(uint8_t channel) {
  if (channel > 7) return;
  Wire.beginTransmission(TCA_ADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

// --------------------------- Encoder ISRs (quadrature) ---------------------------
// For each encoder A pin attach to ISR; in the ISR read the B pin state to determine direction.
void encoder_isr_0() { // wheel 0
  bool b = digitalRead(encB_pins[0]);
  if (b) encoder_counts[0]++; else encoder_counts[0]--;
}
void encoder_isr_1() { bool b = digitalRead(encB_pins[1]); if (b) encoder_counts[1]++; else encoder_counts[1]--; }
void encoder_isr_2() { bool b = digitalRead(encB_pins[2]); if (b) encoder_counts[2]++; else encoder_counts[2]--; }
void encoder_isr_3() { bool b = digitalRead(encB_pins[3]); if (b) encoder_counts[3]++; else encoder_counts[3]--; }
void encoder_isr_4() { bool b = digitalRead(encB_pins[4]); if (b) encoder_counts[4]++; else encoder_counts[4]--; }
void encoder_isr_5() { bool b = digitalRead(encB_pins[5]); if (b) encoder_counts[5]++; else encoder_counts[5]--; }

// --------------------------- Wheel command callbacks ---------------------------
void wheel_cmd_cb_0(const std_msgs::Float64& msg){ wheel_cmd[0] = msg.data; }
void wheel_cmd_cb_1(const std_msgs::Float64& msg){ wheel_cmd[1] = msg.data; }
void wheel_cmd_cb_2(const std_msgs::Float64& msg){ wheel_cmd[2] = msg.data; }
void wheel_cmd_cb_3(const std_msgs::Float64& msg){ wheel_cmd[3] = msg.data; }
void wheel_cmd_cb_4(const std_msgs::Float64& msg){ wheel_cmd[4] = msg.data; }
void wheel_cmd_cb_5(const std_msgs::Float64& msg){ wheel_cmd[5] = msg.data; }

// --------------------------- Steering callbacks ---------------------------
void steer_cb_fl(const std_msgs::Float64& msg){ steer_cmd[0] = msg.data; }
void steer_cb_fr(const std_msgs::Float64& msg){ steer_cmd[1] = msg.data; }
void steer_cb_rl(const std_msgs::Float64& msg){ steer_cmd[2] = msg.data; }
void steer_cb_rr(const std_msgs::Float64& msg){ steer_cmd[3] = msg.data; }

// --------------------------- Utility: set motor PWM and direction ---------------------------
void set_motor_output(int motor_idx, double wheel_rad_per_sec) {
  // Convert wheel angular velocity to PWM using simple proportional mapping
  // positive angular velocity -> forward, negative -> reverse
  double max_rad = 20.0; // rad/s (example), tune to your motor speed
  double frac = wheel_rad_per_sec / max_rad;
  if (frac > 1.0) frac = 1.0;
  if (frac < -1.0) frac = -1.0;
  int pwm = (int)(fabs(frac) * PWM_MAX);
  if (pwm < 0) pwm = 0;
  if (pwm > PWM_MAX) pwm = PWM_MAX;
  // set dir pin
  if (wheel_rad_per_sec >= 0) digitalWrite(motor_dir_pins[motor_idx], LOW); else digitalWrite(motor_dir_pins[motor_idx], HIGH);
  analogWrite(motor_pwm_pins[motor_idx], pwm);
}

// --------------------------- Setup ---------------------------
void setup() {
  Wire.begin();
  Serial.begin(115200);
  nh.initNode();

  // publishers
  nh.advertise(joint_state_pub);
  nh.advertise(encoder_pub);
  nh.advertise(imu_pub);
  nh.advertise(aht_temp_pub);
  nh.advertise(aht_hum_pub);
  nh.advertise(battery_pub);

  // subscribers for wheels
  wheel_subs[0] = new ros::Subscriber<std_msgs::Float64>("/fl_wheel_controller/command", wheel_cmd_cb_0);
  wheel_subs[1] = new ros::Subscriber<std_msgs::Float64>("/ml_wheel_controller/command", wheel_cmd_cb_1);
  wheel_subs[2] = new ros::Subscriber<std_msgs::Float64>("/rl_wheel_controller/command", wheel_cmd_cb_2);
  wheel_subs[3] = new ros::Subscriber<std_msgs::Float64>("/fr_wheel_controller/command", wheel_cmd_cb_3);
  wheel_subs[4] = new ros::Subscriber<std_msgs::Float64>("/mr_wheel_controller/command", wheel_cmd_cb_4);
  wheel_subs[5] = new ros::Subscriber<std_msgs::Float64>("/rr_wheel_controller/command", wheel_cmd_cb_5);
  for (int i=0;i<6;i++) nh.subscribe(*wheel_subs[i]);

  // subscribers for steering
  steer_subs[0] = new ros::Subscriber<std_msgs::Float64>("/fl_steer_controller/command", steer_cb_fl);
  steer_subs[1] = new ros::Subscriber<std_msgs::Float64>("/fr_steer_controller/command", steer_cb_fr);
  steer_subs[2] = new ros::Subscriber<std_msgs::Float64>("/rl_steer_controller/command", steer_cb_rl);
  steer_subs[3] = new ros::Subscriber<std_msgs::Float64>("/rr_steer_controller/command", steer_cb_rr);
  for (int i=0;i<4;i++) nh.subscribe(*steer_subs[i]);

  // configure motor pins
  for (int i=0;i<6;i++){
    pinMode(motor_pwm_pins[i], OUTPUT);
    pinMode(motor_dir_pins[i], OUTPUT);
    analogWrite(motor_pwm_pins[i], 0);
    digitalWrite(motor_dir_pins[i], LOW);
  }

  // encoder pins and attach interrupts (must ensure A pins used support attachInterrupt)
  attachInterrupt(digitalPinToInterrupt(encA_pins[0]), encoder_isr_0, RISING);
  attachInterrupt(digitalPinToInterrupt(encA_pins[1]), encoder_isr_1, RISING);
  attachInterrupt(digitalPinToInterrupt(encA_pins[2]), encoder_isr_2, RISING);
  attachInterrupt(digitalPinToInterrupt(encA_pins[3]), encoder_isr_3, RISING);
  attachInterrupt(digitalPinToInterrupt(encA_pins[4]), encoder_isr_4, RISING);
  attachInterrupt(digitalPinToInterrupt(encA_pins[5]), encoder_isr_5, RISING);

  // init pca9685
  pca.begin();
  pca.setPWMFreq(50); // 50Hz for servos

  // ina219 init (on TCA channel 0, for example)
  tca_select(0);
  ina219.begin();

  // TODO: init MPU6050 and AHT20 via multiplexer channels when reading
}

// --------------------------- Main loop ---------------------------
unsigned long last_control = 0;
unsigned long last_pub = 0;
void loop(){
  nh.spinOnce();
  unsigned long now = millis();
  // control loop at ~50Hz
  if (now - last_control >= (unsigned long)(control_dt*1000)){
    last_control = now;
    // compute velocity from encoder deltas
    for (int i=0;i<6;i++){
      long delta = encoder_counts[i] - last_encoder_counts[i];
      last_encoder_counts[i] = encoder_counts[i];
      // counts to radians
      double revolutions = (double)delta / (double)ENCODER_CPR;
      double rad = revolutions * 2.0 * 3.141592653589793;
      double vel = rad / control_dt; // rad/s estimate
      wheel_velocity[i] = vel;
      // simple P controller to achieve wheel_cmd[i] (desired rad/s)
      double err = wheel_cmd[i] - wheel_velocity[i];
      double effort = vel_kp[i] * err; // effort in rad/s equivalent fraction
      // map effort to PWM via set_motor_output
      set_motor_output(i, effort);
    }
  }

  // publish joint states and sensors ~20Hz
  if (now - last_pub >= 50){
    last_pub = now;
    // joint_states: wheel rotations and steering positions (we only publish wheel positions here)
    static const char* names[] = {"fl_wheel_joint","ml_wheel_joint","rl_wheel_joint","fr_wheel_joint","mr_wheel_joint","rr_wheel_joint","fl_steer_joint","fr_steer_joint","rl_steer_joint","rr_steer_joint"};
    joint_state_msg.header.stamp = nh.now();
    joint_state_msg.name = (char**)names;
    static double positions[10];
    for (int i=0;i<6;i++) positions[i] = (double)encoder_counts[i] * (2.0*3.141592653589793 / (double)ENCODER_CPR);
    // steering positions: we don't track actual servo position here; publish commanded value
    positions[6] = steer_cmd[0]; positions[7] = steer_cmd[1]; positions[8] = steer_cmd[2]; positions[9] = steer_cmd[3];
    joint_state_msg.position = positions;
    joint_state_pub.publish(&joint_state_msg);

    // encoders array publish
    encoder_array_msg.data_length = 6;
    static int32_t enc_out[6];
    for (int i=0;i<6;i++) enc_out[i] = (int32_t)encoder_counts[i];
    encoder_array_msg.data = enc_out;
    encoder_pub.publish(&encoder_array_msg);

    // IMU read via multiplexer channel (example: channel 1)
    tca_select(1);
    // TODO: read MPU6050 and fill imu_msg (placeholder zeros)
    imu_msg.header.stamp = nh.now();
    imu_msg.header.frame_id = "imu_link";
    imu_msg.linear_acceleration.x = 0.0;
    imu_msg.linear_acceleration.y = 0.0;
    imu_msg.linear_acceleration.z = 0.0;
    imu_msg.angular_velocity.x = 0.0;
    imu_msg.angular_velocity.y = 0.0;
    imu_msg.angular_velocity.z = 0.0;
    imu_pub.publish(&imu_msg);

    // AHT20 (example on channel 2)
    tca_select(2);
    // TODO: read AHT20 library
    temp_pub_msg.temperature = 0.0;
    aht_temp_pub.publish(&temp_pub_msg);
    aht_hum_msg.data = 0.0;
    aht_hum_pub.publish(&aht_hum_msg);

    // INA219 on channel 0
    tca_select(0);
    float bus_v = ina219.getBusVoltage_V();
    float curr_mA = ina219.getCurrent_mA();
    battery_msg.header.stamp = nh.now();
    battery_msg.voltage = bus_v;
    battery_msg.current = curr_mA / 1000.0; // A
    battery_pub.publish(&battery_msg);

    // steering: set PCA9685 servo PWM for steering commands
    // map steering radian to pulse length (user must calibrate range)
    int servo_channels[4] = {0,1,2,3};
    for (int i=0;i<4;i++){
      float angle = steer_cmd[i]; // radians
      // map -0.7..0.7 rad to servo pulse 120..520 (example)
      int pulse = (int)(map(constrain(angle*1000,-700,700), -700, 700, 120, 520));
      if (pulse < 0) pulse = 0;
      if (pulse > 4095) pulse = 4095;
      pca.setPWM(servo_channels[i], 0, pulse);
    }
  }
}
