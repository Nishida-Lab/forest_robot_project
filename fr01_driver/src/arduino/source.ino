#include <ros.h> // Use ros_lib.
#include <std_msgs/Int32.h>
#include <std_msgs/Int32MultiArray.h>
#include <sensor_msgs/JointState.h>

#define num_of_motor 6
#define MIN_PWM -100
#define MAX_PWM 100

int dir_pin_array[num_of_motor] = {27, 26, 23, 22, 25, 24};
int pwm_pin_array[num_of_motor] = { 4,  5,  8,  9,  6,  7};

bool rotate_positive[num_of_motor] = {HIGH, LOW, HIGH, LOW, HIGH, LOW};
bool rotate_negative[num_of_motor] = {LOW, HIGH, LOW, HIGH, LOW, HIGH};


/* Declare proto type functions. */
void wheelCb(const std_msgs::Int32MultiArray& msg); 

/* Declare global variables. */
ros::NodeHandle nh; // The nodeHandle.
ros::Subscriber<std_msgs::Int32MultiArray> sub("/motor_input", &wheelCb); // Set subscribe the motor_driver topic.

void setup() {
  /* Set pins Mode. */
  for (int i = 0; i < 6; ++i) {
      pinMode(dir_pin_array[i], OUTPUT);
      pinMode(pwm_pin_array[i], OUTPUT);
  }
  /* Node handle setting. */
  nh.initNode(); // First setup the node handle.
  nh.subscribe(sub); // Start subscribe the "steer_ctrl" topic.
}

void loop() {
  nh.spinOnce(); // Check topic and if change it, run the call back function.
}


void wheelCb(const std_msgs::Int32MultiArray& msg) {
  for(int i = 0; i < num_of_motor; ++i)
  {
    if(msg.data[i] >= 0)
    {
      digitalWrite(dir_pin_array[i], rotate_positive[i]);
      if(msg.data[i] > MAX_PWM)
      {
        msg.data[i] = MAX_PWM;
      }
      analogWrite(pwm_pin_array[i], fabs(msg.data[i]));
    }
    else
    {
      digitalWrite(dir_pin_array[i], rotate_negative[i]);
      if(msg.data[i] < MIN_PWM)
      {
        msg.data[i] = MIN_PWM;
      }
      analogWrite(pwm_pin_array[i], fabs(msg.data[i]));
      //analogWrite(pwm_pin_array[i], 80);
    }
  }
}
