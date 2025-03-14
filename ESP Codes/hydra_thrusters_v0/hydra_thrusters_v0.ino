#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int16MultiArray.h>
#include <ESP32Servo.h>

const int thruster1Pin = 16;  
const int thruster2Pin = 17;
const int thruster3Pin = 18;
const int thruster4Pin = 19;

Servo thruster1Servo;
Servo thruster2Servo;
Servo thruster3Servo;
Servo thruster4Servo;

int16_t feedbackValues[4] = {1500, 1500, 1500, 1500};

std_msgs::Int16MultiArray feedback_msg;
ros::NodeHandle nh;
ros::Publisher feedback_pub("thruster_feedback", &feedback_msg);

int targetPulse[4]  = {1500, 1500, 1500, 1500}; 
float currentPulse[4] = {1500, 1500, 1500, 1500};  

const float alpha = 0.05;   // Lower value for smoother changes
const int errorThreshold = 3;  
const int baseStep = 3;  // Minimum step size
const int maxStep = 15;  // Maximum step size for large changes

int mapToPulseWidth(int input) {
  int pulseWidth = map(input, -400, 400, 1900, 1100);
  return constrain(pulseWidth, 1100, 1900);
}

void publishFeedback() {
  for (int i = 0; i < 4; i++) {
    feedbackValues[i] = round(currentPulse[i]);
  }
  feedback_pub.publish(&feedback_msg);
}

void thruster1Callback(const std_msgs::Int16 &msg) {
  targetPulse[0] = mapToPulseWidth(msg.data);
}

void thruster2Callback(const std_msgs::Int16 &msg) {
  targetPulse[1] = mapToPulseWidth(msg.data);
}

void thruster3Callback(const std_msgs::Int16 &msg) {
  targetPulse[2] = mapToPulseWidth(msg.data);
}

void thruster4Callback(const std_msgs::Int16 &msg) {
  targetPulse[3] = mapToPulseWidth(msg.data);
}

ros::Subscriber<std_msgs::Int16> sub_thruster1("right_thruster", thruster1Callback);
ros::Subscriber<std_msgs::Int16> sub_thruster2("left_thruster", thruster2Callback);
ros::Subscriber<std_msgs::Int16> sub_thruster3("front_thruster", thruster3Callback);
ros::Subscriber<std_msgs::Int16> sub_thruster4("back_thruster", thruster4Callback);

void setup() {
  nh.initNode();

  thruster1Servo.attach(thruster1Pin);
  thruster2Servo.attach(thruster2Pin);
  thruster3Servo.attach(thruster3Pin);
  thruster4Servo.attach(thruster4Pin);

  feedback_msg.data = feedbackValues;
  feedback_msg.data_length = 4;

  nh.subscribe(sub_thruster1);
  nh.subscribe(sub_thruster2);
  nh.subscribe(sub_thruster3);
  nh.subscribe(sub_thruster4);
  nh.advertise(feedback_pub);
}

void loop() {
  nh.spinOnce();

  for (int i = 0; i < 4; i++) {
    float smoothedValue = alpha * targetPulse[i] + (1 - alpha) * currentPulse[i];

    if (abs(smoothedValue - 1500) < errorThreshold) {
      smoothedValue = 1500;
    }

    // Dynamic step adjustment for ultra-smooth transition
    float delta = smoothedValue - currentPulse[i];
    int adaptiveStep = baseStep + (maxStep - baseStep) * (abs(delta) / 400.0);  // Scale step dynamically

    if (abs(delta) > adaptiveStep) {
      smoothedValue = currentPulse[i] + (delta > 0 ? adaptiveStep : -adaptiveStep);
    }

    currentPulse[i] = smoothedValue;
  }

  thruster1Servo.writeMicroseconds(round(currentPulse[0]));
  thruster2Servo.writeMicroseconds(round(currentPulse[1]));
  thruster3Servo.writeMicroseconds(round(currentPulse[2]));
  thruster4Servo.writeMicroseconds(round(currentPulse[3]));

  publishFeedback();

  delay(10); 
}
