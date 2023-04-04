#define USE_USBCON
#include <NewPing.h>
#include <SimpleKalmanFilter.h>
#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Range.h>
 
#define SONAR_NUM 7          //The number of sensors. 
#define MAX_DISTANCE 200     //Mad distance to detect obstacles.
#define PING_INTERVAL 33     //Looping the pings after 33 microseconds.
 
unsigned long pingTimer[SONAR_NUM]; // Holds the times when the next ping should happen for each sensor.
unsigned int cm[SONAR_NUM];         // Where the ping distances are stored.
uint8_t currentSensor = 0;          // Keeps track of which sensor is active.
 
unsigned long _timerStart = 0;
 
int LOOPING = 40; //Loop for every 40 milliseconds.
 
uint8_t oldSensorReading[7];    //Store last valid value of the sensors.
 
uint8_t left_top_Sensor;             //Store raw sensor's value.
uint8_t centerSensor;
uint8_t right_top_Sensor;
uint8_t left_back_Sensor;
uint8_t right_back_Sensor;
uint8_t left_back_back_Sensor;
uint8_t right_back_back_Sensor;

 
uint8_t left_top_SensorKalman;       //Store filtered sensor's value.
uint8_t centerSensorKalman;
uint8_t right_top_SensorKalman;
uint8_t left_back_SensorKalman;
uint8_t right_back_SensorKalman;
uint8_t left_back_back_SensorKalman;
uint8_t right_back_back_SensorKalman;
 
 
NewPing sonar[SONAR_NUM] = {
  NewPing(7, 9, MAX_DISTANCE), //left_top
  NewPing(6, 11, MAX_DISTANCE), //center
  NewPing(8, 10, MAX_DISTANCE),//right_top
  NewPing(5, 12, MAX_DISTANCE),//left_back
  NewPing(14, 15, MAX_DISTANCE),//right_back
  NewPing(17, 16, MAX_DISTANCE), //left_back_back
  NewPing(25, 24, MAX_DISTANCE) //right_back_back
  
};
 
/*
  create Kalman filter objects for the sensors.
   SimpleKalmanFilter(e_mea, e_est, q);
   e_mea: Measurement Uncertainty
   e_est: Estimation Uncertainty
   q: Process Noise
*/
SimpleKalmanFilter KF_Left_top(2, 2, 0.01);
SimpleKalmanFilter KF_Center(2, 2, 0.01);
SimpleKalmanFilter KF_Right_top(2, 2, 0.01);
SimpleKalmanFilter KF_Left_back(2, 2, 0.01);
SimpleKalmanFilter KF_Right_back(2, 2, 0.01);
SimpleKalmanFilter KF_Left_back_back(2, 2, 0.01);
SimpleKalmanFilter KF_Right_back_back(2, 2, 0.01);
 
ros::NodeHandle nh; //create an object which represents the ROS node.
 
//looping the sensors
void sensorCycle() {
  for (uint8_t i = 0; i < SONAR_NUM; i++) {
    if (millis() >= pingTimer[i]) {
      pingTimer[i] += PING_INTERVAL * SONAR_NUM;
      if (i == 0 && currentSensor == SONAR_NUM - 1) oneSensorCycle();
      sonar[currentSensor].timer_stop();
      currentSensor = i;
      cm[currentSensor] = 0;
      sonar[currentSensor].ping_timer(echoCheck);
    }
  }
}
 
// If ping received, set the sensor distance to array.
void echoCheck() {
  if (sonar[currentSensor].check_timer())
    cm[currentSensor] = sonar[currentSensor].ping_result / US_ROUNDTRIP_CM;
}
 
//Return the last valid value from the sensor.
void oneSensorCycle() {
  left_top_Sensor   = returnLastValidRead(0, cm[0]);
  centerSensor = returnLastValidRead(1, cm[1]);
  right_top_Sensor  = returnLastValidRead(2, cm[2]);
  left_back_Sensor   = returnLastValidRead(3, cm[3]);
  right_back_Sensor   = returnLastValidRead(4, cm[4]);
  left_back_back_Sensor   = returnLastValidRead(5, cm[5]);
  right_back_back_Sensor   = returnLastValidRead(6, cm[6]);

}
 
//If sensor value is 0, then return the last stored value different than 0.
int returnLastValidRead(uint8_t sensorArray, uint8_t cm) {
  if (cm != 0) {
    return oldSensorReading[sensorArray] = cm;
  } else {
    return oldSensorReading[sensorArray];
  }
}
 
//Apply Kalman Filter to sensor reading.
void applyKF() {
  left_top_SensorKalman   = KF_Left_top.updateEstimate(left_top_Sensor);
  centerSensorKalman = KF_Center.updateEstimate(centerSensor);
  right_top_SensorKalman  = KF_Right_top.updateEstimate(right_top_Sensor);
  left_back_SensorKalman   = KF_Left_back.updateEstimate(left_back_Sensor);
  right_back_SensorKalman   = KF_Right_back.updateEstimate(right_back_Sensor);
  left_back_back_SensorKalman   = KF_Left_back_back.updateEstimate(left_back_back_Sensor);
  right_back_back_SensorKalman   = KF_Right_back_back.updateEstimate(right_back_back_Sensor);
}
 
void startTimer() {
  _timerStart = millis();
}
 
bool isTimeForLoop(int _mSec) {
  return (millis() - _timerStart) > _mSec;
}
 
void sensor_msg_init(sensor_msgs::Range &range_name, char *frame_id_name)
{
  range_name.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_name.header.frame_id = frame_id_name;
  range_name.field_of_view = 0.26;
  range_name.min_range = 0.0;
  range_name.max_range = 2.0;
}
 
//Create three instances for range messages.
sensor_msgs::Range range_left_top;
sensor_msgs::Range range_center;
sensor_msgs::Range range_right_top;
sensor_msgs::Range range_left_back;
sensor_msgs::Range range_right_back;
sensor_msgs::Range range_left_back_back;
sensor_msgs::Range range_right_back_back;
 
//Create publisher onjects for all sensors
ros::Publisher pub_range_left_top("/ultrasound_left_top", &range_left_top);
ros::Publisher pub_range_center("/ultrasound_center", &range_center);
ros::Publisher pub_range_right_top("/ultrasound_right_top", &range_right_top);
ros::Publisher pub_range_left_back("/ultrasound_left_back", &range_left_back);
ros::Publisher pub_range_right_back("/ultrasound_right_back", &range_right_back);
ros::Publisher pub_range_left_back_back("/ultrasound_left_back_back", &range_left_back_back);
ros::Publisher pub_range_right_back_back("/ultrasound_right_back_back", &range_right_back_back);
 
void setup() {
  pingTimer[0] = millis() + 75;
  for (uint8_t i = 1; i < SONAR_NUM; i++)
    pingTimer[i] = pingTimer[i - 1] + PING_INTERVAL;
 
  nh.initNode();
  nh.advertise(pub_range_left_top);
  nh.advertise(pub_range_center);
  nh.advertise(pub_range_right_top);
  nh.advertise(pub_range_left_back);
  nh.advertise(pub_range_right_back);
  nh.advertise(pub_range_left_back_back);
  nh.advertise(pub_range_right_back_back);
 
  sensor_msg_init(range_left_top, "/ultrasound_left_top");
  sensor_msg_init(range_center, "/ultrasound_center");
  sensor_msg_init(range_right_top, "/ultrasound_right_top");
  sensor_msg_init(range_left_back, "/ultrasound_left_back");
  sensor_msg_init(range_right_back, "/ultrasound_right_back");
  sensor_msg_init(range_left_back_back, "/ultrasound_left_back_back");
  sensor_msg_init(range_right_back_back, "/ultrasound_right_back_back");
}
 
void loop() {
  if (isTimeForLoop(LOOPING)) {
    sensorCycle();
    oneSensorCycle();
    applyKF();
    range_left_top.range = left_top_SensorKalman;
    range_center.range = centerSensorKalman;
    range_right_top.range = right_top_SensorKalman;
    range_left_back.range = left_back_SensorKalman;
    range_right_back.range = right_back_SensorKalman;
    range_left_back_back.range = left_back_back_SensorKalman;
    range_right_back_back.range = right_back_back_SensorKalman;
 
    range_left_top.header.stamp = nh.now();
    range_center.header.stamp = nh.now();
    range_right_top.header.stamp = nh.now();
    range_left_back.header.stamp = nh.now();
    range_right_back.header.stamp = nh.now();
    range_left_back_back.header.stamp = nh.now();
    range_right_back_back.header.stamp = nh.now();
 
    pub_range_left_top.publish(&range_left_top);
    pub_range_center.publish(&range_center);
    pub_range_right_top.publish(&range_right_top);
    pub_range_left_back.publish(&range_left_back);
    pub_range_right_back.publish(&range_right_back);
    pub_range_left_back_back.publish(&range_left_back_back);
    pub_range_right_back_back.publish(&range_right_back_back);
 
    startTimer();
  }
  nh.spinOnce();//Handle ROS events
}
