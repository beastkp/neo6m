/*
 * Author: DJS Antariksh
 * Description: Calculate the angular velocity in radians/second of a DC motor
 * with a built-in encoder (forward = positive; reverse = negative) 
 */
#include <math.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt16.h>
#include <LiquidCrystal.h> 
#include <SoftwareSerial.h> 
#include <TinyGPS.h> 
#include <geometry_msgs/Vector3Stamped.h>
#include <ros/time.h>
#include <geometry_msgs/Twist.h>  
#include <sensor_msgs/NavSatFix.h>
 
// Motor encoder output pulses per 360 degree revolution (measured manually)
#define ENC_COUNT_REV 2250
// PIN DEFINITIONS
#define PWML 4
#define DIRL 21
#define PWMR 22
#define DIRR 26


// Encoder output to Arduino Interrupt pin. Tracks the pulse count.
#define ENC_IN_RIGHT1_A 10 //pico
#define ENC_IN_LEFT1_A 20 //pico

// Other encoder output to Arduino to keep track of wheel direction
// Tracks the direction of rotation.
#define ENC_IN_RIGHT1_B 11 //pico
#define ENC_IN_LEFT1_B 19  //pico

float meterr=0.0;
float meterl=0.0;
 
// True = Forward; False = Reverse
int Direction_right =LOW;
int Direction_right1 =LOW;

 
// Keep track of the number of right wheel pulses
volatile long right_wheel_pulse_count = 0;
volatile long left_wheel_pulse_count = 0;
 
// One-second interval for measurements
int interval = 1000;
  
// Counters for milliseconds during interval
long previousMillis = 0;
long currentMillis = 0;
 
// Variable for PWM motor speed output
int motorPWMR ; 
int motorPWML ; 
// Variable for RPM measuerment
float rpm_right = 0;
float rpm_left = 0;
 
// Variable for angular velocity measurement
float ang_velocity_right = 0;
float ang_velocity_right_deg = 0;
float ang_velocity_left = 0;
float ang_velocity_left_deg = 0;

float linear_velocity_left = 0;
float linear_velocity_right = 0;
float radius = 0.12;

float hellor;//for testing
float hellol;//for testing

 
const float rpm_to_radians = 0.10471975512;
const float rad_to_deg = 57.29578;
void subscriberCallback(const geometry_msgs::Twist& msg) {
    int vx = msg.linear.x;
    int vtheta = msg.angular.z;
  
    double l = 1;
    double vl = 300*(2.0 * vx - (vtheta * l)) / (2.0);
    double vr = 300*(2.0 * vx + (vtheta * l)) / (2.0);

    Direction_right = vr>0?HIGH:LOW;
    Direction_right1 = vl>0?HIGH:LOW;
    
    // set pwm ke variables
    motorPWMR = min(255, abs(vr));
    motorPWML = min(255, abs(vl));
}

// ROS
ros::NodeHandle node_handle;

geometry_msgs::Twist msg;       // from cmd_vel
geometry_msgs::Twist odom_msg;  // to odom_msg
std_msgs::Float64 float_msgr;       //for testing
std_msgs::Float64 float_msgl;       //for testing
geometry_msgs::Vector3Stamped speed_msg;    //create a "speed_msg" ROS message
sensor_msgs::NavSatFix gps_msg;  // recieving latitude and longitude


ros::Publisher speed_pub("speed", &speed_msg);
ros::Publisher publisher("wheel_odom", &msg);
ros::Publisher gps_pub("gps_fix", &gps_msg);
ros::Publisher chatterr("chatterr", &float_msgr);//for testing
ros::Publisher chatterl("chatterl", &float_msgl);//for testing
ros::Subscriber<geometry_msgs::Twist> subscriber("cmd_vel", &subscriberCallback);

float lat = 28.5458,lon = 77.1703; // create variable for latitude and longitude object  
TinyGPS gps; // create gps object 
SoftwareSerial gpsSerial(4,52);//rx,tx 
//LiquidCrystal lcd(A0,A1,A2,A3,A4,A5); 


void setup() {
  //gps part
  Serial.begin(9600); // connect serial   
  //Serial.println("The GPS Received Signal:"); 
  gpsSerial.begin(9600); // connect gps sensor 
//  lcd.begin(16,2); 
 
  // Open the serial port at 57600 bps
  Serial.begin(57600); 
  //long leftCount = 0;
  //long rightCount = 0;
  // Set pin states of the encoder
  pinMode(ENC_IN_RIGHT1_A, INPUT);
  pinMode(ENC_IN_RIGHT1_B , INPUT);
  pinMode(ENC_IN_LEFT1_A, INPUT);
  pinMode(ENC_IN_LEFT1_B , INPUT);
  pinMode(DIRR, OUTPUT);
  pinMode(DIRL, OUTPUT);
    
  pinMode(PWMR, OUTPUT);
  pinMode(PWML, OUTPUT);
  
//  analogWriteFreq(4888.88);

  
 
  // Every time the pin goes HIGH, this is a pulse
  
  attachInterrupt(digitalPinToInterrupt(ENC_IN_RIGHT1_A), right_pulse, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_IN_LEFT1_A), left_pulse, RISING);

      // ROS
    node_handle.initNode();

    node_handle.advertise(chatterr);//for testing
    node_handle.advertise(chatterl);//for testing

    node_handle.advertise(speed_pub);
    node_handle.advertise(publisher);
    node_handle.advertise(gps_pub);
    node_handle.subscribe(subscriber);
  
  
   
}
 
void loop() {
//gps part
while(gpsSerial.available()){ // check for gps data 
  if(gps.encode(gpsSerial.read()))// encode gps data 
  {  
  gps.f_get_position(&lat,&lon); // get latitude and longitude 
  // display position 
//  lcd.clear(); 
//  lcd.setCursor(1,0); 
//  lcd.print("GPS Signal"); 
  //Serial.print("Position: "); 
  //Serial.print("Latitude:"); 
  //Serial.print(lat,6); 
  //Serial.print(";"); 
  //Serial.print("Longitude:"); 
  //Serial.println(lon,6);  
//  lcd.setCursor(1,0); 
//  lcd.print("LAT:"); 
//  lcd.setCursor(5,0); 
//  lcd.print(lat); 
  //Serial.print(lat); 
  //Serial.print(" "); 
//  lcd.setCursor(0,1); 
//  lcd.print(",LON:"); 
//  lcd.setCursor(5,1); 
//  lcd.print(lon); 
 } 
} 
float latitude = lat;
float longitude = lon;
Serial.println(latitude);
Serial.println(longitude);
delay(1000); 

//  gps/fix.altitude = 0.0;

  
  
  node_handle.spinOnce();
      // Control motor with potentiometer
  //motorPWMR =100;
  //motorPWML =100;
  
    // Write PWM to controller
  analogWrite(PWML, motorPWMR);
  digitalWrite(DIRL,Direction_right);
  analogWrite(PWMR, motorPWML);
  digitalWrite(DIRR,Direction_right);

 
  // Record the time
  currentMillis = millis();
 
  // If one second has passed, print the number of pulses
  if (currentMillis - previousMillis > interval) {
 
    previousMillis = currentMillis;
 
    // Calculate revolutions per minute
    rpm_right = (float)(right_wheel_pulse_count * 60 / ENC_COUNT_REV);
    rpm_left = (float)(left_wheel_pulse_count * 60 / ENC_COUNT_REV);
    ang_velocity_right = rpm_right * rpm_to_radians; 
    ang_velocity_left = rpm_left * rpm_to_radians;   
    ang_velocity_right_deg = ang_velocity_right * rad_to_deg;
    ang_velocity_left_deg = ang_velocity_left * rad_to_deg;
    meterl=(left_wheel_pulse_count*2*3.14*radius)/2250;
    meterr=(right_wheel_pulse_count*2*3.14*radius)/2250;
    float linear_velocity_right = ang_velocity_right * radius;
    float linear_velocity_left = ang_velocity_left * radius;
    Serial.println("LENGTH RIGHT");
    Serial.println(meterr);
    Serial.println("LENGTH LEFT");
    Serial.println(meterl);

    
    odom_msg.linear.x = linear_velocity_right;
    odom_msg.linear.y = Direction_right;
    odom_msg.linear.z = motorPWMR;
    odom_msg.angular.x = linear_velocity_left;
    odom_msg.angular.y = Direction_right1;
    odom_msg.angular.z = motorPWML;
    
    gps_msg.latitude= latitude;
    gps_msg.longitude= longitude;

    hellor=meterr;//for testing
    hellol=meterl;//for testing
    
    float_msgr.data=hellor;//for testing
    float_msgl.data=hellol;//for testing
    
    chatterr.publish( &float_msgr );//for testing
    chatterl.publish( &float_msgl);//for testing
    
    gps_pub.publish(&gps_msg);
    
    publisher.publish(&odom_msg);

    publishSpeed(interval);
     
 }
  
  
}

void publishSpeed(double time) {
  speed_msg.header.stamp = node_handle.now();      //timestamp for odometry data
  speed_msg.vector.x = meterl;    //left wheel speed (in m/s)
  speed_msg.vector.y = meterr;   //right wheel speed (in m/s)
  speed_msg.vector.z = time/1000;         //looptime, should be the same as specified in LOOPTIME (in s)
  speed_pub.publish(&speed_msg);
  node_handle.spinOnce();
//  node_handle.loginfo("Publishing odometry");
}
 
// Increment the number of pulses by 1
//void right_pulse() {
//   
//  // Read the value for the encoder for the right wheel
//  int val = digitalRead(ENC_IN_RIGHT1_B);
// 
//  if(val == HIGH) {
//    Direction_right = LOW; // Reverse
//  }
//  else {
//    Direction_right = HIGH; // Forward
//  }
//   
//  if (val == HIGH) {
//    right_wheel_pulse_count++;
//  }
//  else {
//    right_wheel_pulse_count--;
//  }
//}
//void left_pulse() {
//   
//  // Read the value for the encoder for the right wheel
//  int val = digitalRead(ENC_IN_LEFT1_B);
// 
//  if(val == HIGH) {
//    Direction_right1 = LOW; // Reverse
//  }
//  else {
//    Direction_right1 = HIGH; // Forward
//  }
//   
//  if (val == HIGH) {
//    left_wheel_pulse_count++;
//  }
//  else {
//    left_wheel_pulse_count--;
//  }
//}
void right_pulse(){
if (digitalRead(ENC_IN_RIGHT1_A) == HIGH) {

if (digitalRead(ENC_IN_RIGHT1_B) == LOW) {
right_wheel_pulse_count = right_wheel_pulse_count + 1; // CW
}
else {
right_wheel_pulse_count = right_wheel_pulse_count - 1; // CCW
}
}

else // must be a high-to-low edge on channel A
{
// check channel B to see which way encoder is turning
if (digitalRead(ENC_IN_RIGHT1_B) == HIGH) {
right_wheel_pulse_count = right_wheel_pulse_count + 1; // CW
}
else {
right_wheel_pulse_count = right_wheel_pulse_count - 1; // CCW
}
}
}

void left_pulse(){
if (digitalRead(ENC_IN_LEFT1_A) == HIGH) {

if (digitalRead(ENC_IN_LEFT1_B) == LOW) {
left_wheel_pulse_count = left_wheel_pulse_count + 1; // CW
}
else {
left_wheel_pulse_count = left_wheel_pulse_count - 1; // CCW
}
}

else // must be a high-to-low edge on channel A
{
// check channel B to see which way encoder is turning
if (digitalRead(ENC_IN_LEFT1_B) == HIGH) {
left_wheel_pulse_count = left_wheel_pulse_count + 1; // CW
}
else {
left_wheel_pulse_count = left_wheel_pulse_count - 1; // CCW
}
}

}
