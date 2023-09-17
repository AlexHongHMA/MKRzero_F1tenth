/****************************************************************
 * Two Sensors Combined Code::Final Edited by Alexander Hong

 * Original Arthur::Paul Clark Based on original code by: Owen Lyke @ SparkFun Electronics 
 * Original Creation Date: April 17 2019
 
 * Code was Improved by Professor Benjamas Panomruttanarug "BP" 
 * Department of Control System and Instrumention Engineering "INC" @ KMUTT
 * Cosin Lab (COntrol System and INstruction Lab) 

 ***** Professor's Note *****
 * BP trying to combine imu (icm20948) with microros on ROS2
 * Testing with MKR zero arduino board
 
 * Edited by Alexander Hong "AlexHong" (Htet Myat Aung) 
 * Edited Date: Sept 16/09/2023
 * Bachelor Student @ KMUTT 
 * Department of Control System and Instrumention Engineering "INC" @ KMUTT
 * Cosin Lab (COntrol System and INstruction Lab) 

 ***** Important Noteby::Alexander Hong ***** 
 * 
 * Removed QUAT_ANIMATION Library and usages 
 * Added ROS codes to Echo Linear Velocity values from Hall Sensor mounted on Driven Gear of F1tenth Car
 * Create Functions for more clean code
 * Myth : Removed all Serial.println - seem causing to interrupt the output


 * Here, our RPM sensor is actually recording data from large Gear A (not the pinion from brushless motor). 
 * Therefore, we don't need to calculate angular velocity of gear B (pinion gear). 

 * if the RPM sensor is recording data from the motor's pinion or from the motor's shelf directly, 
   we need to apply this equation 
          
          w(A) = w(B)/gearRatio (check the equation back) where :: w(B) = 2*PI*rpm/60

 * In case, if we need to calculate GearRatio related equations, 
        gearRatio = Teeth of Driven/ Teeth of Driver
        
 * Gear Information for this robot - pinion -> 23T , Driven Gear -> 86T,48P  
 * 
 ***************************************************************/
 
#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

//BP added
//micro ros utilities libraries imported::Noteby::AlexHong
#include <micro_ros_utilities/type_utilities.h>
#include <micro_ros_utilities/string_utilities.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/string.h>
#include <std_msgs/msg/int32.h>

rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

//BP added 
// * Echoing Message::Noteby::AlexHong
rcl_publisher_t string_publisher;
rcl_publisher_t pedometer_publisher;

rcl_publisher_t xvel_publisher;                 //Editedby::AlexHong
rcl_publisher_t yaw_publisher;
//rcl_publisher_t err_publisher;                //Editedby::AlexHong (Remove Comment to test with Error Message)

std_msgs__msg__String string_msg;
std_msgs__msg__Int32 pedometer_msg;

std_msgs__msg__Float32 xvel_msg;              //Editedby::AlexHong
std_msgs__msg__Float32 pyaw_msg;
std_msgs__msg__Float32 cyaw_msg;
//std_msgs__msg__Float32 err_msg;               //Editedby::AlexHong (Remove Comment to test with Error Message)

//Topic Name Declaration::Editedby::AlexHong
const char * yaw_tpName = "yaw";
const char * xvel_tpName = "xvel";      
//const char * err_tpName = "err";

//Message Type Support::Editedby::AlexHong
const rosidl_message_type_support_t * type_support =
  ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32);

#define LED_PIN 13 //Error Check::Noteby::AlexHong

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

#include "ICM_20948.h" // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU

#define USE_SPI       // Used SPI::Noteby::AlexHong

#define SERIAL_PORT Serial

#define SPI_PORT SPI // Your desired SPI port.       Used only when "USE_SPI" is defined
#define CS_PIN 2     // Which pin you connect CS to. Used only when "USE_SPI" is defined

#define WIRE_PORT Wire // Your desired Wire port.      Used when "USE_SPI" is not defined
// The value of the last bit of the I2C address.
// On the SparkFun 9DoF IMU breakout the default is 1, and when the ADR jumper is closed the value becomes 0
#define AD0_VAL 1

#ifdef USE_SPI
ICM_20948_SPI myICM; // If using SPI create an ICM_20948_SPI object
#else
ICM_20948_I2C myICM; // Otherwise create an ICM_20948_I2C object
#endif


//Declaring Global Variables for RPM Sensor::Noteby::AlexHong

const int sensorPin = 4; // Connect the RPM sensor to digital pin 2 //AttachInterrupt Pins -> 0,1,4,5,6,7,8,9,A1,A2 for MKR Family Board

volatile unsigned long pulseCount = 0;
unsigned long lastTime = 0;
const int sampleInterval = 1000; // Time interval for calculating RPM in milliseconds

// Vehicle's Wheel Parameter
const float wheelRadius = 0.0525; // Wheel Diameter in meter value

// Parameters for x_velocity, yaw value, previous and current yaw;
double xvel_value = 0.0;
double yaw_value = 0.0;
double previous_yaw = 0.0;
double current_yaw = 0.0;

// End::Noteby::AlexHong

void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

double xvel_data( double linear_vel ){
  
  unsigned long currentTime = millis();
  unsigned long elapsedTime = currentTime - lastTime;
  
// Ensure a minimum time has passed to avoid division by zero
  if (elapsedTime >= sampleInterval) {
   if (elapsedTime != 0){
// Calculate RPM
    int rpm = (pulseCount * 60000) / (elapsedTime); // Multiplying by 60,000 is used to convert the time from milliseconds to minutes (since there are 60,000 milliseconds in a minute)

// Calculate Angular Velocity of Driven Gear
    double angularVelocityGearA = (2 * PI * rpm) / 60 ; // angular velocity of Pinion from Brushless Motor in rad/s // Divided by 60 to change it to second

// Calculate Linear Velocity
    double linear_vel = angularVelocityGearA * wheelRadius; // linear velocity of gear A in Meter/sec
    
    // Reset pulse count and update last time

    pulseCount = 0;
    lastTime = currentTime;

    return linear_vel;
    
    } else return linear_vel;
  } else return linear_vel;
}

double yaw_data( double yaw ){

  // Read any DMP data waiting in the FIFO
  // Note:
  //    readDMPdataFromFIFO will return ICM_20948_Stat_FIFONoDataAvail if no data is available.
  //    If data is available, readDMPdataFromFIFO will attempt to read _one_ frame of DMP data.
  //    readDMPdataFromFIFO will return ICM_20948_Stat_FIFOIncompleteData if a frame was present but was incomplete
  //    readDMPdataFromFIFO will return ICM_20948_Stat_Ok if a valid frame was read.
  //    readDMPdataFromFIFO will return ICM_20948_Stat_FIFOMoreDataAvail if a valid frame was read _and_ the FIFO contains more (unread) data.
  
  icm_20948_DMP_data_t data;
  myICM.readDMPdataFromFIFO(&data);

  if ((myICM.status == ICM_20948_Stat_Ok) || (myICM.status == ICM_20948_Stat_FIFOMoreDataAvail)) // Was valid data available?
  {

    if ((data.header & DMP_header_bitmap_Quat6) > 0) // We have asked for GRV data so we should receive Quat6
    {
      // Q0 value is computed from this equation: Q0^2 + Q1^2 + Q2^2 + Q3^2 = 1.
      // In case of drift, the sum will not add to 1, therefore, quaternion data need to be corrected with right bias values.
      // The quaternion data is scaled by 2^30.

      //SERIAL_PORT.printf("Quat6 data is: Q1:%ld Q2:%ld Q3:%ld\r\n", data.Quat6.Data.Q1, data.Quat6.Data.Q2, data.Quat6.Data.Q3);

      // Scale to +/- 1
      double q1 = ((double)data.Quat6.Data.Q1) / 1073741824.0; // Convert to double. Divide by 2^30
      double q2 = ((double)data.Quat6.Data.Q2) / 1073741824.0; // Convert to double. Divide by 2^30
      double q3 = ((double)data.Quat6.Data.Q3) / 1073741824.0; // Convert to double. Divide by 2^30

      // Convert the quaternions to Euler angles (roll, pitch, yaw)
      // https://en.wikipedia.org/w/index.php?title=Conversion_between_quaternions_and_Euler_angles&section=8#Source_code_2

      double q0 = sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));

      double q2sqr = q2 * q2;

//      // roll (x-axis rotation)                         //::Uncomment this code to use Roll Data::Noteby::AlexHong
//      double t0 = +2.0 * (q0 * q1 + q2 * q3);
//      double t1 = +1.0 - 2.0 * (q1 * q1 + q2sqr);
//      double roll = atan2(t0, t1) * 180.0 / PI;
//
//      // pitch (y-axis rotation)                        //::Uncomment this code to use Pitch Data::Noteby::AlexHong
//      double t2 = +2.0 * (q0 * q2 - q3 * q1);
//      t2 = t2 > 1.0 ? 1.0 : t2;
//      t2 = t2 < -1.0 ? -1.0 : t2;
//      double pitch = asin(t2) * 180.0 / PI;

      // yaw (z-axis rotation)
      double t3 = +2.0 * (q0 * q3 + q1 * q2);
      double t4 = +1.0 - 2.0 * (q2sqr + q3 * q3);
      double updated_yaw = atan2(t3, t4) * 180.0 / PI;
      
      return updated_yaw;
    }
  }
  
  if (myICM.status != ICM_20948_Stat_FIFOMoreDataAvail) // If more data is available then we should read it right away - and not delay
  {
    delay(10);
  }
  
}


void countPulses(){
    if(digitalRead(sensorPin)==HIGH) pulseCount++;
    else pulseCount;
}

void setup()
{
  if (myICM.status != ICM_20948_Stat_FIFOMoreDataAvail) // If more data is available then we should read it right away - and not delay
  {
    delay(10);
  }
  SERIAL_PORT.begin(115200); // Start the serial console
  
  #ifdef USE_SPI
    SPI_PORT.begin();
  #else
    WIRE_PORT.begin();
    WIRE_PORT.setClock(400000);
  #endif


  
  bool initialized = false;
  while (!initialized)
  {
    // Initialize the ICM-20948
    // If the DMP is enabled, .begin performs a minimal startup. We need to configure the sample mode etc. manually.
#ifdef USE_SPI
    myICM.begin(CS_PIN, SPI_PORT);
#else
    myICM.begin(WIRE_PORT, AD0_VAL);
#endif


    if (myICM.status != ICM_20948_Stat_Ok)
    {
      delay(500);
    }
    else
    {
      initialized = true;
    }
  }

  bool success = true; // Use success to show if the DMP configuration was successful

  // Initialize the DMP. initializeDMP is a weak function. You can overwrite it if you want to e.g. to change the sample rate
  success &= (myICM.initializeDMP() == ICM_20948_Stat_Ok);

  // DMP sensor options are defined in ICM_20948_DMP.h
  //    INV_ICM20948_SENSOR_ACCELEROMETER               (16-bit accel)
  //    INV_ICM20948_SENSOR_GYROSCOPE                   (16-bit gyro + 32-bit calibrated gyro)
  //    INV_ICM20948_SENSOR_RAW_ACCELEROMETER           (16-bit accel)
  //    INV_ICM20948_SENSOR_RAW_GYROSCOPE               (16-bit gyro + 32-bit calibrated gyro)
  //    INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED (16-bit compass)
  //    INV_ICM20948_SENSOR_GYROSCOPE_UNCALIBRATED      (16-bit gyro)
  //    INV_ICM20948_SENSOR_STEP_DETECTOR               (Pedometer Step Detector)
  //    INV_ICM20948_SENSOR_STEP_COUNTER                (Pedometer Step Detector)
  //    INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR        (32-bit 6-axis quaternion)                      // <= We are using this option
  //    INV_ICM20948_SENSOR_ROTATION_VECTOR             (32-bit 9-axis quaternion + heading accuracy)
  //    INV_ICM20948_SENSOR_GEOMAGNETIC_ROTATION_VECTOR (32-bit Geomag RV + heading accuracy)
  //    INV_ICM20948_SENSOR_GEOMAGNETIC_FIELD           (32-bit calibrated compass)
  //    INV_ICM20948_SENSOR_GRAVITY                     (32-bit 6-axis quaternion)
  //    INV_ICM20948_SENSOR_LINEAR_ACCELERATION         (16-bit accel + 32-bit 6-axis quaternion)
  //    INV_ICM20948_SENSOR_ORIENTATION                 (32-bit 9-axis quaternion + heading accuracy)

  // Enable the DMP Game Rotation Vector sensor
  success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR) == ICM_20948_Stat_Ok);

  // Enable any additional sensors / features
  //success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_RAW_GYROSCOPE) == ICM_20948_Stat_Ok);
  //success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_RAW_ACCELEROMETER) == ICM_20948_Stat_Ok);
  //success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED) == ICM_20948_Stat_Ok);

  // Configuring DMP to output data at multiple ODRs:
  // DMP is capable of outputting multiple sensor data at different rates to FIFO.
  // Setting value can be calculated as follows:
  // Value = (DMP running rate / ODR ) - 1
  // E.g. For a 5Hz ODR rate when DMP is running at 55Hz, value = (55/5) - 1 = 10.
  success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Quat6, 0) == ICM_20948_Stat_Ok); // Set to the maximum
  //success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Accel, 0) == ICM_20948_Stat_Ok); // Set to the maximum
  //success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Gyro, 0) == ICM_20948_Stat_Ok); // Set to the maximum
  //success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Gyro_Calibr, 0) == ICM_20948_Stat_Ok); // Set to the maximum
  //success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Cpass, 0) == ICM_20948_Stat_Ok); // Set to the maximum
  //success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Cpass_Calibr, 0) == ICM_20948_Stat_Ok); // Set to the maximum

  // Enable the FIFO
  success &= (myICM.enableFIFO() == ICM_20948_Stat_Ok);

  // Enable the DMP
  success &= (myICM.enableDMP() == ICM_20948_Stat_Ok);

  // Reset DMP
  success &= (myICM.resetDMP() == ICM_20948_Stat_Ok);

  // Reset FIFO
  success &= (myICM.resetFIFO() == ICM_20948_Stat_Ok);

  
// Editedby::AlexHong
// RPM Sensor Connection
  pinMode(sensorPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(sensorPin), countPulses, CHANGE);
// End

  //BP add microROS
  set_microros_transports();
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  
  
  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

  // create publisher BP
  // * Editedby::AlexHong
  // * Edited Note -> Publisher for Linear Velocity

/*
 ***** Test_Error_Message *****
 * Important Note 
 * Error Msg Value -> 0 ( Not Work )
 * Error Msg Value -> 1 ( Work )
 */
//    RCCHECK(rclc_publisher_init_default(
//    &err_publisher,
//    &node,
//    type_support,
//    err_tpName)); 

  RCCHECK(rclc_publisher_init_default(
    &xvel_publisher,
    &node,
    type_support,
    xvel_tpName)); 
  
  RCCHECK(rclc_publisher_init_default(
    &yaw_publisher,
    &node,
    type_support,
    yaw_tpName)); 
}


double DeltaYaw(double pyaw_value, double cyaw_value){

  double delta = cyaw_value - pyaw_value;
  if (delta > 180.0) delta -= 360.0;
  else if (delta < -180.0) delta += 360;
  return delta;
}

void loop()
{ 
    current_yaw = yaw_data(yaw_value);
//  pyaw_msg.data = current_yaw;
//  cyaw_msg.data = previous_yaw;
//  RCSOFTCHECK(rcl_publish(&yaw_publisher, &pyaw_msg, NULL));
//  RCSOFTCHECK(rcl_publish(&yaw_publisher, &cyaw_msg, NULL));

    double deltaYaw = DeltaYaw(previous_yaw,current_yaw);
    
    cyaw_msg.data = deltaYaw;
    RCSOFTCHECK(rcl_publish(&yaw_publisher, &cyaw_msg, NULL));

    previous_yaw = current_yaw;
  
    xvel_value = xvel_data(xvel_value); 

    xvel_msg.data = xvel_value;
    RCSOFTCHECK(rcl_publish(&xvel_publisher, &xvel_msg, NULL));
}
