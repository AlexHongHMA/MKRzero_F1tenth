/****************************************************************
 * Two Sensors Combined Code::Final Edited by Alexander Hong

 * Original Arthur::Paul Clark Based on original code by: Owen Lyke @ SparkFun Electronics 
 * Original Creation Date: April 17 2019
 * 
 ***** Important note: by default the DMP functionality is disabled in the library  *****
 * ** as the DMP firmware takes up 14301 Bytes of program memory.
 * ** To use the DMP, you will need to:
 * ** Edit ICM_20948_C.h
 * ** Uncomment line 29: #define ICM_20948_USE_DMP
 * ** Save changes
 * ** If you are using Windows, you can find ICM_20948_C.h in:
 * ** Documents\Arduino\libraries\SparkFun_ICM-20948_ArduinoLibrary\src\util
 *
 * Please see License.md for the license information.
 *
 * Distributed as-is; no warranty is given.
 
 ***** Code was Improved by Professor Benjamas Panomruttanarug "BP"  *****
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
        
 * Comment Testing
        
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

#include <math.h> // Editedby::AlexHong

rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

//BP added 
// * Echoing Message::Noteby::AlexHong
rcl_publisher_t string_publisher;
rcl_publisher_t pedometer_publisher;

rcl_publisher_t linearVel_publisher;                 //Editedby::AlexHong
rcl_publisher_t yaw_publisher;   
rcl_publisher_t xpos_publisher;                 //Editedby::AlexHong
rcl_publisher_t ypos_publisher;                 //Editedby::AlexHong         
//rcl_publisher_t err_publisher;                //Editedby::AlexHong (Remove Comment to test with Error Message)

std_msgs__msg__String string_msg;
std_msgs__msg__Int32 pedometer_msg;

//Topic Name Declaration::Editedby::AlexHong 
std_msgs__msg__Float32 linearVel_msg;           
std_msgs__msg__Float32 deltaYaw_msg;
std_msgs__msg__Float32 xpos_msg;                
std_msgs__msg__Float32 ypos_msg; 
//std_msgs__msg__Float32 err_msg;               //Remove Comment to test with Error Message
               

//Topic Name Declaration::Editedby::AlexHong
const char * yaw_tpName = "yaw";
const char * xvel_tpName = "linearVel";    
const char * xpos_tpName = "xpos";
const char * ypos_tpName = "ypos";  
//const char * err_tpName = "err";

//Message Type Support::Editedby::AlexHong
const rosidl_message_type_support_t * type_support =
  ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32);



#define LED_PIN 13 //Error Check::Noteby::AlexHong

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

#include "ICM_20948.h" // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU

#define USE_SPI       // Used SPI::Editedby::AlexHong

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



//Declaring Global Variables for RPM Sensor::Editedby::AlexHong

const int sensorPin = 4;             // Connect the RPM sensor to digital pin 2 //AttachInterrupt Pins -> 0,1,4,5,6,7,8,9,A1,A2 for MKR Family Board

volatile unsigned long pulseCount = 0;
unsigned long lastTime = 0;
const int sampleInterval = 100000;  // Time interval for calculating RPM in microseconds

// Vehicle's Wheel Parameter
const float wheelRadius = 0.0525;   // Wheel Diameter in meter value

// Parameters for x_velocity, yaw value, previous and current yaw :: Edited by AlexHong

double linear_value = 0.0;

double yaw_value = 0.0;
double previousYaw = 0.0;
double currentYaw = 0.0;

double linear_vel = 0.0;            // Used inside class

double xpos_data = 0.0;             // X & Y Position Declaration 
double ypos_data = 0.0;             // 

// End::Noteby::AlexHong



// YawSensor and Linear Velocity Class::Createdby::AlexHong

inline double to_radians(double degree) {
    return degree * (M_PI / 180);
}

class YawSensor{

   public:

    void error_loop(void){
      while(1){
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
        delay(100);
      }
    }


    void yawSetup( void )
    {
          if (myICM.status != ICM_20948_Stat_FIFOMoreDataAvail) // If more data is available then we should read it right away - and not delay
          {
            delay(10);
          }
          
          SPI_PORT.begin();
          
          bool initialized = false;
          while (!initialized)
          {
            // Initialize the ICM-20948
            // If the DMP is enabled, .begin performs a minimal startup. We need to configure the sample mode etc. manually.
            myICM.begin(CS_PIN, SPI_PORT);
        
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
        
          // Initialize the DMP. initializeDMP
          success &= (myICM.initializeDMP() == ICM_20948_Stat_Ok);
        
      
        
          // Enable the DMP Game Rotation Vector sensor
          success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR) == ICM_20948_Stat_Ok);
        
          // Configuring DMP to output data at multiple ODRs:
          // DMP is capable of outputting multiple sensor data at different rates to FIFO.
          // Setting value can be calculated as follows:
          // Value = (DMP running rate / ODR ) - 1
          // E.g. For a 5Hz ODR rate when DMP is running at 55Hz, value = (55/5) - 1 = 10.
          
          success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Quat6, 0) == ICM_20948_Stat_Ok); // Set to the maximum
        
          // Enable the FIFO
          success &= (myICM.enableFIFO() == ICM_20948_Stat_Ok);
        
          // Enable the DMP
          success &= (myICM.enableDMP() == ICM_20948_Stat_Ok);
        
          // Reset DMP
          success &= (myICM.resetDMP() == ICM_20948_Stat_Ok);
        
          // Reset FIFO
          success &= (myICM.resetFIFO() == ICM_20948_Stat_Ok);



//MicroROS Initiated Here::Noteby::AlexHong         
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
        
          RCCHECK(rclc_publisher_init_default(
            &linearVel_publisher,
            &node,
            type_support,
            xvel_tpName)); 
          
          RCCHECK(rclc_publisher_init_default(
            &yaw_publisher,
            &node,
            type_support,
            yaw_tpName));  

/*
 * Only Two Publishers are able to used for now. 
 * If it is more than two publishers, the program stops working at all
 * Task to myself ::NeedtoFix::AlexHong
 */

//          RCCHECK(rclc_publisher_init_default(
//            &xpos_publisher,
//            &node,
//            type_support,
//            xpos_tpName));
//
//          RCCHECK(rclc_publisher_init_default(
//            &ypos_publisher,
//            &node,
//            type_support,
//            ypos_tpName));  
     }

    double DeltaYaw( void ){

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

            /*
             * Important Noteby::AlexHong
             * Dont use SERIAL_PORT.printf(...) cases with ROS Message publishers, it will cause an Error - 
             */
            //SERIAL_PORT.printf("Quat6 data is: Q1:%ld Q2:%ld Q3:%ld\r\n", data.Quat6.Data.Q1, data.Quat6.Data.Q2, data.Quat6.Data.Q3);
            //
      
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
            double yaw = atan2(t3, t4) * 180.0 / PI;
            
            currentYaw = yaw;

            double delta = currentYaw - previousYaw;
            
            if (delta > 180.0) delta -= 360.0;
            else if (delta < -180.0) delta += 360;
            
            previousYaw = currentYaw;
            
            return delta;   
            
          }
        }
        
        if (myICM.status != ICM_20948_Stat_FIFOMoreDataAvail) // If more data is available then we should read it right away - and not delay
        {
          delay(10);
        }
        
     }

};

// This Function belongs to LinearVel_Data Class
void countPulses(void){
      if(digitalRead(sensorPin)==HIGH) pulseCount++;
      else pulseCount;
}
    
class linearVel_Data
{
  public:

  void linearVelSetup(void){
    // Editedby::AlexHong
    // RPM Sensor Connection
        pinMode(sensorPin, INPUT_PULLUP);
        attachInterrupt(digitalPinToInterrupt(sensorPin), countPulses, RISING); 
  }

  double get_linearVel( double linear_vel )
     {

      unsigned long currentTime = micros();
      unsigned long elapsedTime = currentTime - lastTime;
  
      // Ensure a minimum time has passed to avoid division by zero
      if (elapsedTime >= sampleInterval) {
        if (elapsedTime != 0){
          // Calculate RPM
          double rpm = (pulseCount * 60000000) / (elapsedTime); // Multiplying by 60,000 is used to convert the time from milliseconds to minutes (since there are 60,000 milliseconds in a minute)
    
    
            
            // Calculate Angular Velocity of Driven Gear
          double angularVelocity = (2 * PI * rpm) / 60 ; // angular velocity of Driven Gear from Brushless Motor in rad/s // Divided by 60 to change it to second
            
            // Calculate Linear Velocity
          double linear_vel = angularVelocity * wheelRadius; // linear velocity of gear A in Meter/sec
                
                // Reset pulse count and update last time
          pulseCount = 0;
          lastTime = currentTime;

          return linear_vel;  
                   
          }
          else return linear_vel;  
        } 
        else return linear_vel;  
     }
};

YawSensor yawSensor;
linearVel_Data linearVelData;
//Class End::Editedby::AlexHong


void setup()
{          
    yawSensor.yawSetup();
    linearVelData.linearVelSetup();
}


void loop()
{ 

// Tested::By::AlexHong::InitialTest::Pass
    yaw_value = yawSensor.DeltaYaw();
    deltaYaw_msg.data = yaw_value;
    RCSOFTCHECK(rcl_publish(&yaw_publisher, &deltaYaw_msg, NULL));
  
    linear_value = linearVelData.get_linearVel(linear_value); 
    linearVel_msg.data = linear_value; 
    RCSOFTCHECK(rcl_publish(&linearVel_publisher, &linearVel_msg, NULL));

    delay(500);

// Tested::NotWorking::By::AlexHong::InitialTest::Failed::OnlyTwoPublisherWorked
//    xpos_data += linear_value * cos(to_radians(yaw_value));
//    xpos_msg.data = xpos_data;
//    RCSOFTCHECK(rcl_publish(&xpos_publisher, &xpos_msg, NULL));
//
//    ypos_data += linear_value * sin(to_radians(yaw_value));
//    ypos_msg.data = ypos_data;
//    RCSOFTCHECK(rcl_publish(&ypos_publisher, &ypos_msg, NULL));
       
}
