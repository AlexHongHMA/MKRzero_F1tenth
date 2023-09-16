/*
Important Noteby::Alexander Hong

Here, our RPM sensor is actually recording data from large Gear A (not the pinion from brushless motor). Therefore, we don't need
to calculate angular velocity of gear B (pinion gear). 

if the RPM sensor is recording data from the motor's pinion or from the motor's shelf directly, 
we need to apply this equation 
          
          w(A) = w(B)/gearRatio (check the equation back) where :: w(B) = 2*PI*rpm/60

In case, if we need to calculate GearRatio related equations, 
        gearRatio = Teeth of Driven/ Teeth of Driver
        
Gear Information for this robot - pinion -> 23T , Driven Gear -> 86T,48P  
*/


// Declaring Global Variables 

const int sensorPin = 4; // Connect the RPM sensor to digital pin 2 //AttachInterrupt Pins -> 0,1,4,5,6,7,8,9,A1,A2 for MKR Family Board

volatile unsigned long pulseCount = 0;
unsigned long lastTime = 0;
const int sampleInterval = 1000; // Time interval for calculating RPM in milliseconds

// Vehicle's Wheel Parameter
const float wheelRadius = 0.0525; // Wheel Diameter in meter value
//const float gearRatio = 3.7391; // Transmission Gear Type // Need to change Gear Ratio values if you are using different vehicles

void setup() {
  Serial.begin(115200);
  pinMode(sensorPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(sensorPin), countPulses, RISING); // attachInterrupt(digitalPinToInterrupt(pin), ISR, mode); :: 
                                                                          // check at https://www.arduino.cc/reference/en/language/functions/external-interrupts/attachinterrupt/
}

void loop() {
  unsigned long currentTime = millis();
  unsigned long elapsedTime = currentTime - lastTime;
  
// Ensure a minimum time has passed to avoid division by zero
  if (elapsedTime >= sampleInterval) {
    if (elapsedTime != 0){
// Calculate RPM
    int rpm = (pulseCount * 60000) / (elapsedTime); // Multiplying by 60,000 is used to convert the time from milliseconds to minutes (since there are 60,000 milliseconds in a minute)


// Calculate Angular Velocity of Driven Gear
    float angularVelocityGearA = (2 * PI * rpm) / 60 ; // angular velocity of Pinion from Brushless Motor in rad/s // Divided by 60 to change it to second

// Calculate Linear Velocity
    float linear_vel = angularVelocityGearA * wheelRadius; // linear velocity of gear A in Meter/sec
    

    Serial.print("rpm: ");
    Serial.println(rpm);

    Serial.print("Pulse Count: ");
    Serial.println(pulseCount);

    Serial.print("LinearVel: ");
    Serial.println(linear_vel);

    // Reset pulse count and update last time
    
    pulseCount = 0;
    lastTime = currentTime;
  }
  else
  {
    Serial.println("Elapsed Time is Zero, check the hardware");
  }
}
}

void countPulses() {
  if(digitalRead(sensorPin)==HIGH) pulseCount++;
  else pulseCount;
} 
