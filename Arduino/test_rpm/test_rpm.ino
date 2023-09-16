// Declaring Global Variables 

const int sensorPin = 4; // Connect the RPM sensor to digital pin 2 //AttachInterrupt Pins -> 0,1,4,5,6,7,8,9,A1,A2 for MKR Family Board

volatile unsigned long pulseCount = 0;
unsigned long lastTime = 0;
const int sampleInterval = 1000; // Time interval for calculating RPM in 1 second = 1000 millisec

//const int pulsesPerRevolution = 2UL;

void setup() {
  Serial.begin(115200);
  pinMode(sensorPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(sensorPin), countPulses, CHANGE);
  
}

void loop() {
  unsigned long currentTime = millis();
  unsigned long elapsedTime = currentTime - lastTime;
  
// Ensure a minimum time has passed to avoid division by zero
  if (elapsedTime >= sampleInterval) {

    // Calculate RPM
    unsigned long RPM = (pulseCount * 60000UL) / (elapsedTime ); // Multiply by 60000 to convert to RPM

    /*
    pulseCount * 60000UL (UL : Forced constant into an Unsigned Long Constant) 
    
    This part of the equation calculates the total number of pulses per minute (similar to the first equation). 
    It multiplies pulseCount by 60,000 to convert pulses to pulses per minute (PPM).

    (elapsedTime * pulsesPerRevolution): This part of the equation represents the total time elapsed 
    in milliseconds for the pulses to be counted. It multiplies elapsedTime 
    by 2UL to account for the assumption that each pulse represents a complete rotation.
    */
    
// Print RPM to Serial Monitor
    
    Serial.print("RPM: ");
    Serial.println(RPM);

    // Reset pulse count and update last time  
    pulseCount = 0;
    lastTime = currentTime;
  }
}

//ISR (Interrupt Service Routine) Function -> when interrupt occurs, Function takes no parameter and return nothing
void countPulses() {
  if(digitalRead(sensorPin) == HIGH) 
    pulseCount++;
}
