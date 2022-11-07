//********************************************************************************************
// RC Model Servo
// Original code By EAOROBBIE (Robert Lindsay)
// Completely mangled by aarondc
// Modify by Jun :P
// For free use for Sim Tool Motion Software
//********************************************************************************************
#include <Servo.h>
//#define DEBUG 1                                    // comment out this line to remove debuggin Serial.print lines
const int kActuatorCount = 6;                       // how many Actuators we are handling

// the letters ("names") sent from Sim Tools to identify each actuator
// NB: the order of the letters here determines the order of the remaining constants kPins and kActuatorScale
const char kActuatorName[kActuatorCount] = { 'A', 'B', 'C', 'D', 'E', 'F' };
const int kPins[kActuatorCount] = {7, 2, 3, 4, 5, 6};                    // pins to which the Actuators are attached
/*
const int kActuatorScale[kActuatorCount][6] = { { 0, 179 } ,    // A Actuator scaling
                                                { 179, 0 } ,    // B Actuator scaling
                                                { 0, 179 } ,    // C Actuator scaling
                                                { 179, 0 } ,    // D Actuator scaling
                                                { 0, 179 } ,    // E Actuator scaling
                                                { 179, 0 }      // F Actuator scaling
                                               }; 
*/

// Calibrated values for neutral servo position "Test_Servos_6DOF_degrees.ino" file
const int Init_1 = 94;
const int Init_2 = 90;
const int Init_3 = 95;
const int Init_4 = 96;
const int Init_5 = 82;
const int Init_6 = 94;

 // Servo range 1 way in degrees
const int range = 70;

const int kActuatorScale[kActuatorCount][6] = { { Init_1 - range, Init_1 + range -1 } ,    // A Actuator scaling
                                                { Init_2 + range -1, Init_2 - range } ,    // B Actuator scaling
                                                { Init_3 - range, Init_3 + range -1  } ,    // C Actuator scaling
                                                { Init_4 + range -1, Init_4 - range } ,    // D Actuator scaling
                                                { Init_5 - range, Init_5 + range -1  } ,    // E Actuator scaling
                                                { Init_6 + range -1, Init_6 - range }      // F Actuator scaling
                                               };                                                 
const char kEOL = '~';                              // End of Line - the delimiter for our acutator values 
const int kMaxCharCount = 3;                        // some insurance...
Servo actuatorSet[kActuatorCount];                  // our array of Actuators
//int actuatorPosition[kActuatorCount] = {90, 90, 90, 90, 90, 90};    // current Actuator positions, initialised to 90
int actuatorPosition[kActuatorCount] = {Init_1, Init_2, Init_3, Init_4, Init_5, Init_6};    // current Actuator positions, initialised to 90
int currentActuator;                                // keep track of the current Actuator being read in from serial port
int valueCharCount = 0;                             // how many value characters have we read (must be less than kMaxCharCount!!

// set up some states for our state machine
// psReadActuator = next character from serial port tells us the Actuator
// psReadValue = next 3 characters from serial port tells us the value
enum TPortState { psReadActuator, psReadValue };   
TPortState currentState = psReadActuator;

void setup()
{ 
    // attach the Actuators to the pins
    for (int i = 0; i < kActuatorCount; i++) 
        actuatorSet[i].attach(kPins[i]);
    
    // initialise actuator position
    for (int i = 0; i < kActuatorCount; i++) 
        updateActuator(i);
    
    Serial.begin(9600); // opens serial port at a baud rate of 9600
} 
 
void loop()
{ 

}

// this code only runs when we have serial data available. ie (Serial.available() > 0).
void serialEvent() {
    char tmpChar;
    int tmpValue;

    while (Serial.available()) {
        // if we're waiting for a Actuator name, grab it here
        if (currentState == psReadActuator) {
            tmpChar = Serial.read();
            // look for our actuator in the array of actuator names we set up 
#ifdef DEBUG            
Serial.print("read in ");            
Serial.println(tmpChar);            
#endif
            for (int i = 0; i < kActuatorCount; i++) {
                if (tmpChar == kActuatorName[i]) {
#ifdef DEBUG            
Serial.print("which is actuator ");            
Serial.println(i);            
#endif
                    currentActuator = i;                        // remember which actuator we found
                    currentState = psReadValue;                 // start looking for the Actuator position 
                    actuatorPosition[currentActuator] = 0;      // initialise the new position
                    valueCharCount = 0;                         // initialise number of value chars read in 
                    break;
                }
            }
        }
        
        // if we're ready to read in the current Actuator's position data
        if (currentState == psReadValue) {
            while ((valueCharCount < kMaxCharCount) && Serial.available()) {
                tmpValue = Serial.read();
                if (tmpValue != kEOL) {
                    tmpValue = tmpValue - 48;
                    if ((tmpValue < 0) || (tmpValue > 9)) tmpValue = 0;
                    actuatorPosition[currentActuator] = actuatorPosition[currentActuator] * 10 + tmpValue;
                    valueCharCount++;
                }
                else break;
            }
            
            // if we've read the value delimiter, update the Actuator and start looking for the next Actuator name
            if (tmpValue == kEOL || valueCharCount == kMaxCharCount) {
#ifdef DEBUG            
Serial.print("read in ");            
Serial.println(actuatorPosition[currentActuator]);            
#endif
                // scale the new position so the value is between 0 and 179
                actuatorPosition[currentActuator] = map(actuatorPosition[currentActuator], 0, 255, kActuatorScale[currentActuator][0], kActuatorScale[currentActuator][1]);
#ifdef DEBUG            
Serial.print("scaled to ");            
Serial.println(actuatorPosition[currentActuator]);            
#endif
                updateActuator(currentActuator);
                currentState = psReadActuator;
            }
        }
    }
}


// write the current Actuator position to the passed in Actuator 
void updateActuator(int thisActuator) {
    actuatorSet[thisActuator].write(actuatorPosition[thisActuator]);
}
