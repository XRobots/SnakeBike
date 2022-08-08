#include <Servo.h>
Servo ESC1;   //   throttle ESC

#include <PID_v1.h>

double Pk1 = 2; 
double Ik1 = 1;
double Dk1 = 0;

double Setpoint1, Input1, Output1, Output1a;    // PID variables
PID PID1(&Input1, &Output1, &Setpoint1, Pk1, Ik1 , Dk1, DIRECT);    // PID Setup

double Pk2 = 2; 
double Ik2 = 1;
double Dk2 = 0;

double Setpoint2, Input2, Output2, Output2a;    // PID variables
PID PID2(&Input2, &Output2, &Setpoint2, Pk2, Ik2 , Dk2, DIRECT);    // PID Setup

// Radio
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(7, 8); // CE, CSN
const byte addresses[][6] = {"00001", "00002"};

//**************remote control****************
struct RECEIVE_DATA_STRUCTURE{
  //put your variable definitions here for the data you want to send
  //THIS MUST BE EXACTLY THE SAME ON THE OTHER ARDUINO
    int16_t menuDown;  
    int16_t Select;    
    int16_t menuUp;  
    int16_t toggleBottom;  
    int16_t toggleTop; 
    int16_t toggle1;
    int16_t toggle2;
    int16_t mode;  
    int16_t RLR;
    int16_t RFB;
    int16_t RT;
    int16_t LLR;
    int16_t LFB;
    int16_t LT;
};

RECEIVE_DATA_STRUCTURE mydata_remote;

int RLR = 0;
int RFB = 0;
int RFBa = 0;
int RT = 0;
int LLR = 0;
int LFB = 0;
int LT = 0;

unsigned long currentMillis;
long previousMillis = 0;    // set up timers
long interval = 10;        // time constant for timer
long previousSafetyMillis;    // timer to check the data is there or stop on safties


int pot1;
int pot2;

int throttleL;
int throttleR;
float bendH;
float bendHFiltered;
float bendV;
float bendVFiltered;
int servoL;
int servoR;

int enable;


void setup() {

    // initialize serial communication
    Serial.begin(115200);
    Serial3.begin(115200);
    
    radio.begin();
    radio.openWritingPipe(addresses[0]); // 00002
    radio.openReadingPipe(1, addresses[1]); // 00001
    radio.setPALevel(RF24_PA_MIN);
    radio.startListening();

    pinMode(2,OUTPUT);      // wiper PWMs
    pinMode(3,OUTPUT);
    pinMode(4,OUTPUT);
    pinMode(5,OUTPUT);

    PID1.SetMode(AUTOMATIC);              
    PID1.SetOutputLimits(-127, 127);
    PID1.SetSampleTime(10);

    PID2.SetMode(AUTOMATIC);              
    PID2.SetOutputLimits(-127, 127);
    PID2.SetSampleTime(10);

    ESC1.attach(26);
    ESC1.writeMicroseconds(1300);
   
}   // end of setup

// ********************* MAIN LOOP *******************************

void loop() {  

      
        currentMillis = millis();
        if (currentMillis - previousMillis >= 10) {  // start timed event
          
            previousMillis = currentMillis;


            // check for radio data
            if (radio.available()) {
                    radio.read(&mydata_remote, sizeof(RECEIVE_DATA_STRUCTURE));  
                    previousSafetyMillis = currentMillis;   
            }

            else if (currentMillis - previousSafetyMillis >= 300) {   
                  Serial.println("STOP");
                  mydata_remote.RFB = 512;
                  mydata_remote.RLR = 512;
                  mydata_remote.RT = 512;
                  mydata_remote.LFB = 512;
                  mydata_remote.LLR = 512;
                  mydata_remote.LT = 512;
            }

            else {
              Serial.println("no data");              
            }

            // threshold remote data
            // some are reversed based on stick wiring in remote
            RFB = thresholdStick(mydata_remote.RFB)/-2;   
            LFB = thresholdStick(mydata_remote.LFB);
            LT = thresholdStick(mydata_remote.LT)*0.9; 
            RT = thresholdStick(mydata_remote.RT)*0.9;

            if (mydata_remote.toggleTop == 1) {
              bendH = RT;
            }
            else {
              bendH = LT;
            }
            
            throttleL = RFB;
            throttleR = RFB;
            bendV = LFB;

            bendHFiltered = filter(bendH, bendHFiltered,30);
            bendVFiltered = filter(bendV, bendVFiltered,30);
           
            servoL = bendVFiltered - bendHFiltered;
            servoR = bendVFiltered + bendHFiltered;

            servoL = constrain(servoL,-220,220);
            servoR = constrain(servoR,-220,220);

            enable = mydata_remote.toggleBottom;

            if (enable == 1) {

                  // control right servo
      
                  pot1 = analogRead(A0);
                  pot1 = (pot1 - 512)*-1;
      
                  Input1 = pot1;
                  Setpoint1 = servoR;
                  PID1.Compute();
      
                  if (Output1 > 0) {
                    analogWrite(3, Output1);
                    analogWrite(2, 0);
                  }
                  else if (Output1 < 0) {
                    Output1a = abs(Output1);
                    analogWrite(2, Output1a);
                    analogWrite(3, 0);
                  }
                  else {
                    analogWrite(2, 0);
                    analogWrite(3, 0);
                  }
      
                  // control right servo
      
                  pot2 = analogRead(A1);
                  pot2 = pot2 - 512;
      
                  Input2 = pot2-60;   // trim value for centre
                  Setpoint2 = servoL;
                  PID2.Compute();   
      
      
                  if (Output2 > 0) {
                    analogWrite(4, Output2);
                    analogWrite(5, 0);
                  }
                  else if (Output2 < 0) {
                    Output2a = abs(Output2);
                    analogWrite(5, Output2a);
                    analogWrite(4, 0);
                  }
                  else {
                    analogWrite(4, 0);
                    analogWrite(5, 0);
                  }
                  
                  ESC1.writeMicroseconds(1300 + throttleL);
            }

            else {
                  analogWrite(2, 0);
                  analogWrite(3, 0);
                  analogWrite(4, 0);
                  analogWrite(5, 0);
                  ESC1.writeMicroseconds(1300);
            }

            // send data to other nodes 
            Serial3.print(800);           // identifier for the start of the data
            Serial3.print(" , ");
            Serial3.print(throttleL);
            Serial3.print(" , ");
            Serial3.print(throttleR);
            Serial3.print(" , ");
            Serial3.print(servoL);
            Serial3.print(" , ");
            Serial3.print(servoR);
            Serial3.print(" , ");
            Serial3.print(enable);
            Serial3.print(" , ");    


            
        }     // end of timed loop         

   
}       // end  of main loop
