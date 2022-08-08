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

int var1;
int var2;
int var3;
int var4;
int check;

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

unsigned long currentMillis;
long previousMillis = 0;    // set up timers
long interval = 5;        // time constant for timer

long previousSafetyMillis;    // timer to check the data is there or stop on safties

void setup() {

    Serial.begin(115200);
    Serial3.begin(115200);

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

}

void loop() {


        currentMillis = millis();
        if (currentMillis - previousMillis >= interval) {  // start timed event
          
            previousMillis = currentMillis;

            if (Serial3.available() > 0){
  
                check = Serial3.parseInt();
                if (check == 800) {               //wait fot the check value to come around before reading the rest of the data
                      throttleL = Serial3.parseInt();
                      throttleR = Serial3.parseInt();
                      servoL = Serial3.parseInt();
                      servoR = Serial3.parseInt();
                      enable = Serial3.parseInt();
                }
            }

            pot1 = analogRead(A0);
            pot1 = (pot1 - 512)*-1;                   
            
            if (enable == 1) {

                Input1 = pot1-50;
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


            


        } // end of timed loop









}
