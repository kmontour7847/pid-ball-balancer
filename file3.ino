
#include<Servo.h>
#include<PID_v1.h>
#include<SoftwareSerial.h>

int set=17.35,neg=-24,pos=24,base=90;//Setpoint,Negative,Positive,Base values.neg shows tilt on other side of ultrasonic sensor   6,-24,24,130
long cm1=set;                     //For filtering purpose
const double a=0.5;               //Exponential filter parameter
const int servoPin = 9;           //Servo pin


 
float Kp = 12;                                                    //Initial Proportional Gain  0.8     get ball to setpoint
float Ki = 2;                                                      //Initial Integral Gain 0.02        balance
float Kd = 6.5;                                                    //Intitial Derivative Gain 0.75      stop ball at setpoint
double Setpoint, Input, Output, ServoOutput;                                       
 


PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, REVERSE);           //Initialize PID object, which is in the class PID.
                                                                      
                                                                     
                                                                     
                                                                     
Servo myServo;                                                       //Initialize Servo.


void setup() {

  Serial.begin(9600);                                                //Begin Serial 
  myServo.attach(servoPin);                                          //Attach Servo

  Input = readPosition();                                            //Calls function readPosition() and sets the balls
                                                                     //  position as the input to the PID algorithm
                                                                     
  
 
  myPID.SetMode(AUTOMATIC);                                         //Set PID object myPID to AUTOMATIC 
  myPID.SetOutputLimits(neg,pos);                                   //Set Output limits to neg and pos degrees. 
}

void loop()
{
 
  Setpoint = 15.5;                                                      //Give value for setpoint
  Input = readPosition();                                            
 
  myPID.Compute();                                                   //computes Output in range of neg to pos degrees
  
  ServoOutput=base+Output;                                            // value in base is my horizontal 
  myServo.write(ServoOutput);                                        //Writes value of Output to servo
 
  
  
}
      
      
      

float readPosition() {
  delay(40);                                                            
  
  
const int TrigPin = 11;//Trig
const int EchoPin = 12;//Echo

long duration, cm,cmn;
unsigned long now = millis();
pinMode(TrigPin, OUTPUT);
digitalWrite(TrigPin, LOW);
delayMicroseconds(2);
digitalWrite(TrigPin, HIGH);
delayMicroseconds(5);
digitalWrite(TrigPin, LOW);


pinMode(EchoPin, INPUT);
duration = pulseIn(EchoPin, HIGH);

  cm = duration/(50*2);
  
  
  
  if(cm > 100)              // 30 cm is the maximum position for the ball, change back to 30/15 if needed
  {cm=100;}                //Signal Conditioning for ultrasonic sensor  

  cmn = a * cm + (1 - a) * cm1;     //Exponential filter- signal conditioning 
  Serial.print(cm); Serial.print("\t");
  Serial.println(cmn);            //cmn is filtered value
  delay(10);
  cm1 = cmn;                      //saved to cm1 which is used as history in exponential filter
  
  return (cmn);                           //Returns filtered distance value in cm
}
