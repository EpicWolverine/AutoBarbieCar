/* Servo */
//#include <Servo.h> 
//Servo turnServo;  //create servo object to control a servo 

/* Compass */
#include <Average.h> //import Average library for compass initialization
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(12345); //compass object
const float Pi = 3.14159; //apparently the pi constant isn't built into Arduino
float compassStraight = 0.0; //this will hold what we decide as "straight"

/* Turning */
/*
const byte turnServoPin = 9; //pin for servo motor
byte turnServoDirection = 1; //0=left; 1=center; 2=right
//byte turnServoLastDirection = 1;
*/

/* Drive Motor */
/*
const byte driveMotorDirectionPin = 4; //pin for drive motor direction control
const byte driveMotorSpeedPin = 5; //pin for drive motor speed control
//unsigned long driveMotorUpdateDuration; //will store last time motor was updated
unsigned long driveMotorPreviousMillis; //will store last time motor was updated
int driveMotorDelay; //how long to wait before updating the motor during manuver
boolean driveMotorDirection = false; //false=forward; true=backward
//boolean driveMotorLastDirection = false;
*/

/* Monster Motor Sheild */
#define BRAKEVCC 0
#define CW   1 //Clockwise Arguement in motorGo
#define CCW  2 //Counterclockwise Arguement in motorGo
#define BRAKEGND 3
#define CS_THRESHOLD 100
int inApin[2] = {7, 4};
int inBpin[2] = {8, 9};
int pwmpin[2] = {5, 6};
unsigned long driveMotorPreviousMillis; //will store last time motor was updated
int driveMotorDelay; //how long to wait before updating the motor during manuver
byte turnServoDirection = 1; //0=left; 1=center; 2=right

/* Distance Sensors */
int FLDistSensorVal;          //value from the distance Front Left sensor
//int FLDistSensorValLast;  //last value from the distance Front Left sensor
const byte FLDistSensorPin = 2;   //Front Left sensor analog pin
boolean FLDetect = false; //Front Left sensor detection
int FRDistSensorVal;          //value from the distance Front Right sensor
//int FRDistSensorValLast;  //last value from the distance Front Right sensor
const byte FRDistSensorPin = 3;   //Front Right sensor analog pin
boolean FRDetect = false; //Front Right sensor detection
int distThreshold = 30;     //threshold for the distance sensor

boolean delayOverride = false;

/* Color Sensor */
#include <Wire.h> //Remember to get the Library Downloaded
#include "Adafruit_TCS34725.h" //Another Part of the Library
#define commonAnode true //Absolutely Vital; No idea what it does
byte gammatable[256]; //See above comment
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X); //I^2C communications from sensor to board
int ColorSensorVal = 0; // 0=white/tarmac; 1=red; 2=green; 3=yellow;

void setup(){
    randomSeed(analogRead(3)); //initialize random seed
    Serial.begin(9600); //start Serial
  
    /* Set Pins */
    pinMode(FLDistSensorPin, INPUT);
    pinMode(FRDistSensorPin, INPUT);
    //pinMode(10, OUTPUT); //left LED (red)
    //pinMode(11, OUTPUT); //center LED (yellow)
    //pinMode(12, OUTPUT); //right LED (green)
    //pinMode(driveMotorDirectionPin, OUTPUT); 
    //turnServo.attach(turnServoPin);  //attaches the servo to the servo object
    for (int i=0; i<2; i++){
      pinMode(inApin[i], OUTPUT);
      pinMode(inBpin[i], OUTPUT);
      pinMode(pwmpin[i], OUTPUT);
    }
    
    /* Initilize */
    //drive motor
    for (int i=0; i<2; i++){ //initilize motors, not running
      digitalWrite(inApin[i], LOW);
      digitalWrite(inBpin[i], LOW);
    }
    //compass
    if(!mag.begin()){ //initilize compass
        //There was a problem detecting the LSM303 ... check your connections
        Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
        while(1); //stall forever
    }
    else{
        initCompass(); //initilize the compass; set the direction we will use as "straight"
    }
    //distance sensors
    FLDistSensorVal = readSensorRaw(FLDistSensorPin);
    FRDistSensorVal = readSensorRaw(FRDistSensorPin);
    //color sensor
    Serial.println("Color View Test!");
    if (tcs.begin()){
      Serial.println("Found sensor");
    }
    else{
      Serial.println("No TCS34725 found ... check your connections");
      while (1);
    }
    for (int i = 0; i < 256; i++){
      float x = i;
      x /= 255;
      x = pow(x, 2.5);
      x *= 255;
       if (commonAnode){
         gammatable[i] = 255 - x;
       }
       else{
         gammatable[i] = x;
       }
    }
}

void loop(){
    delay(5);
    //read distance sensors
    FLDistSensorVal = readSensorCorrected(FLDistSensorPin, FLDistSensorVal);
    FRDistSensorVal = readSensorCorrected(FRDistSensorPin, FRDistSensorVal);
    //print distence 
    Serial.print('a');
    Serial.println(FLDistSensorVal);
    Serial.print('b');
    Serial.println(FRDistSensorVal);
    Serial.println("--");
    
    //check for distance sensor detection
    if(FLDistSensorVal < distThreshold){FLDetect=true;}
    if(FRDistSensorVal < distThreshold){FRDetect=true;}
    
    //check if we need to override a manuver
    if (FLDetect==true && FRDetect==true){ //both
        delayOverride = true;
    }
    else if(FLDetect==true && turnServoDirection != 0){ //left
        delayOverride = true;
    }
    else if(FRDetect==true && turnServoDirection != 2){ //right
        delayOverride = true;
    }
    
    //main manuver logic
    if((millis() - driveMotorPreviousMillis) > driveMotorDelay || delayOverride == true){ //only change if delay is done or overriding manuver 
        delayOverride = false; //reset override flag, if any
        driveMotorPreviousMillis = millis(); //set up for next delay
        
        if (driveMotorDelay > 0){ //code to run after a manuver is completed
            initCompass(); //reset the compass
        }
        
        //desision tree based on distance sensors
        if(FLDetect==false && FRDetect==false){ //forward
            forward(150);
            driveMotorDelay = 0;
            straightenCompass();
        }
        else if (FLDetect==true && FRDetect==true){ //backward
            backward(150);
            straight();
            driveMotorDelay = 2000;
        }
        else if(FLDetect==true){  //left
            //backward(150);
            left();
            driveMotorDelay = 2000;
        }
        else if(FRDetect==true){  //right
            //backward(150);
            right();
            driveMotorDelay = 2000;
        }
    }
    
    //reset distance sensor detect flags
    FLDetect=false;
    FRDetect=false;
    //reset distance sensor detect LEDs
    //digitalWrite(10,LOW);
    //digitalWrite(11,LOW);
    //digitalWrite(12,LOW);
}



/* Basic Manuver Functions */
void forward(int velocity){ //drive forward
    motorGo(0, CW, 1023);
    motorGo(1, CCW, 1023);
    //digitalWrite(driveMotorDirectionPin, LOW); //LOW is forward
    //analogWrite(driveMotorSpeedPin, velocity);
    //driveMotorDirection = false;
    //driveMotorLastDirection = false;
    //Serial.println("forward");
}

void backward(int velocity){ //drive backward
    delay(50);
    motorGo(0, CCW, 1023);
    motorGo(1, CW, 1023);
    //digitalWrite(driveMotorDirectionPin, HIGH); //HIGH is backward
    //analogWrite(driveMotorSpeedPin, velocity);
    //driveMotorDirection = true;
    //driveMotorLastDirection = true;
    //Serial.println("backward");
}

void straight(){ //turn wheels straight
    //turnServo.write(79);  //turn servo to x degrees
    turnServoDirection = 1;
    //turnServoLastDirection = 1
    //Serial.println("straight");
}

void left(){ //turn wheels left
    motorGo(0, CW, 1023);
    motorGo(1, CW, 1023);
    //turnServo.write(66); //turn servo to x degrees
    turnServoDirection = 0;
    //turnServoLastDirection = 0;
    //Serial.println("left");
}

void right(){ //turn wheels right
    motorGo(0, CCW, 1023);
    motorGo(1, CCW, 1023);
    //turnServo.write(89); //turn servo to x degrees
    turnServoDirection = 2;
    //turnServoLastDirection = 2;
    //Serial.println("right");
}

/* Advanced Manuver Functions */
void search(){ //searches for a direction to drive
    
}

/* Compass Functions */
void initCompass(){ //initilize the compass; set the direction we will use as "straight"
    Average<float> ave(5);
    for(byte x=0; x<5; x++){ //take five samples
       ave.push(getCompass());
    }
    compassStraight = ave.mean(); //average the samples and use as "straight"
    Serial.print("Initial Compass Average (will use as straight):");
    Serial.println(compassStraight);
}

float getCompass(){ //read from the compass, convert to 360 degrees, and return as float
    if(!mag.begin()){ //initilize compass
        //There was a problem detecting the LSM303 ... check your connections
        Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    }
    //Get a new sensor event
    sensors_event_t event; 
    mag.getEvent(&event);
    
    //Calculate the angle of the vector y,x
    float heading = (atan2(event.magnetic.y,event.magnetic.x) * 180) / Pi;
    
    //Normalize to 0-360
    if (heading < 0){
        heading = 360 + heading;
    }
    Serial.print("Compass Facing: ");
    Serial.println(heading);
    return heading;
}

void straightenCompass(){ //determine if car is not straight and turn if necessary
    float compassTolerance = 3.0; //set compass deviation tolerance in degrees
    
    float compassCurrentDirection  = getCompass(); //get current direction
    float diff = compassCurrentDirection - compassStraight; //find the difference between the current direction and "straight"
    //correct for overflowing
    if (diff > 180){
        diff = diff - 360; //for angles > 180, correct in the opposite direction.
    }
    else if (diff < -180){
        diff = diff + 360; //for angles < -180, correct in the opposite direction.
    }
    
    if (diff > compassTolerance){ //too far right, turn left
        left();
    }
    else if (diff < -compassTolerance){ //too far left, turn right
        right();
    }
    else{ //straight enough
        straight();
    }
}

/* Distance Sensor Functions */
long readSensorRaw(byte pwPin){
    return (pulseIn(pwPin, HIGH)/147)*2.54;
}

long readSensorCorrected(byte pwPin, long pulse){
    int pulsetemp = 0;
    pulsetemp = readSensorRaw(pwPin);
    while(pulsetemp<sqrt(pulse)){
        delayMicroseconds(50);
        pulsetemp = readSensorRaw(pwPin);
    }
    return pulsetemp;
}
/* Motor Control Funtions */
void motorGo(uint8_t motor, uint8_t direct, uint8_t pwm)
{
  if (motor <= 1) //Only 2 pins on sheild (Read as "0" and "1"). Makes sure you didn't invent a third one
  {
    if (direct <=4)
    {
      // Set inA[motor]
      if (direct <=1)
        digitalWrite(inApin[motor], HIGH);
      else
        digitalWrite(inApin[motor], LOW);

      // Set inB[motor]
      if ((direct==0)||(direct==2))
        digitalWrite(inBpin[motor], HIGH);
      else
        digitalWrite(inBpin[motor], LOW);

      analogWrite(pwmpin[motor], pwm); // Speed/Strength/Power of motor (? to 1023)
    }
  }
}

void motorOff(int motor) //Choose the motor to turn off, 0 or 1.
{
  // Initialize braked
  for (int i=0; i<2; i++)//This just turns EVERYTHING off.
  {
    digitalWrite(inApin[i], LOW);
    digitalWrite(inBpin[i], LOW);
  }
  analogWrite(pwmpin[motor], 0);
}

int readColorSensor(){ //Reads Color Sensor, Returns a Number for Color Value; See guide at library inclusions
  uint16_t clear, red, green, blue;
  tcs.setInterrupt(false);
  //delay(50);
  tcs.getRawData(&red, &green, &blue, &clear);
  tcs.setInterrupt(true);
  uint32_t sum = clear;
  float r, g, b;
  r = red; r /= sum;
  g = green; g /= sum;
  b = blue; b /= sum;
  r *= 256; g *= 256; b *= 256;
  //Serial.println("RED RED RED RED RED RED"); //Use this code to get raw RGB values
  //Serial.println(r);
  //Serial.println("GREEN GREEN GREEN GREEN");
  //Serial.println(g);
  //Serial.println("BLUE BLUE BLUE BLUE BLUE");
  //Serial.println(b);
  if ((r / g) >=1.00 && (r/g)<=1.6 && (r / b)>=1.3 && (r / b)<=2.3){ //WHITE-TOP
    //Serial.println("WhiteTop/////*****");
    ColorSensorVal = 0;
    return ColorSensorVal;
  }
  if(r<110 && g<90 && b<90){ //(r/g) >=0.80 && (r/b)>=1.0 && (r/g)<=1.0 && (r/b)<=1.3 //DARK TARMAC
    //Serial.println("REALIGN NEEDED===========================");
    ColorSensorVal = 0;
    return ColorSensorVal;
}
  else{ //If its not off the track. May need logic edit.
   if ((r / g) >= 2.3 && (r / b) >= 2.3){ //RED
    //Serial.println("RED/////######");
    ColorSensorVal = 1;
    return ColorSensorVal;
  }
   if ((r / g) <= 0.80 && (g/b) >= 2.00 ){  //(g / b) >= 1.15 //(r / b)<=1.2 //GREEN
    //Serial.println("GREEN/////&&&&&");
    ColorSensorVal = 2;
    return ColorSensorVal;
  }
  if ((r / g) >= 0.9 && (r / g) <=1.7 && (r/b)>=2.5 && (r / b) <= 3.8){ //(r/g)<=2.20 //YELLOW
    //Serial.println("YELLOW/////@@@@@");
    ColorSensorVal = 3;
    return ColorSensorVal;
  }
  }
}
