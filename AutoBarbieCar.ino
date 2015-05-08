/* Servo */
#include <Servo.h> 
Servo turnServo;  //create servo object to control a servo 

/* Compass */
#include <Average.h> //import Average library for compass initialization
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(12345); //compass object
const float Pi = 3.14159; //apparently the pi constant isn't built into Arduino
float compassStraight = 0.0; //this will hold what we decide as "straight"

/* Turning */
const byte turnServoPin = 9; //pin for servo motor
byte turnServoDirection = 1; //0=left; 1=center; 2=right
//byte turnServoLastDirection = 1;

/* Drive Motor */
const byte driveMotorDirectionPin = 4; //pin for drive motor direction control
const byte driveMotorSpeedPin = 5; //pin for drive motor speed control
//unsigned long driveMotorUpdateDuration; //will store last time motor was updated
unsigned long driveMotorPreviousMillis; //will store last time motor was updated
int driveMotorDelay; //how long to wait before updating the motor during manuver
boolean driveMotorDirection = false; //false=forward; true=backward
//boolean driveMotorLastDirection = false;

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

void setup(){
    randomSeed(analogRead(3)); //initialize random seed
    Serial.begin(9600); //start Serial
  
    //set pins
    pinMode(FLDistSensorPin, INPUT);
    pinMode(FRDistSensorPin, INPUT);
    pinMode(10, OUTPUT); //left LED (red)
    pinMode(11, OUTPUT); //center LED (yellow)
    pinMode(12, OUTPUT); //right LED (green)
    pinMode(driveMotorDirectionPin, OUTPUT); 
    turnServo.attach(turnServoPin);  //attaches the servo to the servo object
    if(!mag.begin()){ //initilize compass
        //There was a problem detecting the LSM303 ... check your connections
        Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
        while(1); //stall forever
    }
    else{
        initCompass(); //initilize the compass; set the direction we will use as "straight"
    }
    FLDistSensorVal = readSensorRaw(FLDistSensorPin);
    //FRDistSensorVal = readSensorRaw(FRDistSensorPin);
    FRDistSensorVal = 1000;
}

void loop(){
    delay(5);
    //read distance sensors
    //FLDistSensorVal = (FLDistSensorAverage.mean()/147)* 2.54; //average the samples and use as "straight"
    /*
    int FLtemp = 0;
    while(FLtemp<FLDistSensorValLast-20){
        FLtemp = (pulseIn(FLDistSensorPin, HIGH)/147)* 2.54;
    }
    FLDistSensorVal = FLtemp; //get value and convert uS to inches to cm
    */
    //FLDistSensorVal = readSensorRaw(FLDistSensorPin);
    FLDistSensorVal = readSensorCorrected(FLDistSensorPin, FLDistSensorVal);
    //FRDistSensorVal = (pulseIn(FRDistSensorPin, HIGH)/147)* 2.54; //get value and convert uS to inches to cm
    //print distence 
    Serial.print('a');
    Serial.println(FLDistSensorVal);
    Serial.print('b');
    Serial.println(FRDistSensorVal);
    Serial.println("--");
    
    //check for distance sensor detection
    if(FLDistSensorVal < distThreshold){FLDetect=true; digitalWrite(10,HIGH);}
    //if(FRDistSensorVal < distThreshold){FRDetect=true; digitalWrite(12,HIGH);}
    
    //check if we need to override a manuver
    if (FLDetect==true && FRDetect==true){ 
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
            backward(150);
            left();
            driveMotorDelay = 2000;
        }
        else if(FRDetect==true){  //right
            backward(150);
            right();
            driveMotorDelay = 2000;
        }
    }
    
    //reset distance sensor detect flags
    FLDetect=false;
    FRDetect=false;
    //reset distance sensor detect LEDs
    digitalWrite(10,LOW);
    digitalWrite(11,LOW);
    digitalWrite(12,LOW);
}

void forward(int velocity){ //drive forward
    digitalWrite(driveMotorDirectionPin, LOW); //LOW is forward
    analogWrite(driveMotorSpeedPin, velocity);
    driveMotorDirection = false;
    //driveMotorLastDirection = false;
    //Serial.println("forward");
}

void backward(int velocity){ //drive backward
    delay(50);
    digitalWrite(driveMotorDirectionPin, HIGH); //HIGH is backward
    analogWrite(driveMotorSpeedPin, velocity);
    driveMotorDirection = true;
    //driveMotorLastDirection = true;
    //Serial.println("backward");
}

void straight(){ //turn wheels straight
    turnServo.write(79);  //turn servo to x degrees
    turnServoDirection = 1;
    //turnServoLastDirection = 1
    //Serial.println("straight");
}

void left(){ //turn wheels left
    turnServo.write(66); //turn servo to x degrees
    turnServoDirection = 0;
    //turnServoLastDirection = 0;
    //Serial.println("left");
}

void right(){ //turn wheels right
    turnServo.write(89); //turn servo to x degrees
    turnServoDirection  = 2;
    //turnServoLastDirection = 2;
    //Serial.println("right");
}

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
