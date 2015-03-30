#include <Servo.h> 

Servo turnServo;  //create servo object to control a servo 

// Turning 
byte turnServoPin = 9; //pin for servo motor
byte turnServoDirection = 1; //0=left; 1=center; 2=right
//byte turnServoLastDirection = 1;

// Drive Motor
byte driveMotorDirectionPin = 4;
byte driveMotorSpeedPin = 5;
//unsigned long driveMotorUpdateDuration; //will store last time motor was updated
unsigned long driveMotorPreviousMillis; //will store last time motor was updated
int driveMotorDelay;
boolean driveMotorDirection = false; //false=forward; true=backward
//boolean driveMotorLastDirection = false;

// Distance Sensors
int FLDistSensorVal;          //value from the distance Front Left sensor
int FLDistSensorPin = 0;   //Front Left Sensor analog pin
boolean FLDetect = false;
int FCDistSensorVal;         //value from the distance Front Right sensor
int FCDistSensorPin = 1;  //Front Center Sensor analog pin
boolean FCDetect = false;
int FRDistSensorVal;         //value from the distance Front Right sensor
int FRDistSensorPin = 2;  //Front Right Sensor analog pin
boolean FRDetect = false;
int distThreshold = 450;    //threshold for the distance sensor

boolean delayOverride = false;

void setup(){
    //set pins
    pinMode(10, OUTPUT);
    pinMode(11, OUTPUT);
    pinMode(12, OUTPUT);
    pinMode(driveMotorDirectionPin, OUTPUT);
    turnServo.attach(turnServoPin);  //attaches the servo to the servo object

    randomSeed(analogRead(5)); //initialize random seed
    Serial.begin(9600);
}

void loop(){
    FLDistSensorVal = analogRead(FLDistSensorPin);
    FCDistSensorVal = analogRead(FCDistSensorPin);
    FRDistSensorVal = analogRead(FRDistSensorPin);
    Serial.print('a');
    Serial.println(FLDistSensorVal);
    Serial.print('b');
    Serial.println(FCDistSensorVal);
    Serial.print('c');
    Serial.println(FRDistSensorVal);
    Serial.println("--");
    
    if(FLDistSensorVal > distThreshold){FLDetect=true; digitalWrite(10,HIGH);}
    if(FCDistSensorVal > distThreshold){FCDetect=true; digitalWrite(11,HIGH);}
    if(FRDistSensorVal > distThreshold){FRDetect=true; digitalWrite(12,HIGH);}
    
    if (FLDetect==true && FCDetect==true && FRDetect==true){ 
        delayOverride = true;
    }
    else if(FLDetect==true && turnServoDirection != 0){ //left
        delayOverride = true;
    }
    else if(FCDetect==true && turnServoDirection != 1){ //center
        delayOverride = true;
    }
    else if(FRDetect==true && turnServoDirection != 2){ //right
        delayOverride = true;
    }
    
    if((millis() - driveMotorPreviousMillis) > driveMotorDelay || delayOverride == true){
        delayOverride = false;
        driveMotorPreviousMillis = millis();
        
        if(FLDetect==false && FCDetect==false && FRDetect==false){ //forward
            forward(150);
            straight();
            driveMotorDelay = 0;
        }
        else if (FLDetect==true && FCDetect==true && FRDetect==true){ //backward
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
        else if(FCDetect==true){  //center
            backward(150);
            //delay(100);
            //if (turnServo.read() == 66){
            //    left();
            //}
            //else if (turnServo.read() == 89){
            //    right();
            //}
            driveMotorDelay = 2000;
        }
    }
    
    FLDetect=false;
    FCDetect=false;
    FRDetect=false;
    digitalWrite(10,LOW);
    digitalWrite(11,LOW);
    digitalWrite(12,LOW);
}

void forward(int velocity){
    digitalWrite(driveMotorDirectionPin, LOW); //LOW is forward
    analogWrite(driveMotorSpeedPin, velocity);
    driveMotorDirection = false;
    //driveMotorLastDirection = false;
    //Serial.println("forward");
}

void backward(int velocity){
    delay(50);
    digitalWrite(driveMotorDirectionPin, HIGH); //HIGH is backward
    analogWrite(driveMotorSpeedPin, velocity);
    driveMotorDirection = true;
    //driveMotorLastDirection = true;
    //Serial.println("backward");
}

void straight(){
    //get position of turning servo and turn until center
    //if (turnServo.read() >= 80){
        turnServo.write(79);  //turn servo to x degrees
    //}
    //else if (turnServo.read() <= 79){
    //    turnServo.write(80);  //turn servo to x degrees
    //}
    turnServoDirection = 1;
    //turnServoLastDirection = 1
    //Serial.println("straight");
}

void left(){
    turnServo.write(66); //turn servo to x degrees
    //delay(duration)
    turnServoDirection = 0;
    //turnServoLastDirection = 0;
    //Serial.println("left");
}

void right(){
    turnServo.write(89); //turn servo to x degrees
    //delay(duration);
    turnServoDirection  = 2;
    //turnServoLastDirection = 2;
    //Serial.println("right");
}

