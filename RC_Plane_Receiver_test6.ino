#define trigPin 10 //distance trigger pin on Digital pin 10
#define echoPin 13 //distance echo pin on Digital pin 13
#include<Servo.h>

int throttleInput = A2; //set analog pin 2 to receive input from receiver
int throttleOut = 5; //set digital pin 5 to send pwm to throttle
int elevIn = 7;
int elevOut = 9;
  int throttle;
  int elevpwm;
Servo esc;
Servo elev;

void setup() {
  // put your setup code here, to run once:
  esc.attach(throttleOut); //set pin for esc
  elev.attach(elevOut); //set pin for elevator
  pinMode(trigPin, OUTPUT); //set up trigger pin for distance sensor
  pinMode(echoPin, INPUT); //set up echo pin for distance sensor
  esc.writeMicroseconds(1000);
  elev.write(90); //set elevator servo to neutral position (in degrees)
  Serial.begin(115200);
}

void loop() {


  
  throttle = pulseIn(throttleInput, HIGH); //set timeout to 3000 microseconds
  elevpwm = pulseIn(elevIn, HIGH, 3000);
  Serial.println(elevpwm);
  throttle = constrain(throttle, 1000, 2000);
  elevpwm = constrain(elevpwm, 1000, 2000);
  Serial.println(elevpwm);
  esc.writeMicroseconds(throttle);
  elev.writeMicroseconds(elevpwm);
}
