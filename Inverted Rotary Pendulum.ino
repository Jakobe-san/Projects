// Welcome to the inverted pendulum automatic balancer with a pid controller by jacob dill
// featuring:
// nema 17 stepper motor 17HS19-2004S1
// rotary encoder Taiss E38S6-600-24G
// L298N motor driver
// and the Arduino Uno
// Evan Music's 3d printer

#include <Stepper.h>

// the pins we've connected our stepper driver to
const int stepIN1Pin = 8;
const int stepIN2Pin = 9;
const int stepIN3Pin = 10;
const int stepIN4Pin = 11;

// steps required for one full revolution
const int stepsPerRevolution = 200;

// initialize stepper library Pins 8-11
Stepper myStepper(stepsPerRevolution,
                  stepIN1Pin, stepIN3Pin,
                  stepIN2Pin, stepIN4Pin);

// the pins the encoder is connnected to
const int encoderPin1 = 2;
const int encoderPin2 = 3;

// initial var for encoder
int currentState;
int temp = 0;
int counter = 0;

// time vars
unsigned long last_time = 0;
unsigned long dt;

// pid vars
double integral, previous, output = 0;
double kp, ki, kd;

// setpoint 180 off from hanging staring positon
double setpoint = 180;
double angle;

// is the program running
bool running = false;

void setup()
{
  // set the speed of the motor RPMs
  myStepper.setSpeed(300);
  // serial port
  Serial.begin(9600);

  // pid coef
  kp = 0.9;
  ki = 0.01;
  kd = 0.001;

  // setting encoder pins
  pinMode(encoderPin1, INPUT_PULLUP);
  pinMode(encoderPin2, INPUT_PULLUP);
  // check interrupt
  attachInterrupt(0, ai0, RISING);
  attachInterrupt(1, ai1, RISING);

}

void loop()
{
  // tracking time
  unsigned long now = millis();
  dt = (now - last_time)/1000.00;
  last_time = now;

  // counter for encoder positon angle
  if (counter != temp) { 
    double counterDouble = (double) counter;
    angle = counterDouble * (360.00 / 1200.00);
    //Serial.println(angle);
    temp = counter;      
  }
  
  // start once reaches top
  if (running != true && angle == setpoint) {
    running = true;
  }

  if (running) {
    // if too far, give up
    if (angle < 110 || angle > 250) {
      running = false;
      Serial.println("OUT OF BOUNDS");
    }

    // PID error and output calc
    double actual = angle;
    double error = setpoint - actual;
    output = pid(error);

    // writing output from pid
    myStepper.step(output);

    // plot graph
    Serial.print(setpoint);
    Serial.print(",");
    Serial.println(actual);
  } 
}

// pid
double pid(double error) {
  double prop = error;
  integral += error * dt;
  double deriv = (error - previous) / dt;
  previous = error;
  double output = (kp*prop) + (ki*integral) + (kd*deriv);
  return output;
}

// attach interrupt functions for counter
void ai0() {
  if (digitalRead(3) == LOW) {
    // reset angle is past 360 degrees
    if (abs(counter) >= 1200) {
      counter = 0;
      temp = 0;
    }
    counter++;
  } else {
    // dont be negative
    if (counter == 0 || counter < 0) {
      counter = 1200;
    }
    counter--;
  }
}

void ai1() {
  if (digitalRead(2) == LOW) {
    // dont be negative
    if (counter == 0 || counter < 0) {
      counter = 1200;
    }
    counter--;
  } else {
    // reset angle is past 360 degrees
    if (abs(counter) >= 1200) {
      counter = 0;
      temp = 0;
    }
    counter++;
  }
}