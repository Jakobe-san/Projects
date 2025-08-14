#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Servo.h>

/* BNO055

   Connections
   ---------------
   Connect SCL to analog 5
   Connect SDA to analog 4
   Connect VDD to 5V DC
   Connect GROUND to common ground

*/

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28, &Wire);

// defining servo motors
Servo motor1;
Servo motor2;

// PID setup
// time vars
double last_time = 0;
double dt;

// pid vars
double integral, previous, output1 = 0, output2 = 0;
double kp, ki, kd;

int PWM1 = 9;
int PWM2 = 10;

// running pid 
bool running = false;
// running start sequence
bool start = true;


void setup(void)
  {
    Serial.begin(115200);

    while (!Serial) delay(10);  // wait for serial port to open

    Serial.println("Orientation Sensor Raw Data Test"); Serial.println("");

    /* Initialise the sensor */
    if(!bno.begin())
    {
      /* Problem detecting the BNO055 check your connections */
      Serial.print("No BNO055 detected, Check your wiring or I2C ADDR");
      while(1);
    }

    delay(1000);

    /* Display the current temperature */
    int8_t temp = bno.getTemp();
    Serial.print("Current Temperature: ");
    Serial.print(temp);
    Serial.println(" C");
    Serial.println("");

    bno.setExtCrystalUse(true);

    Serial.println("Calibration status values: 0=uncalibrated, 3=fully calibrated");

    // Motor setup
    motor1.attach(PWM1); // attach motor to PWM1 - 9
    motor2.attach(PWM2); // attach motot to PWM2 - 10
    delay(1);
    motor1.writeMicroseconds(1000);
    motor2.writeMicroseconds(1000);
    delay(5000);

    // PID coef
    kp = 2.5;
    ki = 0.02;
    kd = 0.8;

  }

// PID fucntion
double pid(double actual, double setpoint) {
  long error = setpoint - actual;
  double prop = error;
  integral += error * dt;
  double deriv = (error - previous) / dt;
  previous = error;
  double output = (kp*prop) + (ki*integral) + (kd*deriv);
  return output;
}

void loop(void)
  {
    // starting sequence
    while (start) {
      for (int i = 1000; i < 1300; i++) { // 1500 is the throttle value 
        delay(10); // slower start
        motor1.writeMicroseconds(i); // ramping up motors
        motor2.writeMicroseconds(i);
      }
      running = true;
      start = false;
      Serial.print("PID START");
      Serial.println();
    }
    
    while (running) {
      // tracking time
      double now = millis();
      dt = (now - last_time) / 1000.00;
      last_time = now;

      // Possible vector values can be:
      // - VECTOR_ACCELEROMETER - m/s^2
      // - VECTOR_MAGNETOMETER  - uT
      // - VECTOR_GYROSCOPE     - rad/s
      // - VECTOR_EULER         - degrees
      // - VECTOR_LINEARACCEL   - m/s^2
      // - VECTOR_GRAVITY       - m/s^2

      /* Display the floating point data */
      imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

      // Serial.print("Yaw: ");
      // Serial.print(euler.x());
      // Serial.print(" Pitch: ");
      // Serial.print(euler.y());
      // Serial.print(" Roll: ");
      // Serial.print(euler.z());
      // Serial.print("\t\t");

      imu::Vector<3> acceleration = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);

      // Serial.print("Acc X: "); //X
      // Serial.print(acceleration.x());
      // Serial.print(" Acc Y: "); //Y
      // Serial.print(acceleration.y());
      // Serial.print(" Acc Z: "); //Z
      // Serial.print(acceleration.z());
      // Serial.print("\t\t");

      imu::Vector<3> angularVel = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

      // Serial.print("Gyro X: "); //X
      // Serial.print(angularVel.x());
      // Serial.print(" Gyro Y: "); //Y
      // Serial.print(angularVel.y());
      // Serial.print(" Gyro Z: "); //Z
      // Serial.print(angularVel.z());
      // Serial.print("\t\t");

      imu::Vector<3> magField = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);

      // Serial.print("Mag X: "); //X
      // Serial.print(magField.x());
      // Serial.print(" Mag Y: "); //Y
      // Serial.print(magField.y());
      // Serial.print(" Mag Z: "); //Z
      // Serial.print(magField.z());
      // Serial.print("\t\t");
      // Serial.println();

      
      // Quaternion data
      imu::Quaternion quat = bno.getQuat();
      //Serial.print("qW: ");
      //Serial.print(quat.w(), 4);
      //Serial.print(" qX: ");
      //Serial.print(quat.x(), 4);
      //Serial.print(" qY: ");
      //Serial.print(quat.y(), 4);
      //Serial.print(" qZ: ");
      //Serial.print(quat.z(), 4);
      //Serial.print("\t\t");
      

      /* Display calibration status for each sensor. */
      uint8_t system, gyro, accel, mag = 0;
      bno.getCalibration(&system, &gyro, &accel, &mag);
      //Serial.print("CALIBRATION: Sys=");
      //Serial.print(system, DEC);
      //Serial.print(" Gyro=");
      //Serial.print(gyro, DEC);
      //Serial.print(" Accel=");
      //Serial.print(accel, DEC);
      //Serial.print(" Mag=");
      //Serial.println(mag, DEC);

      // Motor control test
      double rollAngle = euler.z(); // get roll angle
      double throttle = 1300;
      double PIDvalue = pid(rollAngle, 0); // input PID rollangle setpoint 0 degrees
      output1 = throttle + PIDvalue; 
      output2 = throttle - PIDvalue;
      //checking range for outoputs
      if (output1 < 1000) {
        output1 = 1000;
      } 
      else if (output1 > 2000) {
        output1 = 2000;
      }
      if (output2 < 1000) {
        output2 = 1000;
      } 
      else if (output2 > 2000) {
        output2 = 2000;
      }
      motor1.writeMicroseconds(output1); // writing correction to motor
      motor2.writeMicroseconds(output2);

      Serial.print(output1);
      Serial.print("      ");
      Serial.print(euler.z());
      Serial.print("      ");
      Serial.print(output2);
      Serial.println();

      // stop motor condition
      if ( abs(euler.z()) > 65.0 ) {
        Serial.print(abs(euler.z()));
        Serial.print("PROGRAM STOPPED");
        motor1.writeMicroseconds(1000); // writing motors to off
        motor2.writeMicroseconds(1000);
        running = false;
      }

      delay(BNO055_SAMPLERATE_DELAY_MS);
    }
    // check for okay and make running true again to restart
  }























