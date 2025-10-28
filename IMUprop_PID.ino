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

// Set the delay between fresh samples
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
double integral, previous, output = 0;
double kp, ki, kd;

// setpoint angle
double setpoint = 0;

// setting pins for PWM
int PinMotor1 = 9;
int PinMotor2 = 10;

// min and max micrsec values for ESC's (changes for each esc)
int microsecMin = 1000;
int microsecMax = 2400;

// is the program running
bool running = false;

void setup(void)
  {
    Serial.begin(115200);

    while (!Serial) delay(10);  // wait for serial port to open

    Serial.println("Orientation Sensor Raw Data Test"); Serial.println("");

    // Initialise the sensor
    if(!bno.begin())
    {
      // Problem detecting the BNO055
      Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
      while(1);
    }

    delay(1000);

    // Display the current temperature
    int8_t temp = bno.getTemp();
    Serial.print("Current Temperature: ");
    Serial.print(temp);
    Serial.println(" C");
    Serial.println("");

    bno.setExtCrystalUse(true);

    Serial.println("Calibration status values: 0=uncalibrated, 3=fully calibrated");

    // Motor arming setup
    motor1.attach(PinMotor1); //PWM pin 9
    motor2.attach(PinMotor2); //PWM pin 10
    delay(10);
    motor1.writeMicroseconds(microsecMin); // arm esc
    motor2.writeMicroseconds(microsecMin); // arm esc
    delay(4000);

    // PID coef
    kp = 0.6;
    ki = 0.7;
    kd = 0.30;

  }


void loop(void)
  {
    // tracking time
    double now = micros();
    dt = (now - last_time) / 1000000.0;
    // correcting incase zero divison in pid()
    if (dt <= 0.00001) {
      dt = 0.00001;
    }
    last_time = now;

    // Possible vector values can be:
    // - VECTOR_ACCELEROMETER - m/s^2
    // - VECTOR_MAGNETOMETER  - uT
    // - VECTOR_GYROSCOPE     - rad/s
    // - VECTOR_EULER         - degrees
    // - VECTOR_LINEARACCEL   - m/s^2
    // - VECTOR_GRAVITY       - m/s^2

    // Display the floating point data
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
    

    // Display calibration status for each sensor
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

    // run program when angle is at 0 and not running already
    if (running != true && abs(euler.y() - setpoint) < 1.0) {
      running = true;
    }

    // run program when 
    if (running) {
      // Motor control test
      double rollAngle = euler.y(); // get roll angle
      output = pid(rollAngle, setpoint); // input PID rollangle setpoint 0 degrees
      //analogWrite(PWM1, output); // writing correction to motor 1
      //analogWrite(PWM2, -output); // writing correction to motor 2

      // Ensures outputs are in resonable range
      output = constrain(output, -400, 400);
      
      // base motor speed without control input
      int baseThrottle = 1200;

      // setting motor PWM signal
      int PWMmotor1 = baseThrottle + (int)output;
      int PWMmotor2 = baseThrottle - (int)output;

      // Ensure PWM stays within valid range
      PWMmotor1 = constrain(baseThrottle + (int)output, microsecMin, microsecMax);
      PWMmotor2 = constrain(baseThrottle - (int)output, microsecMin, microsecMax);

      // writing to motors
      motor1.writeMicroseconds(PWMmotor1);
      motor2.writeMicroseconds(PWMmotor2);

      // outputing data in serial monitor
      Serial.print(output);
      Serial.print("\t\t");
      Serial.print(rollAngle);
      Serial.print("\t");
      Serial.print(setpoint);
      Serial.println();
    }

    // STOP
    if (Serial.available()) {
      if (Serial.read() == 'q') {
        // turn motors off and stop program
        motor1.writeMicroseconds(microsecMin);
        motor2.writeMicroseconds(microsecMin);
        integral = 0;
        running = false;
      }
    }

    delay(BNO055_SAMPLERATE_DELAY_MS);
  }


  // PID fucntion
  double pid(double actual, double setpoint) {
    long error = setpoint - actual;
    double prop = error;
    integral += error * dt;
    integral = constrain(integral, -1000, 1000); //prevent windup of int term
    double deriv = (error - previous) / dt;
    previous = error;
    double output = (kp*prop) + (ki*integral) + (kd*deriv);
    return output;
  }
  
