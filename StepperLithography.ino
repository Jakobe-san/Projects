#include <Stepper.h>

const int stepsPerRevolution = 200;  // number of steps per revolution
const int constSpeed = 20; //rpm
const int buttonONOFF = 50; // button pin 
const int laser = 51; // laser on off pin

// array size (max) 
// define max size at start fill text file to matrix and only loop text file size
// max size is 3mm shift from center (6x6mm square) 
const int x = 20; // steps (pizels/resolusion)
const int y = 20; // steps 

// 1 step / 2.5 micrometers (0.0025mm)

// input matrix (binary)
int inputMtx[x][y] = {{1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0},
                      {0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0},
                      {0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0},
                      {0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1},
                      {1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0},
                      {0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0},
                      {0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0},
                      {0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1},
                      {1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0},
                      {0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0},
                      {0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0},
                      {0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1},
                      {1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0},
                      {0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0},
                      {0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0},
                      {0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1},
                      {1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0},
                      {0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0},
                      {0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0},
                      {0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1}};


// Define digit and segment control pins
byte digitPins[] = {42, 45, 46, 31};
byte segmentPins[] = {43, 47, 29, 27, 26, 44, 30, 28};

// Digit segment mappings for 0-9
const byte digits[10][7] = {
    {1, 1, 1, 1, 1, 1, 0}, {0, 1, 1, 0, 0, 0, 0}, {1, 1, 0, 1, 1, 0, 1}, {1, 1, 1, 1, 0, 0, 1},
    {0, 1, 1, 0, 0, 1, 1}, {1, 0, 1, 1, 0, 1, 1}, {1, 0, 1, 1, 1, 1, 1}, {1, 1, 1, 0, 0, 0, 0},
    {1, 1, 1, 1, 1, 1, 1}, {1, 1, 1, 1, 0, 1, 1}};

// initialize the stepper library on pins 8-11 (Stepper X) and 4-7 (Stepper Y):
Stepper stepperX(stepsPerRevolution, 8, 9, 10, 11);
Stepper stepperY(stepsPerRevolution, 4, 5, 6, 7);

void setup() {
  // set the speed at const rpm:
  stepperX.setSpeed(constSpeed);
  stepperY.setSpeed(constSpeed);
  
  // button initalizing
  pinMode(buttonONOFF, INPUT_PULLUP);
  pinMode(laser, OUTPUT);  

  for (byte i = 0; i < 4; i++) { pinMode(digitPins[i], OUTPUT); digitalWrite(digitPins[i], HIGH); }
  for (byte i = 0; i < 7; i++) pinMode(segmentPins[i], OUTPUT); 
  pinMode(segmentPins[7], OUTPUT); // DP pin
  
  // initialize the serial port:
  Serial.begin(9600);
}

// fucntion to display digit
void displayDigit(int digit, int pos) {
    if (digit < 0 || digit > 9) return;
    for (byte i = 0; i < 7; i++) digitalWrite(segmentPins[i], digits[digit][i]);
    digitalWrite(segmentPins[7], LOW); // Decimal point off
    for (byte i = 0; i < 4; i++) digitalWrite(digitPins[i], (i == pos) ? LOW : HIGH);
    delay(5); // Display persistence
    for (byte i = 0; i < 4; i++) digitalWrite(digitPins[i], HIGH);
}

void displayNumber(int number) {
    int digitsToShow[] = {-1, -1, -1, -1};
    if (number == 100) { digitsToShow[0] = 1; digitsToShow[1] = 0; digitsToShow[2] = 0; }
    else if (number >= 10) { digitsToShow[1] = number / 10; digitsToShow[2] = number % 10; }
    else digitsToShow[2] = number;
    for (byte i = 0; i < 4; i++) if (digitsToShow[i] != -1) displayDigit(digitsToShow[i], i);
}

void loop() {
  // Continue program 
  while (digitalRead(buttonONOFF) == LOW) {
    // Program on/off
    if (digitalRead(buttonONOFF) == HIGH) {
      break;
    }

    // turn off laser unit in postion loop
    digitalWrite(laser, LOW);
  
    // zeroing 

    // move to top left (3mm x and y)
    stepperX.step(1200); // 3mm
    stepperY.step(-1200);  // 3mm
    
    // Postioning 
    for (int i = 0; i < x; i++) {
      for (int j = 0; j < y; j++) {
        // move to postion
        stepperX.step(1);
        delay(200);
        if (inputMtx[i][j] == 1) {
          // if pixel, turn on laser
          digitalWrite(laser, HIGH);
          Serial.print(1);  // for picture from rasberrypi
          delay(5000);  // time to take picture
          digitalWrite(laser, LOW); // turn laser off after picture
        }
        // step down and move back to left side of matrix
      stepperY.step(-1);
      delay(200);
      stepperX.step(-y);
      delay(200);

      }
    }  
    
    // delay for setppers
    delay(500); //ms
  }
}

//----------------NOTES and IMPROVMENTS--------------------------------------------
// No fucntion for zeroing or centering the plate. that was done manually.
// The 7 screen was not fully implemented to run while the stepper ran.
// The button onyl starts the program and doesnt stop it.
// The input matrix for teh picture is manully prgrammed to the Arduino.
// ---------------------------------------------------------------------------------
