/**
   WALL-E CONTROLLER CODE

   @file       wall-e.ino
   @brief      Main Wall-E Controller Sketch
   @author     Simon Bluett
   @email      hello@chillibasket.com
   @copyright  Copyright (C) 2021 - Distributed under MIT license
   @version    2.9
   @date       29th May 2021

   HOW TO USE:
   1. Install the Adafruit_PWMServoDriver library
      a. In the Arduino IDE, go to Sketch->Include Library->Manage Libraries
      b. Search for Adafruit PWM Library, and install the latest version
   2. Calibrate the servo motors, using the calibration sketch provided in the
      GitHub repository. Paste the calibrated values between lines 144 to 150.
   3. Upload the sketch to the micro-controller, and open the serial monitor
      at a baud rate of 115200.
   4. Additional instructions and hints can be found at:
      https://wired.chillibasket.com/3d-printed-wall-e/
*/

#include "Arduino.h"
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <SoftwareSerial.h>
#include <ArduinoBlue.h>
#include "MotorController.hpp"
#include "DFRobotDFPlayerMini.h"

#define ANALOG_IN_PIN A0
#define MAXSONAR_PIN A2
#define PWM_SPEED_L_PIN  3           // Motor PWM pins
#define MP3_TX_PIN 4                // This means the pins as labeled on the board: https://wiki.dfrobot.com/DFPlayer_Mini_SKU_DFR0299#target_3 
#define MP3_RX_PIN 5
#define BLUETOOTH_TX 6          // Pins 9 & 13 didn't work for some reason.
#define BLUETOOTH_RX 7
#define BRAKE_R_PIN  8
#define BRAKE_L_PIN  9               // Motor brake pins
#define SERVO_ENABLE_PIN 10          // Servo shield output enable pin
#define PWM_SPEED_R_PIN 11
#define DIRECTION_L_PIN 12           // Motor direction pins
#define DIRECTION_R_PIN 13

#define NUMBER_OF_SERVOS 7

int LEFTSPEED = 200;            // These are for calibration - try to make him go straight
int RIGHTSPEED = 200;
int prevThrottle = 49;
int prevSteering = 49;
int throttle, steering;

SoftwareSerial bluetooth(BLUETOOTH_TX, BLUETOOTH_RX);  // NOTE THIS MEANS WHAT IS SAYS ON THE BT MODULE - THE API TAKES THE OPPOSITE - RX, TX.
ArduinoBlue phone(bluetooth); // pass reference of bluetooth object to ArduinoBlue constructor
SoftwareSerial mp3serial(MP3_TX_PIN, MP3_RX_PIN); // RX, TX
DFRobotDFPlayerMini myDFPlayer;

// Servo shield controller class - assumes default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Set up motor controller classes
MotorController motorL(DIRECTION_L_PIN, PWM_SPEED_L_PIN, BRAKE_L_PIN, false);
MotorController motorR(DIRECTION_R_PIN, PWM_SPEED_R_PIN, BRAKE_R_PIN, false);


// Voltage Detection:

// Floats for ADC voltage & Input voltage
float adc_voltage = 0.0;
float in_voltage = 0.0;

// Floats for resistor values in divider (in ohms)
float R1 = 30000.0;
float R2 = 7500.0;

// Float for Reference Voltage
float ref_voltage = 5.0;

// Integer for ADC value
int adc_value = 0;

// -------------------------------------------------------------------
/// Initial setup
// -------------------------------------------------------------------

void setup() {

  // Output Enable (EO) pin for the servo motors
  pinMode(SERVO_ENABLE_PIN, OUTPUT);
  digitalWrite(SERVO_ENABLE_PIN, HIGH);

  // Communicate with servo shield (Analog servos run at ~60Hz)
  pwm.begin();
  pwm.setPWMFreq(60);

  // Turn off servo outputs
  for (int i = 0; i < NUMBER_OF_SERVOS; i++) {
    pwm.setPin(i, 0);
  }

  // Initialize serial communication for debugging
  Serial.begin(115200);
  bluetooth.begin(9600);    // APPARENTLY THIS HAS TO BE 9600 - DOESN'T WORK AT 115200
  // mp3serial.begin(9600);
  // myDFPlayer.begin(mp3serial);
  bluetooth.listen();

  randomSeed(analogRead(0));
  
  digitalWrite(SERVO_ENABLE_PIN, LOW);

  Serial.println(F("Sartup complete; entering main loop"));
}


void loop() {

  // getVoltage();

  // getButtons();
  getSliders();
  getMovement();
}


void getVoltage() {
  // Read the Analog Input
  adc_value = analogRead(ANALOG_IN_PIN);

  // Determine voltage at ADC input
  adc_voltage  = (adc_value * ref_voltage) / 1024.0;

  // Calculate voltage at divider input
  in_voltage = adc_voltage / (R2 / (R1 + R2));

  // Print results to Serial Monitor to 2 decimal places
  Serial.print("Input Voltage = ");
  Serial.println(in_voltage, 2);
}

void getButtons() {
  int i = phone.getButton();
  if (i != -1) {
    Serial.print("Got a button: ");
    Serial.println(i);
    mp3serial.listen();
    switch (i) {
      case 3:
        myDFPlayer.play(1);
        delay(4000);
        break;
      case 4:
        myDFPlayer.play(5);
        delay(2000);
        break;
    }
    bluetooth.listen();
  }
}

void getSliders() {
  // ID of the slider moved.
  int  sliderId = phone.getSliderId();
  // Slider value goes from 0 to 200.
  if (sliderId != -1) {
    int mapped = 0;
    int  sliderVal = phone.getSliderVal();
    // Serial.print(sliderId); Serial.print(": "); Serial.print(sliderVal);
    switch (sliderId) {
      case 0:
        // Right Eye
        mapped = map(sliderVal, 0, 200, 300, 10);  // TODO: NOT SURE WHAT THE MAX VALUE HERE SHOULD BE THO 600 APPEARS TO MOVE THEM 180
        break;
      case 1:
        // Left Eye
        mapped = map(sliderVal, 0, 200, 10, 300);
        break;
    }
    pwm.setPWM(sliderId, 0, mapped);
  }
}


void getMovement() {
  // Throttle and steering values go from 0 to 99.
  // When throttle and steering values are at 99/2 = 49, the joystick is at center.
  throttle = phone.getThrottle();
  steering = phone.getSteering();

  // Display throttle and steering data if steering or throttle value is changed
  if (prevThrottle != throttle || prevSteering != steering) {
    Serial.print("Throttle: "); Serial.print(throttle); Serial.print("\tSteering: "); Serial.println(steering);
    prevThrottle = throttle;
    prevSteering = steering;
    if (throttle == 49 && steering == 49) {
      Serial.println("STOPPING");
      motorL.setSpeed(0);
      motorR.setSpeed(0);
      delay(500);
    }
    else {
      if (throttle > 49) {
        LEFTSPEED = abs(LEFTSPEED);
        RIGHTSPEED = abs(RIGHTSPEED);
      }
      else {
        LEFTSPEED = abs(LEFTSPEED) * -1;
        RIGHTSPEED = abs(RIGHTSPEED) * -1;
      }
      int speedAdj = map(steering, 0, 99, -40, 40);
      int lSpeed = LEFTSPEED + speedAdj;
      int rSpeed = RIGHTSPEED - speedAdj;
      Serial.print("L SPEED: "); Serial.println(lSpeed);
      Serial.print("R SPEED: "); Serial.println(rSpeed);
      motorL.setSpeed(LEFTSPEED + speedAdj);
      motorR.setSpeed(RIGHTSPEED - speedAdj);
    }
  }
}
