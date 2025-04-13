#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include <VescUart.h>
#include <ezButton.h>

// init the switches -------> limit switches put them wherever there is space and add pin #s
ezButton driveSwitch(49);
ezButton neutralSwitch(51);
ezButton reverseSwitch(53);

String mode = "none";

// Pin to read Analog input from linear potentiometer
int readPIN = A0;

/** Initiate VescUart class */
VescUart UART;

LiquidCrystal_I2C lcd(0x27, 15,4);

void setup() {

  lcd.init();
  lcd.backlight();
  /** Setup Serial port to display data */
  Serial.begin(9600);

  /** Setup UART port (Serial1 on Atmega32u4) */
  Serial1.begin(115200);
  
  while (!Serial) {;}

  /** Define which ports to use as UART */
  UART.setSerialPort(&Serial1);

  // if switches arent working, uncomment next 3 lines: ----> (Switch troubleshooting)
  // pinMode(driveSwitch, INPUT_PULLUP);
  // pinMode(neutralSwitch, INPUT_PULLUP);
  // pinMode(reverseSwitch, INPUT_PULLUP);

  pinMode(readPIN, INPUT);

  driveSwitch.setDebounceTime(10);
  neutralSwitch.setDebounceTime(10);
  reverseSwitch.setDebounceTime(10);

  delay(1000);
}

void dutyset(float start, float end) {
  const float step = 0.04;
  const float tolerance = 0.03;

  while (abs(start - end) > tolerance) {
    if (end > start) {
      start += step;
      if (start > end) start = end;
    } else if (end < start) {
      start -= step;
      if (start < end) start = end;
    }

    if (mode == "DRIVE") UART.setDuty(start);
    else if (mode == "REVERSE") UART.setDuty(-start);
    else if (mode == "NEUTRAL") UART.setDuty(-start);
    else Serial.println("dutyset() func not working b/c no mode");

    delay(10);
 
  }
}

void loop() {
  driveSwitch.loop();
  neutralSwitch.loop();
  reverseSwitch.loop();

  int revst = reverseSwitch.getState();
  int neust = neutralSwitch.getState();
  int drist = driveSwitch.getState();
  
  /** Read The Input from the petentiometer and map the values from 0 - 1*/
  float pot = analogRead(readPIN);
  pot = (pot/1023) * 1.0;
  Serial.println(pot);

  if (drist == 0) {
    mode = "DRIVE";
    Serial.println("Mode swithched to : DRIVE");
  }
  if (neust == 0) {
    mode = "NEUTRAL";
    Serial.println("Mode swithched to : NUETRAL");
  }
  if (revst == 0) {
    mode = "REVERSE";
    Serial.println("Mode swithched to : REVERSE");
  }

  Serial.println("CURRENT MODE IS:" + mode);


  float rpm = 0.0;
  float volt = 0.0;
  float current = 0.0;
  float power = 0.0;
  float duty = 0.0;

  if (UART.getVescValues() ) {

    rpm = UART.data.rpm;
    volt = UART.data.inpVoltage;
    current = UART.data.avgInputCurrent;
    power = UART.data.inpVoltage * UART.data.avgInputCurrent;
    duty = UART.data.dutyCycleNow;

    Serial.print("DUTY CURRENT: ");
    Serial.println(duty);
    Serial.println("data given  on lcd");

    lcd.setCursor(0,0);
    lcd.print("rpm:" + rpm);
    lcd.setCursor(0,1);
    lcd.print("volt:" + volt);
    lcd.setCursor(0,2);
    lcd.print("current:" + current);
    lcd.setCursor(0,3);
    lcd.print("power:" + power);

    // Serial.print("RPM: ");
    // Serial.print(UART.data.rpm);
    // Serial.print(", ");
    // Serial.print("Voltage:  ");
    // Serial.print(UART.data.inpVoltage);
    // Serial.print(", ");
    // Serial.print("Current:  ");
    // Serial.print(UART.data.avgInputCurrent);
    // Serial.print(", ");
    // Serial.print("Power:    ");
    // Serial.println(UART.data.inpVoltage * UART.data.avgInputCurrent);
  }
  else
  {
    Serial.println("Failed to get data!");
  }

  if (mode == "DRIVE") {
    dutyset(duty, pot);
  } else if (mode == "REVERSE") {
    dutyset(duty, pot);
  } else if (mode == "NEUTRAL") {
    dutyset(duty, 0);
  } else {
    UART.setDuty(0);
    Serial.println("idk what im doing i didnt get a mode");
  }



  /** Call the function getVescValues() to acquire data from VESC */

  delay(500);
}

// hello my name is logan gao
// i like trains
// my name is kay singhal
// i am super nonchalant