#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include <VescUart.h>
#include <BuckPSU.h> // to install: put the github link in deps and extracted file in lib
#include <ezButton.h>
// #include <SoftwareWire.h>
// #include <hd44780.h>
// #include <hd44780ioClass/hd44780_I2Cexp.h>  // For I2C (PCF8574)


// init the switches -------> limit switches put them wherever there is space and add pin #s
ezButton driveSwitch(49);
ezButton neutralSwitch(53);
ezButton reverseSwitch(51);

String mode = "none";

#define readPIN A0       // pot pin
#define SignalBattery 26 // signal from buck to activate battery relay
#define SignalSolar 28   // signal from buck to activate solar relay

#define relayBattery 24 // output pin to control battery relay
#define relaySolar 25   // output pin to control solar relay

VescUart UART;
BuckPSU psu(Serial2);

// SoftwareWire softI2C(3, 2); //sda scl

LiquidCrystal_I2C lcd(0x27, 20, 4);
// hd44780_I2Cexp lcd2(0x27, &softI2C);

void setup()
{

  lcd.init();
  lcd.backlight();

  Serial.begin(9600);

  /** Setup UART port (Serial1 on Atmega32u4) */
  Serial1.begin(115200);
  Serial2.begin(4800);

  while (!Serial)
  {
    ;
  }

  /** Define which ports to use as UART */
  UART.setSerialPort(&Serial1);


  // if switches arent working, uncomment next 3 lines: ----> (Switch troubleshooting)
  // pinMode(driveSwitch, INPUT_PULLUP);
  // pinMode(neutralSwitch, INPUT_PULLUP);
  // pinMode(reverseSwitch, INPUT_PULLUP);

  pinMode(readPIN, INPUT);

  pinMode(SignalBattery, INPUT_PULLUP);
  pinMode(SignalSolar, INPUT_PULLUP);

  pinMode(relayBattery, OUTPUT);
  pinMode(relaySolar, OUTPUT);

  // rekays start in off mode
  digitalWrite(relayBattery, LOW);
  digitalWrite(relaySolar, LOW);

  driveSwitch.setDebounceTime(10);
  neutralSwitch.setDebounceTime(10);
  reverseSwitch.setDebounceTime(10);

  lcd.clear();

  delay(1000);
}

void dutyset(float start, float end)
{
  const float step = 0.005;
  const float tolerance = 0.03;

  start = abs(start);
  while (abs(start - end) > tolerance)
  {
    if (end > start)
    {
      start += step;
      if (start > end)
        start = end;
    }
    else if (end < start)
    {
      start -= step;
      if (start < end)
        start = end;
    }

    if (mode == "DRIVE")
      UART.setDuty(start);
    else if (mode == "REVERSE")
      UART.setDuty(-start);
    else if (mode == "NEUTRAL")
      UART.setDuty(-start);
    else
    {}
      // Serial.println("dutyset() func not working b/c no mode");

    // delay(10);
  }
}

void buckset(float start, float end)
{
  const float step = 1000;
  const float tolerance = 500;

  while (abs(start - end) > tolerance)
  {
    if (end > start)
    {
      start += step;
      if (start > end)
        start = end;
    }
    else if (end < start)
    {
      start -= step;
      if (start < end)
        start = end;
    }

    psu.setVoltageMilliVolts(start);

    delay(10);
  }
}


void loop()
{ // --------------------------------- LOOP ---------------------------------

  // int batonoff = digitalRead(SignalBattery);
  int batonoff = 0;
  int solonoff = digitalRead(SignalSolar);

  if (batonoff == 0)
  {
    digitalWrite(relayBattery, HIGH);
    // Serial.println("   BATTERY ON");

  }
  else
  {
    digitalWrite(relayBattery, LOW);
    // Serial.println("BATTERY OFF");
  }
  if (solonoff == 0)
  {
    digitalWrite(relaySolar, HIGH);
    // Serial.println("SOLAR ON");
    
    buckset(0, 28000);
    psu.enableOutput(true);
  }
  else
  {
    digitalWrite(relaySolar, LOW);
    // Serial.println("SOLAR OFF");

    
    buckset(28000, 0);
    psu.enableOutput(false);
  }

  driveSwitch.loop();
  neutralSwitch.loop();
  reverseSwitch.loop();

  int revst = reverseSwitch.getState();
  int neust = neutralSwitch.getState();
  int drist = driveSwitch.getState();

  /** Read The Input from the petentiometer and map the values from 0 - 1*/
  float pot = analogRead(readPIN);
  pot = (pot / 1023) * 1.0;
  Serial.print("Pot: ");
  Serial.print(pot);
  Serial.print(", ");

  if (drist == 0)
  {
    mode = "DRIVE";
  }
  if (neust == 0)
  {
    mode = "NEUTRAL";
  }
  if (revst == 0)
  {
    mode = "REVERSE";
  }
  Serial.print("Current Mode: ");
  Serial.print(mode);
  Serial.print(", ");

  float rpm = 0.0;
  float volt = 0.0;
  float amps = 0.0;
  float power = 0.0;
  float duty = 0.0;

  lcd.clear();
  // lcd.print("Hi");

  // call this function if u wanna get data
  if (UART.getVescValues())
  {

    rpm = UART.data.rpm;
    volt = UART.data.inpVoltage;
    amps = UART.data.avgInputCurrent;
    power = UART.data.inpVoltage * UART.data.avgInputCurrent;
    duty = UART.data.dutyCycleNow;

    // Serial.print(" DUTY CURRENT: ");
    // Serial.print(duty);
    // Serial.print("data given  on lcd");

    String voltage  = "R: " + String((int)rpm);
    String current = " C: " + String(amps);
    String pow = "P: " + String((int)power);
    String dut = " D: " + String(duty);

    Serial.print(" ");
    Serial.print(voltage);
    Serial.print(" ");
    Serial.print(current);
    Serial.print("  ");
    Serial.print(pow);
    Serial.print(" ");
    Serial.print(dut);
    Serial.println("");

    lcd.setCursor(0, 0);
    lcd.print(voltage);
    int col = sizeof(voltage) / sizeof(String);
    lcd.setCursor(6, 0);
    lcd.print(current);
    lcd.setCursor(0, 1);
    lcd.print(pow);
    lcd.setCursor(6, 1);
    lcd.print(dut);
  }
  else
  {
    Serial.println("Failed to get data!");
  }

  if (mode == "DRIVE")
  {
    dutyset(duty, pot);
  }
  else if (mode == "REVERSE")
  {
    dutyset(duty, pot);
  }
  else if (mode == "NEUTRAL")
  {
    dutyset(duty, 0);
  }
  else
  {
    UART.setDuty(0);
    // Serial.println("idk what im doing i didnt get a mode");
  }


  delay(10);
}