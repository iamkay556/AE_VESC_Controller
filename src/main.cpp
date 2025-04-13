#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include <VescUart.h>
#include <ezButton.h>

// init the switches -------> limit switches put them wherever there is space and add pin #s
ezButton driveSwitch(49);
ezButton neutralSwitch(51);
ezButton reverseSwitch(53);

String mode = "none";

#define readPIN A0       // pot pin
#define SignalBattery 22 // signal from buck to activate battery relay
#define SignalSolar 23   // signal from buck to activate solar relay

#define relayBattery 24 // output pin to control battery relay
#define relaySolar 25   // output pin to control solar relay

VescUart UART;

LiquidCrystal_I2C lcd(0x27, 15, 4);

void setup()
{

  lcd.init();
  lcd.backlight();

  Serial.begin(9600);

  /** Setup UART port (Serial1 on Atmega32u4) */
  Serial1.begin(115200);

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

  pinMode(SignalBattery, INPUT);
  pinMode(SignalSolar, INPUT);

  pinMode(relayBattery, OUTPUT);
  pinMode(relaySolar, OUTPUT);

  // rekays start in off mode
  digitalWrite(relayBattery, LOW);
  digitalWrite(relaySolar, LOW);

  driveSwitch.setDebounceTime(10);
  neutralSwitch.setDebounceTime(10);
  reverseSwitch.setDebounceTime(10);

  delay(1000);
}

void dutyset(float start, float end)
{
  const float step = 0.04;
  const float tolerance = 0.03;

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
      Serial.println("dutyset() func not working b/c no mode");

    delay(10);
  }
}

void loop()
{ // --------------------------------- LOOP ---------------------------------

  int batonoff = digitalRead(SignalBattery);
  int solonoff = digitalRead(SignalSolar);

  if (batonoff == 1)
  {
    digitalWrite(relayBattery, HIGH);
    Serial.println("BATTERY ON");
  }
  else
  {
    digitalWrite(relayBattery, LOW);
    Serial.println("BATTERY OFF");
  }
  if (solonoff == 1)
  {
    digitalWrite(relaySolar, HIGH);
    Serial.println("SOLAR ON");
  }
  else
  {
    digitalWrite(relaySolar, LOW);
    Serial.println("SOLAR OFF");
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
  Serial.println(pot);

  if (drist == 0)
  {
    mode = "DRIVE";
    Serial.println("Mode swithched to : DRIVE");
  }
  if (neust == 0)
  {
    mode = "NEUTRAL";
    Serial.println("Mode swithched to : NUETRAL");
  }
  if (revst == 0)
  {
    mode = "REVERSE";
    Serial.println("Mode swithched to : REVERSE");
  }

  Serial.println("CURRENT MODE IS:" + mode);

  float rpm = 0.0;
  float volt = 0.0;
  float amps = 0.0;
  float power = 0.0;
  float duty = 0.0;

  // call this function if u wanna get data
  if (UART.getVescValues())
  {

    rpm = UART.data.rpm;
    volt = UART.data.inpVoltage;
    amps = UART.data.avgInputCurrent;
    power = UART.data.inpVoltage * UART.data.avgInputCurrent;
    duty = UART.data.dutyCycleNow;

    Serial.print("DUTY CURRENT: ");
    Serial.println(duty);
    Serial.println("data given  on lcd");

    lcd.setCursor(0, 0);
    lcd.print("rpm:" + String(rpm));
    lcd.setCursor(0, 1);
    lcd.print("volt:" + String(volt));
    lcd.setCursor(0, 2);
    lcd.print("current:" + String(amps));
    lcd.setCursor(0, 3);
    lcd.print("power:" + String(power));

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
    Serial.println("idk what im doing i didnt get a mode");
  }


  delay(500);
}