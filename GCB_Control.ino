#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;
String data = "";
int spd = 0;
String dir = "";
unsigned long t = 0;
short pwm = 0;

#define spd1 27
#define spd2 12
#define adir 26
#define bdir 14

void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32_Test"); //Bluetooth device name
  Serial.println("The device started, now you can pair it with bluetooth!");
  pinMode(spd1, OUTPUT);
  pinMode(spd2, OUTPUT);
  pinMode(adir, OUTPUT);
  pinMode(bdir, OUTPUT);
  digitalWrite(adir,LOW);
  digitalWrite(bdir,LOW);
}

void loop() {
if(SerialBT.available() > 0)   
{
    data = SerialBT.readStringUntil(',');
    dir = (char)(data.toInt());
    data = SerialBT.readStringUntil('/');
    spd = data.toInt();

 if (dir == "s")
 {
  Serial.println("STOP");
  digitalWrite(adir,LOW);
  digitalWrite(bdir,LOW);
  pwm = 0;
  analogWrite(spd1, 0);
  analogWrite(spd2, 0);
 }
 if (dir == "f")
 {
  Serial.println("Forward");
  digitalWrite(adir,LOW);
  digitalWrite(bdir,LOW);
  analogWrite(spd1, pwm);
  analogWrite(spd2, pwm);
 }
 if (dir == "b")
 {
  Serial.println("Back");
  digitalWrite(adir,HIGH);
  digitalWrite(bdir,HIGH);
  analogWrite(spd1, pwm);
  analogWrite(spd2, pwm);
 }
 if (dir == "r")
 {
  Serial.println("Right");
  digitalWrite(adir,LOW);
  digitalWrite(bdir,HIGH);
  analogWrite(spd1, pwm);
  analogWrite(spd2, pwm);
 }
 if (dir == "l")
 {
  Serial.println("Left");
  digitalWrite(adir,HIGH);
  digitalWrite(bdir,LOW);
  analogWrite(spd1, pwm);
  analogWrite(spd2, pwm);
 }
 if (spd == 1) { pwm = 255/3; }
 if (spd == 2) { pwm = 2 * 255/3; }
 if (spd == 3) { pwm = 250; }

 Serial.print("Direction:");
 Serial.println(dir);
 Serial.print("Speed:");
 Serial.println(spd);
 Serial.println(pwm);

 delay(10);

 data = "";
 dir = "";
 spd = 0;
}
else
{
  pwm = 0;
  analogWrite(spd1, pwm);
  analogWrite(spd2, pwm);
  Serial.println("No Data.");
}
}
// MAC: b0:b2:1c:0a:db:44
// BT: b0:b2:1c:0a:db:46
