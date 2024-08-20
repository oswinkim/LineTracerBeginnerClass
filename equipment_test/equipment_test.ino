#include <LiquidCrystal_I2C.h>
#include <Wire.h>
 
#include <Wire.h>
 
#define IN1 7
#define IN2 5
#define IN3 3
#define IN4 2

#define speed_left 6       // 모터 속도 제어 왼쪽
#define speed_right 4      // 모터 속도 제어 오른쪽

#define fixed_speed_left 160      // 보정한 값
#define fixed_speed_right 190     // 보정한 값

LiquidCrystal_I2C lcd(0x27, 16, 2);

void forward() {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(speed_left, fixed_speed_left);
    analogWrite(speed_right, fixed_speed_right);
}

void setup()
{
  lcd.init();
  lcd.backlight();
  Wire.begin();
 
  Serial.begin(9600);
  while (!Serial);
  Serial.println("\nI2C Scanner");
  lcd.clear();

    pinMode(IN1, OUTPUT);  // A 모터 1
    pinMode(IN2, OUTPUT);  // A 모터 2
    pinMode(IN3, OUTPUT);  // B 모터 1
    pinMode(IN4, OUTPUT);  // B 모터 2

    pinMode(speed_left, OUTPUT);
    pinMode(speed_right, OUTPUT);
    }
 
void loop()
{
  forward();
  byte error, address;
  int nDevices;
  Serial.println("Scanning...");
  nDevices = 0;
  for (address = 1; address < 127; address++ )
  {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0)
    {
      Serial.print("::::");
      if (address < 16)
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");
      forward();
      if (int(address)==39){
        LiquidCrystal_I2C lcd(0x27, 16, 2);
        lcd.backlight();
        lcd.setCursor(0, 0);
        lcd.print("27");  // 문구를 출력합니다.
      }
      else{
        LiquidCrystal_I2C lcd(0x3F, 16, 2);
        lcd.backlight();
        lcd.setCursor(0, 0);
        lcd.print("3F");  // 문구를 출력합니다.
      }

 
      nDevices++;
    }

  }

  delay(5000);
  
}