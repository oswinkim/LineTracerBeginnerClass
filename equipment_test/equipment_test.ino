#include <LiquidCrystal_I2C.h>
#include <Wire.h>
 
#include <Wire.h>
 
#define IN1 2
#define IN2 4
#define IN3 6
#define IN4 7

#define speed_left 3       // 모터 속도 제어 왼쪽
#define speed_right 5      // 모터 속도 제어 오른쪽
LiquidCrystal_I2C lcd(0x27, 16, 2);

void forward() {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(speed_left, 255);
    analogWrite(speed_right, 255);
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

}
 
void loop()
{
  
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