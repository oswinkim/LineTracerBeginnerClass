#include <Wire.h>               //I2C 통신 라이브러리
#include <LiquidCrystal_I2C.h>  //I2C 통신 가능 LCD 라이브러리

byte I2CAddress = 0000;                    //임시 LCD I2C 주소
LiquidCrystal_I2C lcd(I2CAddress, 16, 2);  //임시 LCD 객체 생성


void setup() {
  Wire.begin();
  Serial.begin(9600);
  while (!Serial) {};
  Serial.print("\n---I2C Scanner---\n");

  I2CDeviceScaner();  //I2C주소 찾기
  lcd.init();         //lcd 시작
  lcd.clear();        //lcd 화면 초기화
  lcd.backlight();    //lcd 밝게 하기(백라이트) 키기

  Serial.print("---setup done---\n");  //셋업 종료
}


void loop() {
  lcd.clear();             //이전에 쓰인글자 모두 지우기
  lcd.setCursor(0, 0);     //커서를 setCusor(행, 열)로 이동 시키기
  lcd.print("my name: ");  //커서 위치에 글자 쓰기

  lcd.setCursor(0, 1);    //커서를 setCusor(행, 열)로 이동 시키기
  lcd.print("my age: ");  //커서 위치에 글자 쓰기

  delay(1000);  //잠시 멈추기 1초간


  lcd.clear();             //이전에 쓰인글자 모두 지우기
  lcd.setCursor(0, 0);     //커서를 setCusor(행, 열)로 이동 시키기
  lcd.print("my hobby: ");  //커서 위치에 글자 쓰기

  lcd.setCursor(0, 1);    //커서를 setCusor(행, 열)로 이동 시키기
  lcd.print("my favorite: ");  //커서 위치에 글자 쓰기

  delay(1000);  //잠시 멈추기
}









void I2CDeviceScaner() {
  while (true) {
    Serial.print("Scanning...\n");
    byte error;
    int DevicesCount = 0;
    for (byte address = 1; address < 127; address++) {
      Wire.beginTransmission(address);  //wire 통신 시작
      error = Wire.endTransmission();

      //wire 통신 종료 및 에러코드0(성공)
      if (error == 0) {
        Serial.print("I2C address:  0x");
        if (address < 16) {
          Serial.print("0");
        }
        Serial.print(address, HEX);
        Serial.print("\n");
        DevicesCount++;

        I2CAddress = address;
        lcd = LiquidCrystal_I2C(I2CAddress, 16, 2);  //클래스 객체 재정의
        return;
      } else if (error == 4) {
        Serial.print("Unknow error: 0x");
        if (address < 16) {
          Serial.print("0");
        }
        Serial.println(address, HEX);
      }
    }
    if (DevicesCount == 0) {
      Serial.print("No I2C devices found\n");
    }
  }
}