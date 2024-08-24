#include <Wire.h>               //I2C 통신 라이브러리
#include <LiquidCrystal_I2C.h>  //I2C 통신 가능 LCD 라이브러리

//버튼 핀(노랑, 초록, 파랑)
#define ButtonPin_1 8
#define ButtonPin_2 12
#define ButtonPin_3 13

//LED 핀(주황, 빨강, 갈색)
#define LED_R 11
#define LED_G 10
#define LED_B 9

#define MaxLEDBright 10  //LED 최대 밝기(눈뽕 방지)

//선택한 LED
int LEDSelect = 0;
//버튼 눌림 횟수 계수
int ButtonCount = 0;

//임시 LCD I2C 주소
byte I2CAddress = 0000;
LiquidCrystal_I2C lcd(I2CAddress, 16, 2);  //임시 LCD 객체 생성

int RedLEDValue;
int GreenLEDValue;
int BlueLEDValue;



int mode = 0;

bool flag = true;


void setup() {
  Wire.begin();
  Serial.begin(9600);
  while (!Serial) {};
  Serial.print("\n---I2C Scanner---\n");

  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);

  pinMode(ButtonPin_1, INPUT_PULLUP);
  pinMode(ButtonPin_2, INPUT_PULLUP);
  pinMode(ButtonPin_3, INPUT_PULLUP);

  I2CDeviceScaner();  //I2C주소 찾기
  lcd.init();         //lcd 시작
  lcd.clear();        //lcd 화면 초기화
  lcd.backlight();    //lcd 밝게 하기(백라이트) 키기

  Serial.print("---setup done---\n");  //셋업 종료
}



void loop() {
  //버튼들의 디지털신호 읽어 오기
  int Button1 = digitalRead(ButtonPin_1);
  int Button2 = digitalRead(ButtonPin_2);
  int Button3 = digitalRead(ButtonPin_3);

  //버튼 카운트
  if (Button1 == LOW) {  // 1번 버튼 눌림
    if (ButtonCount > 0) {
      ButtonCount -= 1;
    }
  }
  if ((Button2 == LOW) && (flag == true)) {  // 2번 버튼 눌림
    LEDSelect++;
    if (LEDSelect % 3 == 0) {
      ButtonCount = RedLEDValue;
    } else if (LEDSelect % 3 == 1) {
      ButtonCount = GreenLEDValue;
    } else if (LEDSelect % 3 == 2) {
      ButtonCount = BlueLEDValue;
    }
    flag = false;
  } else if (Button2 == HIGH) {
    flag = true;
  }
  if (Button3 == LOW) {  // 3번 버튼 눌림
    if (ButtonCount < 255) {
      ButtonCount += 1;
    }
  }

  //LED 선택
  if (LEDSelect % 3 == 0) {
    RedLEDValue = ButtonCount;
    lcd.setCursor(0, 0);
    lcd.print("[R]     G     B ");
  } else if (LEDSelect % 3 == 1) {
    GreenLEDValue = ButtonCount;
    lcd.setCursor(0, 0);
    lcd.print(" R     [G]    B ");
  } else if (LEDSelect % 3 == 2) {
    BlueLEDValue = ButtonCount;
    lcd.setCursor(0, 0);
    lcd.print(" R      G    [B]");
  }

  //LCD 쓰기
  LCDUpdate(RedLEDValue, GreenLEDValue, BlueLEDValue);


  if (mode == 0) {
    //               //
    ////부등호 과제////
    //               //
    //빨강(RedLEDValue)이 제일 큰가 판단하시오
    if ((RedLEDValue 부등호 GreenLEDValue) && (RedLEDValue 부등호 BlueLEDValue)) {
      LED_SetRed();
    }
    //초록(GreenLEDValue)이 제일 큰가 판단하시오
    else if ((GreenLEDValue 부등호 RedLEDValue) && (GreenLEDValue 부등호 BlueLEDValue)) {
      LED_SetGreen();
    }
    //파랑(BlueLEDValue)이 제일 큰가 판단 하시오
    else if ((BlueLEDValue  부등호 RedLEDValue) && (BlueLEDValue 부등호 GreenLEDValue)) {
      LED_SetBlue();
    }
    //               //
    ////부등호 과제////
    //               //
  } else {
    analogWrite(LED_R, RedLEDValue);
    analogWrite(LED_G, GreenLEDValue);
    analogWrite(LED_B, BlueLEDValue);
  }
}








void LCDUpdate(int R, int G, int B) {
  //lcd에 누른 횟수 표시
  lcd.setCursor(0, 1);
  if (R < 10) {
    lcd.print("  " + String(R));
  } else if (R < 100) {
    lcd.print(" " + String(R));
  } else {
    lcd.print(R);
  }
  lcd.setCursor(7, 1);
  if (G < 10) {
    lcd.print("  " + String(G));
  } else if (G < 100) {
    lcd.print(" " + String(G));
  } else {
    lcd.print(G);
  }
  lcd.setCursor(13, 1);
  if (B < 10) {
    lcd.print("  " + String(B));
  } else if (B < 100) {
    lcd.print(" " + String(B));
  } else {
    lcd.print(B);
  }
}

void LED_SetRed() {
  analogWrite(LED_R, MaxLEDBright);  //빨간색 LED 켜기
  analogWrite(LED_G, 0);
  analogWrite(LED_B, 0);
}

void LED_SetGreen() {
  analogWrite(LED_R, 0);
  analogWrite(LED_G, MaxLEDBright);  //초록색 LED 켜기
  analogWrite(LED_B, 0);
}

void LED_SetBlue() {
  analogWrite(LED_R, 0);
  analogWrite(LED_G, 0);
  analogWrite(LED_B, MaxLEDBright);  //파란색 LED 켜기
}

void LED_SetWhite() {
  analogWrite(LED_R, (int)(MaxLEDBright / 3));
  analogWrite(LED_G, (int)(MaxLEDBright / 3));
  analogWrite(LED_B, (int)(MaxLEDBright / 3));
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