#include <Wire.h>               //I2C 통신 라이브러리
#include <LiquidCrystal_I2C.h>  //I2C 통신 가능 LCD 라이브러리

//버튼 핀(노랑, 초록, 파랑)
#define ButtonPin_1 8
#define ButtonPin_2 12
#define ButtonPin_3 13

#define line_detector_left A1   // 좌측 적외선 센서
#define line_detector_right A0  // 우측 적외선 센서

//LED 핀(주황, 빨강, 갈색)
#define LED_R 11
#define LED_G 10
#define LED_B 9

#define MotorPin_LeftF 7   //좌측 모터 전진 핀
#define MotorPin_LeftB 5   //좌측 모터 후진 핀
#define MotorPin_RightF 4  //우측 모터 전진 핀
#define MotorPin_RightB 2  //우측 모터 후진 핀

#define LeftMotor 3   // 좌측 모터 속도 제어 핀
#define RightMotor 6  // 오른 쪽모터 속도 제어 핀

#define MinMotorOutput 0    //모터 최저 출력
#define MaxMotorOutput 255  //모터 최고 출력

#define MaxLEDBright 10  //LED 최대 밝기(눈뽕 방지)

float Diffential4Staright = 0.00;  //직진 상태 유지를 위한 모터 조향값
int LeftSpeed = 255;               //좌측 모터 속도
int RightSpeed = 255;              //우측 모터 속도

//버튼 눌리지 않음 상태
bool Button1_Flag = true;
bool Button2_Flag = true;
bool Button3_Flag = true;

//버튼 눌림 횟수 계수
int Button1_Count = 0;
int Button2_Count = 0;
int Button3_Count = 0;

//임시 LCD I2C 주소
byte I2CAddress = 0000;
LiquidCrystal_I2C lcd(I2CAddress, 16, 2);  //임시 LCD 객체 생성



void LED_SetRed() {
  analogWrite(LED_R, MaxLEDBright);
  analogWrite(LED_G, 0);
  analogWrite(LED_B, 0);
}

void LED_SetGreen() {
  analogWrite(LED_R, 0);
  analogWrite(LED_G, MaxLEDBright);
  analogWrite(LED_B, 0);
}

void LED_SetBlue() {
  analogWrite(LED_R, 0);
  analogWrite(LED_G, 0);
  analogWrite(LED_B, MaxLEDBright);
}

void LED_SetWhite() {
  analogWrite(LED_R, (int)(MaxLEDBright / 3));
  analogWrite(LED_G, (int)(MaxLEDBright / 3));
  analogWrite(LED_B, (int)(MaxLEDBright / 3));
}

void forward(int LeftSpeed, int RightSpeed) {
  digitalWrite(MotorPin_LeftF, HIGH);
  digitalWrite(MotorPin_LeftB, LOW);
  digitalWrite(MotorPin_RightF, HIGH);
  digitalWrite(MotorPin_RightB, LOW);

  analogWrite(LeftMotor, LeftSpeed);
  analogWrite(RightMotor, RightSpeed);
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



void setup() {
  Wire.begin();
  Serial.begin(9600);
  while (!Serial) {};
  Serial.print("\n---I2C Scanner---\n");
  I2CDeviceScaner();

  pinMode(ButtonPin_1, INPUT_PULLUP);
  pinMode(ButtonPin_2, INPUT_PULLUP);
  pinMode(ButtonPin_3, INPUT_PULLUP);

  pinMode(line_detector_left, INPUT);
  pinMode(line_detector_right, INPUT);

  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);

  pinMode(MotorPin_LeftF, OUTPUT);
  pinMode(MotorPin_LeftB, OUTPUT);
  pinMode(MotorPin_RightF, OUTPUT);
  pinMode(MotorPin_RightB, OUTPUT);
  pinMode(LeftMotor, OUTPUT);
  pinMode(RightMotor, OUTPUT);

  lcd.init();
  lcd.clear();
  lcd.backlight();

  //I2C 주소 출력
  lcd.setCursor(0, 0);
  lcd.print(String("I2C:0x" + String(I2CAddress, HEX)));

  //버튼 상태 출력
  lcd.setCursor(11, 1);
  lcd.print("B:");

  //센서 상태 출력
  lcd.setCursor(11, 0);
  lcd.print("S:");
  lcd.setCursor(14, 0);
  lcd.print("/");

  //모터 조향 출력
  lcd.setCursor(0, 1);
  lcd.print("M:");
  lcd.setCursor(5, 1);
  lcd.print("/");

  Serial.print("---setup done---\n");
}



void loop() {
  int Button1 = digitalRead(ButtonPin_1);
  int Button2 = digitalRead(ButtonPin_2);
  int Button3 = digitalRead(ButtonPin_3);

  int left_sensor = digitalRead(line_detector_left);    // 적외선 센서 값(40~600)
  int right_sensor = digitalRead(line_detector_right);  // 적외선 센서 값(40~600)

  ///*연속 입력 가능
  Button1_Flag = true;
  Button2_Flag = true;
  Button3_Flag = true;
  //*/해제하려면 주석처리

  if ((Button1 == LOW) && (Button1_Flag)) {
    LeftSpeed--;
    Diffential4Staright -= 0.01;
    Button1_Count++;
    Button1_Flag = false;
  } else if (Button1 == HIGH) {
    Button1_Flag = true;
  }
  if ((Button2 == LOW) && (Button2_Flag)) {
    RightSpeed--;
    Diffential4Staright += 0.01;
    Button2_Count++;
    Button2_Flag = false;
  } else if (Button2 == HIGH) {
    Button2_Flag = true;
  }
  if ((Button3 == LOW) && (Button3_Flag)) {
    LeftSpeed++;
    RightSpeed++;
    Button3_Count++;
    Button3_Flag = false;
  } else if (Button3 == HIGH) {
    Button3_Flag = true;
  }

  Diffential4Staright = max(-1, min(1, Diffential4Staright));
  LeftSpeed = max(MinMotorOutput, min(MaxMotorOutput, LeftSpeed));    // * (1 - Diffential4Staright)));
  RightSpeed = max(MinMotorOutput, min(MaxMotorOutput, RightSpeed));  // * (1 + Diffential4Staright)));
  forward(LeftSpeed, RightSpeed);

  if (Button1_Count > max(Button2_Count, Button3_Count)) {
    LED_SetRed();
  } else if (Button2_Count > max(Button1_Count, Button3_Count)) {
    LED_SetGreen();
  } else if (Button3_Count > max(Button1_Count, Button2_Count)) {
    LED_SetBlue();
  } else if ((Button1_Count == max(Button2_Count, Button3_Count)) || (Button2_Count == max(Button1_Count, Button3_Count)) || (Button3_Count == max(Button1_Count, Button2_Count))) {
    LED_SetWhite();
  }

  //버튼 상태 럽데이트
  lcd.setCursor(13, 1);
  lcd.print(Button1);
  lcd.setCursor(14, 1);
  lcd.print(Button2);
  lcd.setCursor(15, 1);
  lcd.print(Button3);

  //버튼 카운트 업데이트
  Serial.print("Button_1: ");
  Serial.print(Button1_Count);
  Serial.print(" Button_2: ");
  Serial.print(Button2_Count);
  Serial.print(" Button_3: ");
  Serial.println(Button3_Count);

  //센서 상태 럽데이트
  lcd.setCursor(13, 0);
  lcd.print(left_sensor);
  lcd.setCursor(15, 0);
  lcd.print(right_sensor);

  //모터 조향 업데이트
  lcd.setCursor(2, 1);
  if (LeftSpeed < 10) {
    lcd.print("  " + String(LeftSpeed));
  } else if (LeftSpeed < 100) {
    lcd.print(" " + String(LeftSpeed));
  } else {
    lcd.print(LeftSpeed);
  }
  lcd.setCursor(6, 1);
  if (RightSpeed < 10) {
    lcd.print("  " + String(RightSpeed));
  } else if (RightSpeed < 100) {
    lcd.print(" " + String(RightSpeed));
  } else {
    lcd.print(RightSpeed);
  }
  /*  if (LeftSpeed >= 100) {
    lcd.setCursor(2, 1);
    if (Diffential4Staright < 0.01) {
      lcd.print("  " + String((int)(Diffential4Staright * 100)));
    } else if (Diffential4Staright < 0.1) {
      lcd.print(" " + String((int)(Diffential4Staright * 100)));
    } else {
      lcd.print(" " + String((int)(Diffential4Staright * 100)));
    }
  }  */
}


/*//1번 버튼의 입력을 체크하기(LOW: 눌림, HIGH: 눌리지 않음)
  if ((Button1 == LOW) && (Button1_Flag)) {
    LeftSpeed -= 2;        //1번 버튼이 눌렸으면 좌측모터를 2만큼 늦추기
    Button1_Flag = false;  //1번 버튼을 눌려있는 상태로 저장
  } else if (Button1 == HIGH) {
    Button1_Flag = true;  //1번 버튼을 눌려 있지 않은 상태로 저장
  }
  //2번 버튼의 입력을 체크하기(LOW: 눌림, HIGH: 눌리지 않음)
  if ((Button2 == LOW) && (Button2_Flag)) {
    RightSpeed -= 2;       //2번 버튼이 눌렸으면 우측모터를 2만큼 늦추기
    Button2_Flag = false;  //2번 버튼을 눌려있는 상태로 저장
  } else if (Button2 == HIGH) {
    Button2_Flag = true;  //2번 버튼을 눌려 있지 않은 상태로 저장
  }
  //3번 버튼의 입력을 체크하기(LOW: 눌림, HIGH: 눌리지 않음)
  if ((Button3 == LOW) && (Button3_Flag)) {
    LeftSpeed += 2;        //3번 버튼이 눌렸으면 좌측모터를 2만큼 빠르게 하기
    RightSpeed += 2;       //3번 버튼이 눌렸으면 우측모터를 2만큼 빠르게 하기
    Button3_Flag = false;  //3번 버튼을 눌려있는 상태로 저장
  } else if (Button3 == HIGH) {
    Button3_Flag = true;  //3번 버튼을 눌려 있지 않은 상태로 저장
  }*/
