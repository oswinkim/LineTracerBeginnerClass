#include <Wire.h>               //I2C 통신 라이브러리
#include <LiquidCrystal_I2C.h>  //I2C 통신 가능 LCD 라이브러리

//버튼 핀(노랑, 초록, 파랑)
#define ButtonPin_1 8
#define ButtonPin_2 12
#define ButtonPin_3 13

#define MotorPin_LeftF 4   //좌측 모터 전진 핀
#define MotorPin_LeftB 2   //좌측 모터 후진 핀
#define MotorPin_RightF 7  //우측 모터 전진 핀
#define MotorPin_RightB 5  //우측 모터 후진 핀

#define LeftMotor 3   // 좌측 모터 속도 제어 핀
#define RightMotor 6  // 오른 쪽모터 속도 제어 핀

#define MinMotorOutput 0    //모터 최저 출력
#define MaxMotorOutput 255  //모터 최고 출력

int LeftSpeed = MaxMotorOutput;   //좌측 모터 속도
int RightSpeed = MaxMotorOutput;  //우측 모터 속도

//임시 LCD I2C 주소
byte I2CAddress = 0000;
LiquidCrystal_I2C lcd(I2CAddress, 16, 2);  //임시 LCD 객체 생성


void setup() {
  Wire.begin();
  Serial.begin(9600);
  while (!Serial) {};
  Serial.print("\n---I2C Scanner---\n");

  //버튼을 사용하기 위한 핀들은 입력으로 설정하기
  pinMode(ButtonPin_1, INPUT_PULLUP);
  pinMode(ButtonPin_2, INPUT_PULLUP);
  pinMode(ButtonPin_3, INPUT_PULLUP);

  //모터를 사용하기 위한 핀들은 출력으로 설정하기
  pinMode(MotorPin_LeftF, OUTPUT);
  pinMode(MotorPin_LeftB, OUTPUT);
  pinMode(MotorPin_RightF, OUTPUT);
  pinMode(MotorPin_RightB, OUTPUT);
  pinMode(LeftMotor, OUTPUT);
  pinMode(RightMotor, OUTPUT);

  I2CDeviceScaner();  //I2C주소 찾기
  lcd.init();         //lcd 시작
  lcd.clear();        //lcd 화면 초기화
  lcd.backlight();    //lcd 밝게 하기(백라이트) 키기

  //버튼 기호 출력
  lcd.setCursor(0, 0);
  lcd.print("Button:");

  //모터 기호 출력
  lcd.setCursor(0, 1);
  lcd.print("Motor:");
  lcd.setCursor(10, 1);
  lcd.print("/");

  Serial.print("---setup done---\n");  //셋업 종료
}



void loop() {
  //버튼들의 디지털신호 읽어 오기
  int Button1 = digitalRead(ButtonPin_1);
  int Button2 = digitalRead(ButtonPin_2);
  int Button3 = digitalRead(ButtonPin_3);

  //1번 버튼의 입력을 체크하기(LOW: 눌림, HIGH: 눌리지 않음)
  if (Button1 == LOW) {
    RightSpeed -= 1;  //1번 버튼이 눌렸으면 우측모터를 1만큼 늦추기
  }
  //2번 버튼의 입력을 체크하기(LOW: 눌림, HIGH: 눌리지 않음)
  if (Button2 == LOW) {
    LeftSpeed -= 1;  //2번 버튼이 눌렸으면 좌측모터를 1만큼 늦추기
  }
  //3번 버튼의 입력을 체크하기(LOW: 눌림, HIGH: 눌리지 않음)
  if (Button3 == LOW) {
    LeftSpeed += 1;   //3번 버튼이 눌렸으면 좌측모터를 1만큼 빠르게 하기
    RightSpeed += 1;  //3번 버튼이 눌렸으면 우측모터를 1만큼 빠르게 하기
  }

  //좌, 우측 모터의 속도를 0~255로 제한하기
  LeftSpeed = max(MinMotorOutput, min(MaxMotorOutput, LeftSpeed));    // * (1 - Diffential4Staright)));
  RightSpeed = max(MinMotorOutput, min(MaxMotorOutput, RightSpeed));  // * (1 + Diffential4Staright)));

  forward(LeftSpeed, RightSpeed);  //모터 앞으로 움직이기

  //버튼 상태 출력하기
  lcd.setCursor(8, 0);
  lcd.print(String(Button1) + String(Button2) + String(Button3));

  //모터 속도 출력하기
  lcd.setCursor(7, 1);   //커서를 setCusor(행, 열)로 이동 시키기
  lcdprint(RightSpeed);  //커서 위치에 글자수를 세자리에 맞춰 출력하기
  lcd.setCursor(11, 1);  //커서를 setCusor(행, 열)로 이동 시키기
  lcdprint(LeftSpeed);   //커서 위치에 글자수를 세자리에 맞춰 출력하기
}

void forward(int LeftSpeed, int RightSpeed) {
  digitalWrite(MotorPin_LeftF, HIGH);   //좌측 전진신호를 높음(HIGH)으로 주기
  digitalWrite(MotorPin_LeftB, LOW);    //좌측 후진신호를 낮음(LOW)으로 주기
  digitalWrite(MotorPin_RightF, HIGH);  //우측 전진신호를 높음(HIGH)으로 주기
  digitalWrite(MotorPin_RightB, LOW);   //우측 후진신호를 낮음(LOW)으로 주기

  analogWrite(LeftMotor, LeftSpeed);    //좌측 모터를 속도(LeftSpeed)만큼 회전시키기
  analogWrite(RightMotor, RightSpeed);  //우측 모터를 속도(RightSpeed)만큼 회전시키기
}









void lcdprint(int speed) {
  if (speed < 10) {
    lcd.print("  " + String(speed));
  } else if (speed < 100) {
    lcd.print(" " + String(speed));
  } else {
    lcd.print(speed);
  }
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