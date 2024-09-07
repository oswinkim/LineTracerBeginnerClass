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

#define MaxLEDBright 1  //LED 최대 밝기(눈뽕 방지)

#define MotorPin_LF 4  //좌측 모터 전진 핀
#define MotorPin_LB 2  //좌측 모터 후진 핀
#define MotorPin_RF 7  //우측 모터 전진 핀
#define MotorPin_RB 5  //우측 모터 후진 핀

#define LeftMotor 3   // 좌측 모터 속도 제어 핀
#define RightMotor 6  // 우측모터 속도 제어 핀

//ㅐㅜㅐㄹㄹ
#define MinMotorOutput 0    //모터 최저 출력
#define MaxMotorOutput 255  //모터 최고 출력

//ㅔ
//#define MinMotorOutput 40   //모터 최저 출력
//#define MaxMotorOutput 220  //모터 최고 출력



//버튼 관련
//버튼 누름 시작 시간
unsigned long Button1_press_start = 0;
unsigned long Button2_press_start = 0;
unsigned long Button3_press_start = 0;
//버튼 입력 간격
unsigned int Button1_count_interval = 200;
unsigned int Button3_count_interval = 200;
unsigned int Button2_count_interval = 500;
//버튼 입력 가능 상태
bool Button1_Flag = true;
bool Button2_Flag = true;
bool Button3_Flag = true;
bool ButtonMODE_Flag = true;
//버튼 입력 횟수
int Button1_Count = 0;
int Button2_Count = 0;
int Button3_Count = 0;

//lcd 관련
byte I2CAddress = 0000;                    //임시 LCD I2C 주소
LiquidCrystal_I2C lcd(I2CAddress, 16, 2);  //임시 LCD 객체 생성

//모터 관련
float Diffential4Staright = 0.00;                               //직진 상태 유지를 위한 모터 조향값
int LeftSpeed = (int)((MinMotorOutput + MaxMotorOutput) / 2);   //좌측 모터 초기 속도
int RightSpeed = (int)((MinMotorOutput + MaxMotorOutput) / 2);  //우측 모터 초기 속도

int softmin = -222;  //-222;
int softmax = 222;

int minSpeed = 0;
int maxSpeed = 0;

//모드 관련
int mode = 0;

//제어 관련
int skew = 0;  // 치우침 방향(0: 정, -1: 좌, 1:우)

//pid 관련
float kp = 0.25;
float kd = 0.01;
float ki = 0.05;

long error = 0;
long pre_error = 0;
long error_integration = 0;
long error_Differention = 0;

unsigned long Left_error_start_time = 0;

unsigned long Right_error_start_time = 0;

//pid LeftMotorSpeed(MaxLeft);
//pid RightMotorSpeed(MaxRight);

unsigned long left_start_time = 0;
unsigned long right_start_time = 0;


int ConvertMotor = 0;

int L_Polarity = 1;
int R_Polarity = 1;


void setup() {
  Wire.begin();
  Serial.begin(9600);
  while (!Serial) {};
  Serial.print("\n---I2C scanner---\n");
  //I2C 주소 탐색
  I2CDeviceScaner();
  //lcd 초기화
  lcd.init();
  lcd.backlight();
  lcd.clear();

  pinMode(ButtonPin_1, INPUT_PULLUP);
  pinMode(ButtonPin_2, INPUT_PULLUP);
  pinMode(ButtonPin_3, INPUT_PULLUP);

  pinMode(line_detector_left, INPUT);
  pinMode(line_detector_right, INPUT);

  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);

  pinMode(MotorPin_LF, OUTPUT);
  pinMode(MotorPin_LB, OUTPUT);
  pinMode(MotorPin_RF, OUTPUT);
  pinMode(MotorPin_RB, OUTPUT);
  pinMode(LeftMotor, OUTPUT);
  pinMode(RightMotor, OUTPUT);

  ConvertMotor = RightSpeed;
  //pid LeftPID(MaxLeft);
  //pid RightPID(MaxRight);

  Serial.print("---setup done---\n");
  Serial.print("---mode: " + String(mode) + "---\n");
}



void loop() {
  int left_sensor = digitalRead(line_detector_left);
  int right_sensor = digitalRead(line_detector_right);

  int Button1 = digitalRead(ButtonPin_1);
  int Button2 = digitalRead(ButtonPin_2);
  int Button3 = digitalRead(ButtonPin_3);

  //2번 버튼 3초 누르면 모드 변경
  if ((Button2 == LOW) && (ButtonMODE_Flag)) {
    Button2_press_start = millis();
    ButtonMODE_Flag = false;
  } else if ((Button2 == LOW) && (Button2_press_start != 0)) {
    if (millis() - Button2_press_start >= Button2_count_interval) {
      mode++;
      //모드 도달 제한
      if (mode >= 2) {
        mode = 0;
      }

      if (mode == 0) {
        LeftSpeed = maxSpeed * L_Polarity;
        RightSpeed = minSpeed * R_Polarity;
        ConvertMotor = RightSpeed;
      } else if (mode == 1) {
        minSpeed = min(abs(LeftSpeed), abs(RightSpeed));
        maxSpeed = max(abs(LeftSpeed), abs(RightSpeed));
        if (LeftSpeed >= 0) {
          L_Polarity = 1;
        } else {
          L_Polarity = -1;
        }
        if (RightSpeed >= 0) {
          R_Polarity = 1;
        } else {
          R_Polarity = -1;
        }
      }


      Button1_Count = 0;
      Button2_Count = 0;
      Button3_Count = 0;
      lcd.clear();
      Serial.print("\n---mode: " + String(mode) + "---\n");
      Button2_press_start = 0;
    }
  } else if (Button2 == HIGH) {
    ButtonMODE_Flag = true;
    Button2_press_start = 0;
  }


  //0번: 테스트 모드
  if (mode == 0) {
    //2번 버튼: 좌 우 모터 선택
    if ((Button2 == LOW) && (Button2_Flag)) {
      Button2_Count++;
      //변환할 모터 속도를 불러오기
      if (Button2_Count % 2 == 0) {
        ConvertMotor = RightSpeed;
      } else {
        ConvertMotor = LeftSpeed;
      }
      Button2_Flag = false;
    } else if (Button2 == HIGH) {
      //변환한 모터 속도를 적용
      if (Button2_Count % 2 == 0) {
        RightSpeed = ConvertMotor;
      } else {
        LeftSpeed = ConvertMotor;
      }
      Button2_Flag = true;
    }

    //1번 버튼: 속도 증가
    if (Button1 == LOW) {
      if (Button1_Flag) {
        Button1_press_start = millis();
        Button1_Count++;
        ConvertMotor++;
      }  //꾹 누르면 빠르게 증가
      else if (millis() - Button1_press_start >= Button1_count_interval) {
        ConvertMotor += 1 + min(4, (millis() - Button1_press_start - Button1_count_interval) * 0.005);
      }
      Button1_Flag = false;
    } else if (Button1 == HIGH) {
      Button1_Flag = true;
      Button1_press_start = 0;
    }
    //3번 버튼: 속도 감소
    if (Button3 == LOW) {
      if (Button3_Flag) {
        Button3_press_start = millis();
        Button3_Count++;
        ConvertMotor--;
      }  //꾹 누르면 빠르게 감소
      else if (millis() - Button3_press_start >= Button3_count_interval) {
        ConvertMotor -= 1 + min(4, (millis() - Button3_press_start - Button3_count_interval) * 0.005);
      }
      Button3_Flag = false;
    } else if (Button3 == HIGH) {
      Button3_Flag = true;
      Button3_press_start = 0;
    }

    //속도 제한
    ConvertMotor = max(softmin, min(softmax, ConvertMotor));
    //Diffential4Staright = max(0, min(1, (LeftSpeed / RightSpeed)));

    //모터 동작
    MotorAction(LeftSpeed, RightSpeed);

    //가장 많이 누른 버튼의 색으로 LED 점등
    if (Button1_Count > max(Button2_Count, Button3_Count)) {
      LED_SetRed();
    } else if (Button2_Count > max(Button1_Count, Button3_Count)) {
      LED_SetGreen();
    } else if (Button3_Count > max(Button1_Count, Button2_Count)) {
      LED_SetBlue();
    } else if ((Button1_Count == max(Button2_Count, Button3_Count)) || (Button2_Count == max(Button1_Count, Button3_Count)) || (Button3_Count == max(Button1_Count, Button2_Count))) {
      LED_SetWhite();
    }

    //모드 출력
    lcd.setCursor(0, 0);
    lcd.print("M:test");
    //버튼 상태 출력
    //lcd.setCursor(7, 0);
    //lcd.print("B:" + String(Button1) + String(Button2) + String(Button3));
    //버튼 카운트 출력
    //Serial.print("Button_1: " + String(Button1_Count) + " Button_2: " + String(Button2_Count) + " Button_3: " + String(Button3_Count) + "\n");
    //센서 상태 출력
    lcd.setCursor(11, 0);
    lcd.print("S:" + String(right_sensor) + "/" + String(left_sensor));
    //Serial.print("LS: " + String(left_sensor) + " RS: " + String(right_sensor) + "\n");
    //모터 조향 출력
    String LeftSpeedDisplay = "";
    String RightSpeedDisplay = "";
    for (int i = 4; i > String(LeftSpeed).length(); i--) {
      LeftSpeedDisplay += " ";
    }
    LeftSpeedDisplay += String(LeftSpeed);
    for (int i = 4; i > String(RightSpeed).length(); i--) {
      RightSpeedDisplay += " ";
    }
    RightSpeedDisplay += String(RightSpeed);
    if (Button2_Count % 2 == 0) {
      lcd.setCursor(0, 1);
      lcd.print("[R:" + String(RightSpeedDisplay) + "]");
      lcd.setCursor(8, 1);
      lcd.print(" L:" + String(LeftSpeedDisplay) + " ");
    } else {
      lcd.setCursor(0, 1);
      lcd.print(" R:" + String(RightSpeedDisplay) + " ");
      lcd.setCursor(8, 1);
      lcd.print("[L:" + String(LeftSpeedDisplay) + "]");
    }
    //Serial.print("LM: " + String(LeftSpeedDisplay) + " RM: " + String(RightSpeedDisplay) + "\n");
  }

  //1번 온오프 제어 주행
  else if (mode == 1) {
    /*
    //양측 센서 모두 선 감지 시 치우침 없음
    if ((left_sensor == HIGH) && (right_sensor == HIGH)) {
      skew = 0;
    }
    //우측 센서만 선 감지 시 좌측 치우침
    else if ((left_sensor == HIGH) && (right_sensor == LOW)) {
      skew = -1;
    }
    //좌측 센서만 선 감지 시 우측 치우침
    if ((left_sensor == LOW) && (right_sensor == HIGH)) {
      skew = 1;
    }*/
    /*
    //양측 센서 모두 선 감지 시 치우침 없음
    if ((left_sensor == LOW) && (right_sensor == LOW)) {
      skew = 0;
    }*/
    //우측 센서만 선 감지 시 좌측 치우침
    if ((left_sensor == HIGH) && (right_sensor == LOW)) {
      skew = -1;
      LeftSpeed = minSpeed;
      RightSpeed = maxSpeed;
    }
    //좌측 센서만 선 감지 시 우측 치우침
    else if ((left_sensor == LOW) && (right_sensor == HIGH)) {
      skew = 1;
      LeftSpeed = maxSpeed;
      RightSpeed = minSpeed;
    }

    /*
    //치우침 없을 시 직진
    if (skew == 0) {
      LeftSpeed = maxSpeed;
      RightSpeed = maxSpeed;
    }
    //좌측 치우침 시 좌회전
    else if (skew == 1) {
      LeftSpeed = minSpeed;
      RightSpeed = maxSpeed;
    }
    //우측 치우침 시 우회전
    else if (skew == -1) {
      LeftSpeed = maxSpeed;
      RightSpeed = minSpeed;
    }*/

    //모터 동작
    MotorAction(LeftSpeed * L_Polarity, RightSpeed * R_Polarity);

    //모드 출력
    lcd.setCursor(0, 0);
    lcd.print("M:on/off");
    //센서 상태 출력
    lcd.setCursor(11, 0);
    lcd.print("S:" + String(right_sensor) + "/" + String(left_sensor));
    //Serial.print("LS: " + String(left_sensor) + " RS: " + String(right_sensor) + "\n");
    //모터 조향 출력
    String LeftSpeedDisplay = "";
    String RightSpeedDisplay = "";
    for (int i = 4; i > String(LeftSpeed).length(); i--) {
      LeftSpeedDisplay += " ";
    }
    LeftSpeedDisplay += String(LeftSpeed);
    for (int i = 4; i > String(RightSpeed).length(); i--) {
      RightSpeedDisplay += " ";
    }
    RightSpeedDisplay += String(RightSpeed);
    if (skew == 0) {
      lcd.setCursor(0, 1);
      lcd.print("[R:" + String(RightSpeedDisplay) + " ");
      lcd.setCursor(8, 1);
      lcd.print(" L:" + String(LeftSpeedDisplay) + "]");
    } else if (skew == -1) {
      lcd.setCursor(0, 1);
      lcd.print(" R:" + String(RightSpeedDisplay) + " ");
      lcd.setCursor(8, 1);
      lcd.print("[L:" + String(LeftSpeedDisplay) + "]");
    } else if (skew == 1) {
      lcd.setCursor(0, 1);
      lcd.print("[R:" + String(RightSpeedDisplay) + "]");
      lcd.setCursor(8, 1);
      lcd.print(" L:" + String(LeftSpeedDisplay) + " ");
    }
    //Serial.print("LM: " + String(LeftSpeedDisplay) + " RM: " + String(RightSpeedDisplay) + "\n");
  }



  //2번 P 제어 주행
  else if (mode == 2) {
    //2번 버튼: 좌 우 모터 선택

    /*
    if ((Button2 == LOW) && (Button2_Flag)) {
      Button2_Count++;
      //변환할 모터 속도를 불러오기
      if (Button2_Count % 2 == 0) {
        ConvertMotor = RightSpeed;
      } else {
        ConvertMotor = LeftSpeed;
      }
      Button2_Flag = false;
    } else if (Button2 == HIGH) {
      //변환한 모터 속도를 적용
      if (Button2_Count % 2 == 0) {
        RightSpeed = ConvertMotor;
      } else {
        LeftSpeed = ConvertMotor;
      }
      Button2_Flag = true;
    }

    //1번 버튼: 속도 증가
    if (Button1 == LOW) {
      if (Button1_Flag) {
        Button1_press_start = millis();
        Button1_Count++;
        ConvertMotor++;
      }  //꾹 누르면 빠르게 증가
      else if (millis() - Button1_press_start >= Button1_count_interval) {
        ConvertMotor += min(2, (millis() - Button1_press_start - Button1_count_interval) * 0.01);
      }
      Button1_Flag = false;
    } else if (Button1 == HIGH) {
      Button1_Flag = true;
      Button1_press_start = 0;
    }
    //3번 버튼: 속도 감소
    if (Button3 == LOW) {
      if (Button3_Flag) {
        Button3_press_start = millis();
        Button3_Count++;
        ConvertMotor--;
      }  //꾹 누르면 빠르게 감소
      else if (millis() - Button3_press_start >= Button3_count_interval) {
        ConvertMotor -= min(2, (millis() - Button3_press_start - Button3_count_interval) * 0.01);
      }
      Button3_Flag = false;
    } else if (Button3 == HIGH) {
      Button3_Flag = true;
      Button3_press_start = 0;
    }

    //속도 제한
    ConvertMotor = max(MinMotorOutput, min(MaxMotorOutput, ConvertMotor));
    Diffential4Staright = max(0, min(1, (LeftSpeed / RightSpeed)));

    //모터 동작
    MotorAction(LeftSpeed, RightSpeed);

    //가장 많이 누른 버튼의 색으로 LED 점등
    if (Button1_Count > max(Button2_Count, Button3_Count)) {
      LED_SetRed();
    } else if (Button2_Count > max(Button1_Count, Button3_Count)) {
      LED_SetGreen();
    } else if (Button3_Count > max(Button1_Count, Button2_Count)) {
      LED_SetBlue();
    } else if ((Button1_Count == max(Button2_Count, Button3_Count)) || (Button2_Count == max(Button1_Count, Button3_Count)) || (Button3_Count == max(Button1_Count, Button2_Count))) {
      LED_SetWhite();
    }

    //모드 출력
    lcd.setCursor(0, 0);
    lcd.print("M:test");
    //버튼 상태 출력
    //lcd.setCursor(7, 0);
    //lcd.print("B:" + String(Button1) + String(Button2) + String(Button3));
    //버튼 카운트 출력
    //Serial.print("Button_1: " + String(Button1_Count) + " Button_2: " + String(Button2_Count) + " Button_3: " + String(Button3_Count) + "\n");
    //센서 상태 출력
    lcd.setCursor(11, 0);
    lcd.print("S:" + String(right_sensor) + "/" + String(left_sensor));
    //Serial.print("LS: " + String(left_sensor) + " RS: " + String(right_sensor) + "\n");
    //모터 조향 출력
    String LeftSpeedDisplay = "";
    String RightSpeedDisplay = "";
    for (int i = 3; i > String(LeftSpeed).length(); i--) {
      LeftSpeedDisplay += " ";
    }
    LeftSpeedDisplay += String(LeftSpeed);
    for (int i = 3; i > String(RightSpeed).length(); i--) {
      RightSpeedDisplay += " ";
    }
    RightSpeedDisplay += String(RightSpeed);
    if (Button2_Count % 2 == 0) {
      lcd.setCursor(0, 1);
      lcd.print("[R:" + String(RightSpeedDisplay) + "]");
      lcd.setCursor(9, 1);
      lcd.print(" L:" + String(LeftSpeedDisplay) + " ");
    } else {
      lcd.setCursor(0, 1);
      lcd.print(" R:" + String(RightSpeedDisplay) + " ");
      lcd.setCursor(9, 1);
      lcd.print("[L:" + String(LeftSpeedDisplay) + "]");
    }
    //Serial.print("LM: " + String(LeftSpeedDisplay) + " RM: " + String(RightSpeedDisplay) + "\n");*/









    //1번 버튼: kp 증가
    if ((Button1 == LOW) && (Button1_Flag)) {
      Button1_press_start = millis();
      Button1_Count++;
      Button1_Flag = false;
    } else if ((Button1 == HIGH) || (millis() - Button1_press_start >= Button1_count_interval)) {
      Button1_Flag = true;
    }
    //3번 버튼: kp 감소
    if ((Button3 == LOW) && (Button3_Flag)) {
      Button3_press_start = millis();
      Button3_Count++;
      Button3_Flag = false;
    } else if ((Button3 == HIGH) || (millis() - Button3_press_start >= Button3_count_interval)) {
      Button3_Flag = true;
    }

    //kp 최소 최대 제한
    kp = max(0.01, min(1, (Button1_Count - Button3_Count) * 0.01));

    kd = 0.01;
    ki = 0.05;


    //양측 센서 모두 선 감지 시
    if ((left_sensor == HIGH) && (right_sensor == HIGH)) {
      Left_error_start_time = millis();
      Right_error_start_time = millis();
      error_integration = 0;
      skew = 0;
    } else if ((left_sensor == LOW) && (right_sensor == LOW)) {
      if (Left_error_start_time <= 0) {
        Left_error_start_time = millis();
      }
      if (Right_error_start_time <= 0) {
        Right_error_start_time = millis();
      }
    }
    //우측 센서 선 감지 시 좌측 치우침
    else if ((left_sensor == LOW) && (right_sensor == HIGH)) {
      if (skew != -1) {
        skew = -1;
        Right_error_start_time = 0;
        Left_error_start_time = millis();
      }
    }
    //좌측 센서 선 감지 시 우측 치우침
    else if ((left_sensor == HIGH) && (right_sensor == LOW)) {
      if (skew != 1) {
        skew = 1;
        Right_error_start_time = millis();
        Left_error_start_time = 0;
      }
    }

    //시간 에러와 에러 방향 구하기
    if (Right_error_start_time != 0 && Left_error_start_time != 0) {
      error = ((millis() - Right_error_start_time) + (millis() - Left_error_start_time)) * skew;
    } else if (Right_error_start_time != 0) {
      error = ((millis() - Right_error_start_time)) * skew;
    } else if (Left_error_start_time != 0) {
      error = ((millis() - Left_error_start_time)) * skew;
    }
    error_integration += error;
    error_Differention = abs((abs(error) - abs(pre_error)) * 0.5 + abs(error_Differention) * 0.5) * skew;
    float gain = (error * kp) + (error_integration * ki) + (error_Differention * kd);

    Serial.println("P: " + String(error) + " I: " + String(error_integration) + " D: " + String(error_Differention));

    //모터 속도 계산
    LeftSpeed = (int)(MaxMotorOutput + gain);
    RightSpeed = (int)(MaxMotorOutput - gain);
    LeftSpeed = max(MinMotorOutput, min(MaxMotorOutput, LeftSpeed));
    RightSpeed = max(MinMotorOutput, min(MaxMotorOutput, RightSpeed));

    //모터 동작
    MotorAction(LeftSpeed, RightSpeed);

    //모드와 Proportional gain 출력(1/100까지)
    String kpDisplay = String(kp);
    for (int i = 4; i > String(kp).length(); i--) {
      kpDisplay += " ";
    }
    lcd.setCursor(0, 0);
    lcd.print("M:P_" + String(kpDisplay));

    //센서 상태 출력
    lcd.setCursor(10, 0);
    lcd.print("S:" + String(right_sensor) + "/" + String(left_sensor));
    //Serial.print("LS: " + String(left_sensor) + " RS: " + String(right_sensor) + "\n");

    //모터 조향 출력
    String LeftSpeedDisplay = "";
    String RightSpeedDisplay = "";
    for (int i = 3; i > String(LeftSpeed).length(); i--) {
      LeftSpeedDisplay += " ";
    }
    LeftSpeedDisplay += String(LeftSpeed);
    for (int i = 3; i > String(RightSpeed).length(); i--) {
      RightSpeedDisplay += " ";
    }
    RightSpeedDisplay += String(RightSpeed);
    if (skew == 0) {
      lcd.setCursor(8, 1);
      lcd.print(" L:" + String(LeftSpeedDisplay));
      lcd.setCursor(0, 1);
      lcd.print(" R:" + String(RightSpeedDisplay));
    } else if (skew == -1) {
      lcd.setCursor(8, 1);
      lcd.print("[L:" + String(LeftSpeedDisplay) + "]");
      lcd.setCursor(0, 1);
      lcd.print(" R:" + String(RightSpeedDisplay) + " ");
    } else if (skew == 1) {
      lcd.setCursor(8, 1);
      lcd.print(" L:" + String(LeftSpeedDisplay) + " ");
      lcd.setCursor(0, 1);
      lcd.print("[R:" + String(RightSpeedDisplay) + "]");
    }
    //Serial.print("LM: " + String(LeftSpeedDisplay) + " RM: " + String(RightSpeedDisplay) + "\n");

    pre_error = error;
  }
}


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
  analogWrite(LED_R, max(1, (int)(MaxLEDBright / 3)));
  analogWrite(LED_G, max(1, (int)(MaxLEDBright / 3)));
  analogWrite(LED_B, max(1, (int)(MaxLEDBright / 3)));
}

void MotorAction(int LeftSpeed, int RightSpeed) {
  LeftSpeed = min(255, max(-255, LeftSpeed));
  if (LeftSpeed > 0) {
    digitalWrite(MotorPin_LF, HIGH);
    digitalWrite(MotorPin_LB, LOW);
    analogWrite(LeftMotor, LeftSpeed);
  } else if (LeftSpeed < 0) {
    digitalWrite(MotorPin_LF, LOW);
    digitalWrite(MotorPin_LB, HIGH);
    analogWrite(LeftMotor, abs(LeftSpeed));
  } else {
    digitalWrite(MotorPin_LF, LOW);
    digitalWrite(MotorPin_LB, LOW);
    analogWrite(LeftMotor, 0);
  }

  RightSpeed = min(255, max(-255, RightSpeed));
  if (RightSpeed > 0) {
    digitalWrite(MotorPin_RF, HIGH);
    digitalWrite(MotorPin_RB, LOW);
    analogWrite(RightMotor, RightSpeed);
  } else if (RightSpeed < 0) {
    digitalWrite(MotorPin_RF, LOW);
    digitalWrite(MotorPin_RB, HIGH);
    analogWrite(RightMotor, abs(RightSpeed));
  } else {
    digitalWrite(MotorPin_RF, LOW);
    digitalWrite(MotorPin_RB, LOW);
    analogWrite(RightMotor, 0);
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

///////////
/*
    digitalWrite(MotorPin_RF, HIGH);
    digitalWrite(MotorPin_RB, LOW);
    digitalWrite(MotorPin_LF, HIGH);
    digitalWrite(MotorPin_LB, LOW);
    if (left_sensor == LOW) {
      analogWrite(RightMotor, abs(RightSpeed));
    } else {
      analogWrite(RightMotor, abs(0));
    }
    if (right_sensor == LOW) {
      analogWrite(LeftMotor, abs(LeftSpeed));
    } else {
      analogWrite(LeftMotor, abs(0));
    }
*/
/*
    if ((left_sensor == LOW) && (right_sensor == LOW)) {
      analogWrite(LeftMotor, LeftSpeed);
      analogWrite(RightMotor, RightSpeed);
    } else if ((left_sensor == HIGH) && (right_sensor == LOW)) {
      analogWrite(LeftMotor, LeftSpeed);
      analogWrite(RightMotor, 0);
    } else if ((left_sensor == LOW) && (right_sensor == HIGH)) {
      analogWrite(LeftMotor, 0);
      analogWrite(RightMotor, RightSpeed);
    }
    */
