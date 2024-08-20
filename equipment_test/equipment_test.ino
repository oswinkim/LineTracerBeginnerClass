#include <LiquidCrystal_I2C.h>
#include <Wire.h>

#define IN1 2
#define IN2 4
#define IN3 6
#define IN4 7

#define speed_left 3   // 모터 속도 제어 왼쪽
#define speed_right 5  // 모터 속도 제어 오른쪽
LiquidCrystal_I2C lcd(0x27, 16, 2);


int RED = 11;
int GREEN = 10;
int BLUE = 9;

int b1 = 8;
int b2 = 12;
int b3 = 13;

#define line_detector_left A1   // 적외선 센서 왼쪽
#define line_detector_right A0  // 적외선 센서 오른쪽


void forward() {
  int lspeed = 100;
  int rspeed = 200;

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  analogWrite(speed_left, lspeed);
  analogWrite(speed_right, rspeed);

  lcd.setCursor(0, 1);
  lcd.print("M:");  // 문구를 출력합니다.
  lcd.setCursor(5, 1);
  lcd.print("/");  // 문구를 출력합니다.

  lcd.setCursor(2, 1);
  lcd.print(lspeed);  // 문구를 출력합니다.
  lcd.setCursor(6, 1);
  lcd.print(rspeed);  // 문구를 출력합니다.
}

void setup() {
  lcd.init();
  lcd.backlight();
  Wire.begin();

  pinMode(IN1, OUTPUT);  // A 모터 1
  pinMode(IN2, OUTPUT);  // A 모터 2
  pinMode(IN3, OUTPUT);  // B 모터 1
  pinMode(IN4, OUTPUT);  // B 모터 2
  pinMode(speed_left, OUTPUT);
  pinMode(speed_right, OUTPUT);

  pinMode(b1, INPUT_PULLUP);
  pinMode(b2, INPUT_PULLUP);
  pinMode(b3, INPUT_PULLUP);

  pinMode(line_detector_left, INPUT);
  pinMode(line_detector_right, INPUT);

  Serial.begin(9600);
  while (!Serial) {};
  Serial.println("\n---I2C Scanner---");
  lcd.clear();

  byte error, address;
  int nDevices;
  Serial.println("Scanning...");
  nDevices = 0;
  for (address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("I2C adress:  0x");
      if (address < 16)
        Serial.print("0");
      Serial.print(address, HEX);
      Serial.print("\n");

      if (int(address) == 39) {
        LiquidCrystal_I2C lcd(0x27, 16, 2);
        lcd.backlight();
        lcd.setCursor(0, 0);
        lcd.print("I2C:0x27");  // 문구를 출력합니다.
      } else {
        LiquidCrystal_I2C lcd(0x3F, 16, 2);
        lcd.backlight();
        lcd.setCursor(0, 0);
        lcd.print("I2C:0x3F");  // 문구를 출력합니다.
      }
      nDevices++;
    }
  }
  lcd.setCursor(0, 1);
  lcd.print("run");  // 문구를 출력합니다.
  Serial.print("---setup done---\n");
}

void loop() {

  int button1 = digitalRead(b1);
  int button2 = digitalRead(b2);
  int button3 = digitalRead(b3);

  int left_sensor = digitalRead(line_detector_left);    // 적외선 센서 값(40~600)
  int right_sensor = digitalRead(line_detector_right);  // 적외선 센서 값(40~600)

  forward();
  /*
  Serial.print("b1: ");
  Serial.print(button1);
  Serial.print(" b2: ");
  Serial.print(button2);
  Serial.print(" b3: ");
  Serial.println(button3);
  */

  lcd.setCursor(11, 1);
  lcd.print("B:");  // 문구를 출력합니다.
  //lcd.setCursor(5, 1);
  //lcd.print("/");  // 문구를 출력합니다.
  lcd.setCursor(13, 1);
  lcd.print(button1);  // 문구를 출력합니다.
  lcd.setCursor(14, 1);
  lcd.print(button2);  // 문구를 출력합니다.
  lcd.setCursor(15, 1);
  lcd.print(button3);  // 문구를 출력합니다.

  lcd.setCursor(11, 0);
  lcd.print("S:");  // 문구를 출력합니다.
  //lcd.setCursor(5, 1);
  //lcd.print("/");  // 문구를 출력합니다.
  lcd.setCursor(13, 0);
  lcd.print(left_sensor);  // 문구를 출력합니다.
  lcd.setCursor(15, 0);
  lcd.print(right_sensor);  // 문구를 출력합니다.
}
