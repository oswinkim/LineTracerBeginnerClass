#include <LiquidCrystal_I2C.h>
#include <Wire.h>

int RED = 11;
int GREEN = 10;
int BLUE = 9;    // LED(주빨갈)

int b1 = 8;
int b2 = 12;
int b3 = 13;    // 버튼(LED에서 가까운 순서, 1~3번, 노초파)

int light = 5;

int state = 0;
int state2 = 0;

int R = 0;
int G = 0;
int B = 0;

void setup() {
    pinMode(RED, OUTPUT);
    pinMode(GREEN, OUTPUT);
    pinMode(BLUE, OUTPUT);

    pinMode(b1, INPUT_PULLUP);
    pinMode(b2, INPUT_PULLUP);
    pinMode(b3, INPUT_PULLUP);

    
    lcd.init();    // I2C LCD 초기화
    lcd.backlight();    // I2C LCD 백라이트 on
    lcd.setCursor(0, 0);
    lcd.print("    waiting    ");
}

void red() {
    analogWrite(RED, light);    // RED ON
    analogWrite(GREEN, 0);
    analogWrite(BLUE, 0);
}

void green() {
    analogWrite(RED, 0);
    analogWrite(GREEN, light);    // GREEN ON
    analogWrite(BLUE, 0);
}

void blue() {
    analogWrite(RED, 0);
    analogWrite(GREEN, 0);
    analogWrite(BLUE, light);    // BLUE ON
}
void loop() {


    int b1_state = digitalRead(b1);
    int b2_state = digitalRead(b2);
    int b3_state = digitalRead(b3);

    if (b1_state == 0) {    // 1번 버튼 눌림
        if (state2>0)state2--;
        delay(200);
        lcd.clear();
    }

    if (b2_state == 0) {    // 2번 버튼 눌림
        state++;
        state2=0;
        lcd.clear();
        delay(500);
    }


    if (b3_state == 0) {    // 3번 버튼 눌림림
        if (state2<255)state2++;
        delay(200);

        lcd.clear();
    }

    if (state % 3 == 0){
      lcd.setCursor(0, 0);
      lcd.print("[R]     G       B ");
      R = state2;
    }

    elif (state % 3 == 1){
      lcd.setCursor(0, 0);
      lcd.print(" R     [G]      B ");
      G = state2;
    }

    elif (state % 3 == 2){
      lcd.setCursor(0, 0);
      lcd.print(" R      G      [B]");
      B = state2;
    }

    lcd.setCursor(0,1);
    lcd.print(" ");
    lcd.print(R);
    lcd.print("      ");
    lcd.print(G);
    lcd.print("      ");
    lcd.print(B);

//이제부터 출력부분
/*
    analogWrite(RED, R);
    analogWrite(GREEN, G);
    analogWrite(BLUE, B);
*/

    if(R>G || R>B){
      red();
    }

    elif(G>R || G>B){
      green();
    }

    elif(B>R || B>G){
      blue();
    }


}
