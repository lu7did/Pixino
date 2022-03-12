/*
 Pixino Project
 Test LCD 16x2 display, rotary encoder and buttons
 Copyright Pedro E Colla (c) LU7DZ 2022

*/

// include the library code:
#include <LiquidCrystal.h>

// initialize the library by associating any needed LCD interface pin
// with the arduino pin number it is connected to
const int rs=A4, en=4, d4=0, d5=1, d6=2, d7=3;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

void setup() {
  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);

  // Print a message to the LCD.
  lcd.setCursor(0,0);
  lcd.print("Pixino Test     ");
}

void loop() {
  delay(1000);
  lcd.setCursor(0,0);
  lcd.print("Timestamp test  ");
  // set the cursor to column 0, line 1
  lcd.setCursor(0, 1);
  // print the number of seconds since reset:
  lcd.print(millis() / 1000);
  delay(5000);
  
// set the cursor to (0,0):
  lcd.setCursor(0,0);
  lcd.print("Print test      ");
  lcd.setCursor(0, 1);
  // print from 0 to 9:
  for (int thisChar = 0; thisChar < 10; thisChar++) {
    lcd.print(thisChar);
    delay(500);
  }
  delay(5000);
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Scroll test     ");
  // set the cursor to (6,1):
  lcd.print("0123456789");
  lcd.clear();
  lcd.print("Autoscroll");
  lcd.setCursor(16, 1);
  // set the display to automatically scroll:
  lcd.autoscroll();
  // print from 0 to 9:
  for (int thisChar = 0; thisChar < 10; thisChar++) {
    lcd.print(thisChar);
    delay(500);
  }
  delay(5000);
  // turn off automatic scrolling
  lcd.noAutoscroll();
  // clear screen for the next loop:
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Cursor blink");
 // Turn off the blinking cursor:
  lcd.noBlink();
  delay(3000);
  // Turn on the blinking cursor:
  lcd.blink();
  delay(3000);

  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Cursor hide");
  // Turn off the cursor:
  lcd.noCursor();
  delay(500);
  // Turn on the cursor:
  lcd.cursor();
  delay(500);
}
