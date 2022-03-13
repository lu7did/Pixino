
/*
 Pixino Project
 Hardware diagnostic firmware
 Test Switches and rotary enconder
 Copyright Pedro E Colla (c) LU7DZ 2022

*/
/*
 * Program define
 */

#define ROT_A   6         //PD6    (pin 12)
#define ROT_B   7         //PD7    (pin 13)
#define BUTTONS 17        //PC3/A3 (pin 26)
#define _digitalRead(x) digitalRead(x)


/*-------------------------------------------------------------------------------------------------
 * Liquid Crystal Library
 * initialize the library by associating any needed LCD interface pin
 * with the arduino pin number it is connected to
 */ 
#include <LiquidCrystal.h>
const int rs=A4, en=4, d4=0, d5=1, d6=2, d7=3;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

/*
 * Program variables
 */

volatile int8_t encoder_val = 0;
static uint8_t last_state;
bool encoder_move=false;
bool flagFirst=true;


/*
 * Interrupt to service encoder
 */
ISR(PCINT2_vect){  // Interrupt on rotary encoder turn
  switch(last_state = (last_state << 4) | (_digitalRead(ROT_B) << 1) | _digitalRead(ROT_A)){ //transition  (see: https://www.allaboutcircuits.com/projects/how-to-use-a-rotary-encoder-in-a-mcu-based-project/  )
    case 0x23:  encoder_val++; encoder_move=true; break;
    case 0x32:  encoder_val--; encoder_move=true; break;
  }
}

/*
 * setup of encoder
 */
void encoder_setup()
{
  pinMode(ROT_A, INPUT_PULLUP);
  pinMode(ROT_B, INPUT_PULLUP);
  PCMSK2 |= (1 << PCINT22) | (1 << PCINT23); // interrupt-enable for ROT_A, ROT_B pin changes; see https://github.com/EnviroDIY/Arduino-SDI-12/wiki/2b.-Overview-of-Interrupts
  PCICR |= (1 << PCIE2); 
  last_state = (_digitalRead(ROT_B) << 1) | _digitalRead(ROT_A);
  interrupts();
}
/*
 * setup
 * Standard Arduino initialization
 */


void setup() {

/*
 * Disable external interrupts INT0,INT1 and Pin Change
 */
  PCICR = 0;
  PCMSK0 = 0;
  PCMSK1 = 0;
  PCMSK2 = 0;

/*
 *  Enable BUTTON PIn Change Interrupt
 */
 /* 
   
  *digitalPinToPCMSK(BUTTONS) |= (1<<digitalPinToPCMSKbit(BUTTONS));
  *digitalPinToPCICR(BUTTONS) |= (1<<digitalPinToPCICRbit(BUTTONS));
  
 */
  pinMode(BUTTONS, INPUT);  // L/R/rotary button
  encoder_setup();

/*  
 *   Initialize an clear the display
 */
  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Pixino Test");
  delay(3000);
}

void loop() {
  if (flagFirst == true) {

  lcd.print("LCD Write");
  lcd.setCursor(0,0);
  lcd.print("0123456789012345");
  lcd.setCursor(0,1);
  lcd.print("0123456789012345");
  delay(5000);
/*  
 *   Write print from 0 to 9, left to right
 */
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Direct");
  lcd.setCursor(0, 1);
  for (int thisChar = 0; thisChar < 10; thisChar++) {
    lcd.print(thisChar);
    delay(500);
  }
  delay(5000);
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Reverse");
  lcd.setCursor(16, 1);
  lcd.autoscroll();
  for (int thisChar = 0; thisChar < 10; thisChar++) {
    lcd.print(thisChar);
    delay(500);
  }
  delay(5000);
  lcd.noAutoscroll();
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Cursor Blink>");
  lcd.noBlink();
  delay(3000);
  // Turn on the blinking cursor:
  lcd.blink();
  delay(3000);

  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Cursor hide>");
  lcd.noCursor();
  delay(1000);
  lcd.cursor();
  delay(1000);

/*  
 *   Buttons test
 */

  // Print a message to the LCD.
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Encoder/Button");
  lcd.setCursor(0,1);  
  flagFirst=false;

  /*  
 *   Disable internal interrupts
 */
 
  TIMSK0 = 0;
  TIMSK1 = 0;
  TIMSK2 = 0;
  WDTCSR = 0;

    
  }

  if (encoder_move == true) {
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Encoder detected");
    lcd.setCursor(0,1);
    lcd.print(encoder_val);
    encoder_move=false;
  }
  
   uint16_t v = analogRead(BUTTONS);
   if (v > 500) {
      delay(10); //debounce
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Button detected");
      lcd.setCursor(0,1);
      float V=(v*1.0/1023.0)*5.0;
      char buffer[32];
      char szBuffer[32];
      dtostrf( V, 4, 2, szBuffer );
      sprintf(buffer, "v=%d V=%s", v,szBuffer);
      lcd.print(buffer);
      //event |= (v < (4.2 * 1024.0 / 5.0)) ? BL : (v < (4.8 * 1024.0 / 5.0)) ? BR : BE; // determine which button pressed based on threshold levels
      while (analogRead(BUTTONS) >500) { }
      lcd.clear();
   }
  

}
