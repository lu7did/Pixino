/*
Si5351 VFO

By LA3PNA  27 March 2015
Modified by NT7S  25 April 2015

Modified to be Si5351 Arduino v2 compliant by NT7S  21 Nov 2016

This version uses the new version of the Si5351 library from NT7S.
see: http://arduino.cc/en/Reference/AttachInterrupt for what pins that have interrupts.

UNO and 328 boards: Encoder on pin 2 and 3. Center pin to GND.
Leonardo: Encoder on pin 0 and 1. Center pin to GND.

100nF from each of the encoder pins to gnd is used to debounce

The pushbutton goes to pin 11 to set the tuning rate.
Pin 12 is the RX/TX pin. Put this pin LOW for RX, open or high for TX.
Single transistor switch to +RX will work.

VFO will NOT tune in TX.

LCD connections for for the LinkSprite 16 X 2 LCD Keypad Shield for Arduino.
Change as necessary for your LCD.
 
IF frequency is positive for sum product (IF = RF + LO) and negative for diff (IF = RF - LO)
VFO signal output on CLK0, BFO signal on CLK2

*/

// Only leave one uncommented for the display you wish to use
#define OLED
//#define LCD

#include <si5351.h>
#include "Wire.h"

// Conditional includes based on which display is defined above
#if defined(LCD)
  #include <LiquidCrystal.h>
  LiquidCrystal lcd( 8, 9, 4, 5, 6, 7 );
#endif
#if defined(OLED)
  #include "U8glib.h"
  U8GLIB_SSD1306_128X64_2X u8g(U8G_I2C_OPT_NONE);	// I2C / TWI
#endif

// Class instantiation
Si5351 si5351;

// interrupt service routine vars
boolean A_set = false;
boolean B_set = false;
volatile unsigned long frequency = 7100000UL; // This will be the frequency it always starts on.
volatile int tx;

unsigned long iffreq = 0; // set the IF frequency in Hz.
const unsigned long freqstep[] = {50, 100, 500, 1000, 5000, 10000}; // set this to your wanted tuning rate in Hz.
int corr = 0; // this is the correction factor for the Si5351, use calibration sketch to find value.
unsigned int lastReportedPos = 1;   // change management
static boolean rotating = false;    // debounce management
int inData;
int txpin = 12;
int freqsteps = 1;
int stepbutton = 11;
#define arraylength       (sizeof(freqstep) / sizeof(freqstep[0]))

// Define hardware pins based on platform
#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega16U4__)
  int encoderPinA = 0;   // right
  int encoderPinB = 1;   // left
#endif
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
  int encoderPinA = 2;   // right
  int encoderPinB = 3;   // left
#endif

// Interrupt on A changing state
void doEncoderA()
{
  // debounce
  if (rotating) delay (1);  // wait a little until the bouncing is done
  // Test transition, did things really change?
  if (digitalRead(encoderPinA) != A_set) { // debounce once more
    A_set = !A_set;
    // adjust counter + if A leads B
    if (A_set && !B_set) {
      if (!tx) {
        frequency += freqstep[freqsteps]; // hehre is the amount to increase the freq
      }
      rotating = false;  // no more debouncing until loop() hits again
    }
  }
}

// Interrupt on B changing state, same as A above
void doEncoderB()
{
  if (rotating) delay (1);
  if (digitalRead(encoderPinB) != B_set) {
    B_set = !B_set;
    //  adjust counter - 1 if B leads A
    if (B_set && !A_set) {
      if (!tx) {
        frequency -= freqstep[freqsteps]; // here is the amount to decrease the freq
      }
      rotating = false;
    }
  }
}


void sprintf_seperated(char *str, unsigned long num)
{
  // We will print out the frequency as a fixed length string and pad if less than 100s of MHz
  char temp_str[6];
  int zero_pad = 0;
  
  // MHz
  if(num / 1000000UL > 0)
  {
    sprintf(str, "%3lu", num / 1000000UL);
    zero_pad = 1;
  }
  else
  {
    strcat(str, "   ");
  }
  num %= 1000000UL;
  
  // kHz
  if(zero_pad == 1)
  {
    sprintf(temp_str, ",%03lu", num / 1000UL);
    strcat(str, temp_str);
  }
  else if(num / 1000UL > 0)
  {
    sprintf(temp_str, ",%3lu", num / 1000UL);
    strcat(str, temp_str);
    zero_pad = 1;
  }
  else
  {
    strcat(str, "   ");
  }
  num %= 1000UL;
  
  // Hz
  if(zero_pad == 1)
  {
    sprintf(temp_str, ",%03lu", num);
    strcat(str, temp_str);
  }
  else
  {
    sprintf(temp_str, ",%3lu", num);
    strcat(str, temp_str);
  }
  
  strcat(str, " MHz");
}

#if defined(OLED)
  void draw_oled(void)
  {
    char temp_str[21];
    
    u8g.setFont(u8g_font_unifont);
    //u8g.setFont(u8g_font_helvR12);
    sprintf_seperated(temp_str, frequency);
    u8g.drawStr(0, 32, temp_str);
    
    u8g.setFont(u8g_font_unifont);
    sprintf(temp_str, "Step: %5u", freqstep[freqsteps]);
    u8g.drawStr(0, 56, temp_str);
  }
#endif

#if defined(LCD)
  void draw_lcd(void)
  {
    char temp_str[21];
    
    sprintf_seperated(temp_str, frequency);
    lcd.setCursor(0, 0);
    lcd.print(temp_str);
    
    lcd.setCursor(6, 1);
    sprintf(temp_str, "%5u", freqstep[freqsteps]);
    lcd.print(temp_str);
  }
#endif

void setup()
{
  Serial.begin(9600);
  
  // Set GPIO
  pinMode(encoderPinA, INPUT);
  pinMode(encoderPinB, INPUT);
  pinMode(stepbutton, INPUT);
  pinMode(txpin, INPUT);
  
  // Turn on pullup resistors
  digitalWrite(encoderPinA, HIGH);
  digitalWrite(encoderPinB, HIGH);
  digitalWrite(stepbutton, HIGH);
  digitalWrite(txpin, HIGH);
  
#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega16U4__)
  //Code in here will only be compiled if an Arduino Leonardo is used.
  // encoder pin on interrupt 0 (pin 0)
  attachInterrupt(0, doEncoderA, CHANGE);
  // encoder pin on interrupt 1 (pin 1)
  attachInterrupt(1, doEncoderB, CHANGE);
#endif
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
  //Code in here will only be compiled if an Arduino Uno (or older) is used.
  // encoder pin on interrupt 0 (pin 2)
  attachInterrupt(0, doEncoderA, CHANGE);
  // encoder pin on interrupt 1 (pin 3)
  attachInterrupt(1, doEncoderB, CHANGE);
#endif

// Initialize the display
#if defined(LCD)
  lcd.begin(16, 2);
  lcd.print("Si5351 VFO");
  
  delay(2000);
  lcd.clear();
  lcd.setCursor(0, 1);
  lcd.print("Step:           ");
#endif
#if defined(OLED)
  //U8GLIB_SSD1306_128X32 u8g(U8G_I2C_OPT_NONE);	// I2C / TWI
  
  // Assign default color value
  if (u8g.getMode() == U8G_MODE_R3G3B2)
  {
    u8g.setColorIndex(255);     // white
  }
  else if (u8g.getMode() == U8G_MODE_GRAY2BIT)
  {
    u8g.setColorIndex(3);         // max intensity
  }
  else if (u8g.getMode() == U8G_MODE_BW)
  {
    u8g.setColorIndex(1);         // pixel on
  }
  else if (u8g.getMode() == U8G_MODE_HICOLOR)
  {
    u8g.setHiColorByRGB(255,255,255);
  }
#endif
  
  si5351.init(SI5351_CRYSTAL_LOAD_8PF, 0, corr);
  si5351.set_freq((frequency + iffreq) * 100ULL, SI5351_CLK0);
  si5351.set_freq(iffreq * 100ULL, SI5351_CLK2);
}

void loop()
{
  if(digitalRead(txpin))
  {
    tx = 0;
  }
  else
  {
    tx = 1;
  }
  
  rotating = true;  // reset the debouncer
  
  if (lastReportedPos != frequency)
  {
    lastReportedPos = frequency;
    
    // Handle LCD
    #if defined(LCD)
      draw_lcd();
    #endif
    
    si5351.set_freq((frequency + iffreq) * 100ULL, SI5351_CLK0);
  }
  
  // Handle OLED
  #if defined(OLED)
    u8g.firstPage();  
    do
    {
      draw_oled();
    } while(u8g.nextPage());
    delay(50);
  #endif
  
  if (Serial.available() > 0)   // see if incoming serial data:
  {
    inData = Serial.read();  // read oldest byte in serial buffer:
  }
  
  if (inData == 'F')
  {
    frequency = Serial.parseInt();
    inData = 0;
  }
  
  if (digitalRead(stepbutton) == LOW )
  {
    delay(50);   // delay to debounce
    if (digitalRead(stepbutton) == LOW )
    {
      freqsteps = freqsteps + 1;

      if (freqsteps > arraylength - 1 )
      {
        freqsteps = 0;
      }
      delay(50); //delay to avoid many steps at one
    }
  }
}