// Feb 2016 - Graphics Test
//=======Libraries===============================
#include <Encoder.h>
#include "FastLED.h"
#include "RunningAverage.h"

//=======Hardware Setup===============================
//LEDs
#define NUM_LEDS 13
#define LED_PIN 17 //5V buffered pin on Teensy LC, single wire data out to WS8212Bs
#define LED_TYPE    WS2812B
#define COLOR_ORDER GRB
#define BRIGHTNESS 80 //Subject to change, limits current that the LEDs draw

//Encoder
#define REDPIN   5
#define GREENPIN 4
#define BLUEPIN  3
#define ENC_SW   6 //Pushbutton on the encoder
Encoder myEnc(8, 7); //Quadrature inputs on pins 7,8

//DIP Switches
#define SW1PIN 12 //Dip switch pins, for setting software options without reflashing code
#define SW2PIN 11
#define SW3PIN 10
#define SW4PIN 9

//Motor
#define MOTPIN 23

//Pressure Sensor Analog In
#define BUTTPIN 15

//=======Software/Timing options=====================
#define FREQUENCY 60 //Update frequency in Hz
#define LONG_PRESS_MS 2000

//Update/render period
#define period (1000/FREQUENCY)
#define longBtnCount (LONG_PRESS_MS / period)

//Running pressure average length and update frequency
#define RA_HIST_SECONDS 25
#define RA_FREQUENCY 6
#define RA_TICK_PERIOD (FREQUENCY / RA_FREQUENCY)
RunningAverage raPressure(RA_FREQUENCY*RA_HIST_SECONDS);

//=======State Machine Modes=========================
#define MANUAL      1
#define AUTO        2
#define OPT_SPEED   3
#define OPT_RAMPSPD 4
#define OPT_BEEP    5
#define OPT_PRES    6
#define SAVE        7
#define RESET       8

//=======Global Variables=============================
//DIP switch options:
bool SERIAL_EN =  false;
bool SW2 =        false;
bool SW3 =        false;
bool SW4 =        false;

CRGB leds[NUM_LEDS];
int cursorLed = 0;

int oldEnc = 0; //previously observed encoder position
int newEnc = 0;
int encPos = 0;
bool btnPress = 0;
int longPress = 0;
int pressure = 0;
int avgPressure = 0;
int bri =100;
const int pOffset = analogRead(BUTTPIN);
int rampTimeS = 30;
int pLimit = 300;
//=======Setup=======================================
void setup() {
  pinMode(REDPIN,   OUTPUT);
  pinMode(GREENPIN, OUTPUT);
  pinMode(BLUEPIN,  OUTPUT);
  pinMode(ENC_SW,   INPUT);

  //Set DIP switch pins as inputs
  pinMode(SW1PIN,   INPUT);
  pinMode(SW2PIN,   INPUT);
  pinMode(SW3PIN,   INPUT);
  pinMode(SW4PIN,   INPUT);

  //Enable pullup resistors on DIP pins. They are tied to GND if enabled.
  digitalWrite(SW1PIN, HIGH);
  digitalWrite(SW2PIN, HIGH);
  digitalWrite(SW3PIN, HIGH);
  digitalWrite(SW4PIN, HIGH);

  pinMode(MOTPIN,   OUTPUT); //Might want uh, analog out? PWM at some frequency.
  pinMode(BUTTPIN,  INPUT);
  raPressure.clear(); //Initialize a running pressure average
  //Make sure the motor is off
  digitalWrite(MOTPIN, LOW);

  delay(3000); // 3 second delay for recovery

  //If a pin reads low, the switch is enabled. Here, we read in the DIP settings
  SERIAL_EN = (digitalRead(SW1PIN) == LOW);
  SW2 = (digitalRead(SW2PIN) == LOW);
  SW3 = (digitalRead(SW3PIN) == LOW);
  SW4 = (digitalRead(SW4PIN) == LOW);

  if (SERIAL_EN) {
    Serial.begin(115200);
  }

  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
  // limit my draw to .6A at 5v of power draw
  //FastLED.setMaxPowerInVoltsAndMilliamps(5,600);
  FastLED.setBrightness(BRIGHTNESS);
}

//=======LED Drawing Functions=================


//Fade all LEDs to 200/255 their previous value
void fadeall() {
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i].nscale8(245);
  }
}

void showAnalogRGB(const CRGB& rgb)
{
  analogWrite(REDPIN,   rgb.r );
  analogWrite(GREENPIN, rgb.g );
  analogWrite(BLUEPIN,  rgb.b );
}

void draw_cursor(int pos,CRGB C1, CRGB C2, CRGB C3){
  pos = constrain(pos,1,NUM_LEDS*3);
  int colorNum = pos/NUM_LEDS;
  int cursorPos = pos % NUM_LEDS;
  switch(colorNum){
    case 0:
      leds[cursorPos] = C1;
      break;
    case 1:
      leds[cursorPos] = C2;
      break;
    case 2:
      leds[cursorPos] = C3;
      break;
  }
}

void draw_bars(int pos,CRGB C1, CRGB C2, CRGB C3){
  pos = constrain(pos,1,NUM_LEDS*3);
  int colorNum = pos/NUM_LEDS;
  int barPos = pos % NUM_LEDS;
  switch(colorNum){
    case 0:
      fill_solid(leds, NUM_LEDS, CHSV(0,0,0)); //black background
      fill_solid(leds, barPos, C1);
      break;
    case 1:
      fill_solid(leds, NUM_LEDS, C1);
      fill_solid(leds, barPos, C2);
      break;
    case 2:
      fill_solid(leds, NUM_LEDS, C2);
      fill_solid(leds, barPos, C3);
      break;
  }
}


//=======Main Loop=============================
void loop() {
  //Run everything at the update frequency
  if (millis() % period == 0) {
    //Serial.print(state);
    delay(1);
    int pos = myEnc.read();
    CRGB red = CRGB::Red;
    CRGB green = CRGB::Green;
    CRGB yellow = CRGB::Yellow;

    
    draw_cursor(pos,green,yellow,red);
    FastLED.show();
  }







}
