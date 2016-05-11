// Feb 2016 - nogasm
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

//=======Helper Functions=============================


//Fade all LEDs to 200/255 their previous value
void fadeall() {
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i].nscale8(245);
  }
}

void drawCursor(int value, int minVal, int maxVal) {

  cursorLed = constrain(encPos / 4, 0, NUM_LEDS - 1);
}

void showAnalogRGB(const CRGB& rgb)
{
  analogWrite(REDPIN,   rgb.r );
  analogWrite(GREENPIN, rgb.g );
  analogWrite(BLUEPIN,  rgb.b );
}

void gyrGraphDraw(int val, int minVal, int maxVal) {
  val = constrain(val, minVal, maxVal);
  int revSize = maxVal / 3;
  int barPos = constrain(map(val % revSize, 0, revSize, 0, NUM_LEDS), 0, NUM_LEDS - 1);
  //int nextPosInterpol = (revSize/NUM_LEDS)-(val % (revSize/NUM_LEDS));
  //int nextPosInterpol = map(val % (maxVal/(3*NUM_LEDS)),0,(maxVal/(3*NUM_LEDS)),25,150);
  int revsAlready = (3 * val) / maxVal;

  switch (revsAlready) {
    case 2:
      fill_solid(leds, NUM_LEDS, CHSV(45, 255, bri)); //Yellow Background
      fill_solid(leds, barPos, CHSV(0, 255, bri)); //Red Bar
      //showAnalogRGB(CHSV(45,255,bri)); //Make knob Yellow
      break;


    //Or, if val is > 1/3 max, make background green
    case 1:
      fill_solid(leds, NUM_LEDS, CHSV(85, 255, bri)); //Green background
      fill_solid(leds, barPos, CHSV(45, 255, bri)); //Yellow Bar
      //showAnalogRGB(CHSV(85,255,bri)); //Make knob Green
      break;

    //Otherwise, just a dim white backgorund
    case 0:
      fill_solid(leds, NUM_LEDS, CHSV(0, 0, bri)); //white back
      fill_solid(leds, barPos, CHSV(85, 255, bri)); //Green Bar
      //leds[barPos] = CHSV(85,255,nextPosInterpol); Tried to do fading, probably a bad idea
      //showAnalogRGB(CHSV(60,255,bri)); //Make knob White
      break;
    default:
      fill_solid(leds, NUM_LEDS, CHSV(45, 255, bri)); //Yellow Background
      fill_solid(leds, NUM_LEDS, CHSV(0, 255, bri)); //Red Bar
      //showAnalogRGB(CHSV(45,255,bri)); //Make knob Yellow
      break;
    
  }

}

void cursorDraw(int val, int maxVal, int revs) {
  int revSize = maxVal / revs;
  int cursorPos = constrain(map(val % revSize, 0, revSize, 0, NUM_LEDS), 0, NUM_LEDS - 1);
  //int nextPosInterpol = (revSize/NUM_LEDS)-(val % (revSize/NUM_LEDS));
  //int nextPosInterpol = map(val % (maxVal/(3*NUM_LEDS)),0,(maxVal/(3*NUM_LEDS)),25,150);
  int revsAlready = (revs * val) / maxVal;

  switch (revsAlready) {
    case 2:
      leds[cursorPos] = CRGB(220, 0, 255); //Purple
      showAnalogRGB(CRGB(220, 0, 255)); //Color the knob, too.
      break;


    //Or, if val is > 1/3 max, make background green
    case 1:
      leds[cursorPos] = CHSV(HUE_BLUE, 255, 255); //Blue
      showAnalogRGB(CHSV(HUE_BLUE, 255, 255)); //Make knob Green
      break;

    //Otherwise, just a dim white backgorund
    case 0:
      leds[cursorPos] = CHSV(HUE_AQUA, 200, 255); //light blue
      showAnalogRGB(CHSV(HUE_AQUA, 255, 255)); //Make knob White
      break;
  }

}


void colorBars()
{
  showAnalogRGB( CRGB::Red );   delay(2000);
  showAnalogRGB( CRGB::Green ); delay(2000);
  showAnalogRGB( CRGB::Blue );  delay(2000);
  showAnalogRGB( CRGB::Black ); delay(2000);
}
//=======Program Modes/States==================


void run_manual(int lastState) {
  //Serial.println("Manual");
  static int knobOffset = 0;

  if (lastState != MANUAL) {
    knobOffset = myEnc.read();
  }
  //In manual mode, only allow for 13 cursor positions, for adjusting motor speed.
  int knob = constrain((myEnc.read() - knobOffset), 0, 4 * (NUM_LEDS - 1));
  int motSpeed = map(knob, 0, 4 * NUM_LEDS, 0, 255);
  analogWrite(MOTPIN, motSpeed);

  //gyrGraphDraw(avgPressure, 0, 4 * 3 * NUM_LEDS);
  gyrGraphDraw(constrain(pressure - avgPressure, 0, pLimit), 0, pLimit);
  cursorDraw(knob, 4 * NUM_LEDS, 1);
  //Serial.print("pressure:");
  //Serial.println(pressure);
  //Serial.println(knob);

}

void run_auto(int lastState) {
  static int knobOffset = 0;
  static float motSpeed = 0;
  static float motIncrement = 0.0;
  //Serial.println("Auto");
  if (lastState != AUTO) {
    knobOffset = myEnc.read();
    motSpeed = 0.0;
    motIncrement = (255.0 / ((float)FREQUENCY * (float)rampTimeS));
  }

  int knob = constrain((myEnc.read() - knobOffset), 0, 4 * 3 * (NUM_LEDS - 1));
  //Reverse "Knob" to map it onto a pressure limit, so that it effectively adjusts sensitivity
  pLimit = map(knob, 0, 4 * 3 * (NUM_LEDS - 1), 300, 0); //300 is max sensitivity change
  //When someone clenches harder than the pressure limit
  if (pressure - avgPressure > pLimit) {
    motSpeed = -156.; //Stay off for a while (half the ramp up time)
    analogWrite(MOTPIN, (int) motSpeed);
  }
  else if (motSpeed < 255) {
    motSpeed += motIncrement;
    analogWrite(MOTPIN, (int) motSpeed);
  }
  Serial.println(motSpeed);


  gyrGraphDraw(constrain(pressure - avgPressure, 0, pLimit), 0, pLimit);
  cursorDraw(knob, 4 * 3 * NUM_LEDS, 3);

}

void run_opt_speed(int lastState) {
  Serial.println("speed");
}

void run_opt_rampspd(int lastState) {
  Serial.println("rampSpeed");
}

void run_opt_beep(int lastState) {
  Serial.println("Brightness");
}

void run_opt_pres(int lastState) {
  Serial.println("Brightness");
}

void run_save(int lastState) {
  Serial.println("Save");
}

void run_reset(int lastState) {
  Serial.println("Reset");
}

//=======Main Loop=============================
void loop() {
  static uint8_t hue = 0;
  //showAnalogRGB( CHSV( hue, 255, 255) );

  static int state = 1;
  static int lastState = 0;
  static int thisBtn = 0;
  static int lastBtn = 0;
  static int btnCount = 0;
  static int longBtnPress = 0;
  static int tick = 0;

  //Run everything at the update frequency
  if (millis() % period == 0) {
    //Serial.print(state);
    delay(1);
    pressure = analogRead(BUTTPIN) - pOffset;
    tick ++;
    if (tick % RA_TICK_PERIOD == 0) {
      raPressure.addValue(pressure);
      avgPressure = raPressure.getAverage();
    }
    thisBtn = digitalRead(ENC_SW);

    //Detect single presses, no repeating, on keyup
    if (thisBtn == LOW && lastBtn == HIGH) {
      btnPress = true;
    }
    else {
      btnPress = false;
    }

    //Detect long presses (as defined in timing section)
    if (thisBtn == HIGH) {
      btnCount ++;
    }
    lastBtn = thisBtn;

    //switch states when button is pressed
    if (btnPress) {
      switch (state) {
        case MANUAL:
          state = AUTO;
          break;
        case AUTO:
          state = MANUAL;
          break;
        case OPT_SPEED:
          state = OPT_RAMPSPD;
          break;
        case OPT_RAMPSPD:
          state = OPT_BEEP;
          break;
        case OPT_BEEP:
          state = OPT_PRES;
          break;
        case OPT_PRES:
          state = OPT_SPEED;
          break;
      }
      //Given that there's a keyup, was it a long press?
      if (btnCount >= longBtnCount) {
        switch (state) {
          case MANUAL:
            state = OPT_SPEED;
            break;
          case AUTO:
            state = OPT_SPEED;
            break;
          case OPT_SPEED:
            state = SAVE;
            break;
          case OPT_RAMPSPD:
            state = SAVE;
            break;
          case OPT_BEEP:
            state = SAVE;
            break;
          case OPT_PRES:
            state = SAVE;
            break;
        }
      }
      btnCount = 0;
    }

    //Run in the selected state
    switch (state) {
      case MANUAL:
        run_manual(lastState);
        break;
      case AUTO:
        run_auto(lastState);
        break;
      case OPT_SPEED:
        run_opt_speed(lastState);
        break;
      case OPT_RAMPSPD:
        run_opt_rampspd(lastState);
        break;
      case OPT_BEEP:
        run_opt_beep(lastState);
        break;
      case OPT_PRES:
        run_opt_pres(lastState);
        break;
      case SAVE:
        run_save(lastState);
        break;
      case RESET:
        run_reset(lastState);
        break;
    }
    lastState = state;
    FastLED.show();
  }




  //if(SERIAL_EN){Serial.print(SERIAL_EN);}//probably how to do prints *cringes*






}
