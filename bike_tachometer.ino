

#include <SPI.h>
#include <Wire.h>
//#include <EEPROM.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define EEPROM_DISTVAL_LOC 0
#define EEPROM_WRITE_INTERVAL  60

#define NUM_SPEED_AVG_SAMPLES 5


#define TIMER_COUNTER_DISTANCE_COMPUTE   1000  // eg: 10ms * <50> ==>500ms  0.5

#define DISPLAY_UPDATE_INTERVAL 2000
#define SPEED_UPDATE_INTERVAL 25
#define TOP_SPEED_HIT_DISPLAY_START 20

//#define SERIAL_TEST_ENABLED

// On an arduino UNO:       A4(SDA), A5(SCL)

#define OLED_RESET     4 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);



#define LOGO_HEIGHT   16
#define LOGO_WIDTH    16

static const unsigned char PROGMEM logo_bmp[] =
{ 0b00000000, 0b11000000,
  0b00000001, 0b11000000,
  0b00000001, 0b11000000,
  0b00000011, 0b11100000,
  0b11110011, 0b11100000,
  0b11111110, 0b11111000,
  0b01111110, 0b11111111,
  0b00110011, 0b10011111,
  0b00011111, 0b11111100,
  0b00001101, 0b01110000,
  0b00011011, 0b10100000,
  0b00111111, 0b11100000,
  0b00111111, 0b11110000,
  0b01111100, 0b11110000,
  0b01110000, 0b01110000,
  0b00000000, 0b00110000
};

const int interruptPin = 2;

volatile uint32_t speedTick = 0;
uint32_t oldSpeedTick = 0;
volatile uint32_t  debElapsed = 0;
volatile uint32_t startTimeCompute = 0;
volatile uint16_t oldTicks = 0;

//uint16_t currentSpeed = 0;
uint16_t oldSpeed = 0;
uint16_t avgSpeed = 0;
double distance = 0.0;
float displaySpeed = 0;
uint32_t startTimeDisp = 0;

volatile uint8_t sensrDebInt;
uint16_t topSpeed = 0;
boolean topSpeedFlag = false;
boolean updateDisplayFlag = false;

volatile uint8_t oldTimer2Ct1;

volatile byte timer2_counter1;
volatile byte timer2_counter2;
volatile byte timer2_counter3;

static void rotationInterruptHandler()
{

  sensrDebInt = timer2_counter1 - oldTimer2Ct1;
  if (sensrDebInt <5  && speedTick >= 1) {
    speedTick = oldTicks;
    oldTimer2Ct1=timer2_counter1;
    // Serial.println("int bad");

  }
  else {
    speedTick++;
    oldTimer2Ct1=timer2_counter1;
    // Serial.println("int good");
  }
  oldTimer2Ct1=timer2_counter1;
  oldTicks = speedTick;

}






uint32_t getEepDistVal(uint8_t eepDataLoc)
{
  uint32_t retval;
  // EEPROM.get( eepDataLoc, retval );
  return retval;
}
uint8_t speedScale = 0;

void computeDistanceCovered();
void computeSpeed();

byte reload = 0xFA;
ISR(TIMER2_COMPA_vect)
{
  
  timer2_counter1++;
  timer2_counter2++;
  timer2_counter3++;
  OCR2A = reload;

  if (timer2_counter1 == TIMER_COUNTER_DISTANCE_COMPUTE)
  {
    timer2_counter1 = 0;
    oldTimer2Ct1=0;// round interrupt error handle
#ifdef SERIAL_TEST_ENABLED
    speedTick += speedScale;
#endif
    computeDistanceCovered();
    //Serial.println("Distance computed");
  }
  if (timer2_counter2 == SPEED_UPDATE_INTERVAL)
  {
    timer2_counter2 = 0;
    computeSpeed();
    //Serial.println("Speed computed");
  }
  if (timer2_counter3 == DISPLAY_UPDATE_INTERVAL)
  {
    timer2_counter3 = 0;
    updateDisplayFlag = true;
    //Serial.println("update display");

  }
}

/*
 * https://microcontrollerslab.com/arduino-timer-interrupts-tutorial/
   timer2
   System clock is 16Mhz and prescalar is 1024 for generating clock of 10ms
  Speed of Timer2 = 16MHz/1024 = 15.625 KHz
  Pulse Time = 1/15.625 KHz = 64 us
  Hence value of OCR register will be set to: 10ms/64us = 156.25 ->156 (whole number)  = 0x9C
*/


void setUpTimer2ForDistanceCompute() {
  cli();
  TCCR0B = 0;
  OCR2A = reload;
  TCCR2A = 1 << WGM21;
  TCCR2B = (1 << CS01) | (1 << CS00);
  TIMSK2 = (1 << OCIE2A);
  sei();
  Serial.println("TIMER2 ticks ever 0.5ms!! Setup Finished.");

}


void setup() {
  Serial.begin(9600);




  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;); // Don't proceed, loop forever
  }


  display.display();
  delay(500); // Pause for 2 seconds

  // Clear the buffer
  display.clearDisplay();
  display.display();

  pinMode(interruptPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(interruptPin),  rotationInterruptHandler, RISING);

  //distance=getEepDistVal(EEPROM_DISTVAL_LOC);

  startTimeCompute = millis();
  debElapsed = millis();
  startTimeDisp = millis();
  setUpTimer2ForDistanceCompute();
}



void updateDisplay(float spd, double dist, bool tripFlg, float power)
{


  display.clearDisplay();
  display.setTextSize(3);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 5);

  if (topSpeedFlag) {
    display.print((uint32_t)(topSpeed*3.6));
    display.setTextSize(1);
    display.println("kmph TOP");
  }
  else {
    display.print((uint32_t)(spd * 3.6));
    display.setTextSize(2);
    display.println("kmph");
  }
  display.setTextSize(2);
  display.setCursor(0, 30);
  tripFlg = true; //hardcoded
  //  if (tripFlg) {}

  display.print(dist);
  display.setTextSize(2);
  display.println("m");
  display.setCursor(0, 50);
  display.setTextSize(2);
  display.print(power);
  display.setTextSize(1);
  display.println("m/s2");
  // display.println("W");
  display.display();
  //delay(200);

}

void updateEepDistVal(uint32_t dist)
{

  //  EEPROM.put((int)EEPROM_DISTVAL_LOC,dist);

}

float powerGen = 0;

float distanceCovered = 0.0;
float prvDistanceCovered = 0.0;
float currentSpeed = 0.0;

float currentAcceleration=0;


void computeDistanceCovered()
{
  uint32_t currentTicks = speedTick - oldSpeedTick;
  double dist = (1.96 * (double)currentTicks);
  distanceCovered += dist;
  oldSpeedTick = speedTick;
}



float speedSumForAvg = 0.0;
float averageSpeed = 0.0;
float prevAvgSpeed=0.0;
void computeSpeed()
{
  static uint8_t speedAvgSamples;
  prevAvgSpeed=averageSpeed;
  currentSpeed = ((distanceCovered - prvDistanceCovered) * 1000) / (SPEED_UPDATE_INTERVAL*10);
  if(topSpeed<prevAvgSpeed && distanceCovered> 20.00 )
  { 
    topSpeed=prevAvgSpeed;
   }
  //currentAcceleration=((currentSpeed-prevCurrentSpeed)*1000)/(SPEED_UPDATE_INTERVAL*10);
  
  prvDistanceCovered = distanceCovered;

  speedSumForAvg += currentSpeed;
  speedAvgSamples++;

  if (speedAvgSamples == NUM_SPEED_AVG_SAMPLES)
  {
    averageSpeed = speedSumForAvg / NUM_SPEED_AVG_SAMPLES;
    speedAvgSamples = 0;
    speedSumForAvg = 0;
    currentAcceleration=((averageSpeed-prevAvgSpeed)*1000)/(SPEED_UPDATE_INTERVAL*10);
  }

}





void loop()
{
  static uint8_t topSpeedDisplayHitCtr;
#ifdef SERIAL_TEST_ENABLED
  if (Serial.available()) {
    char ch = Serial.read();
    if (ch == '+') {
      speedScale < 254 ? speedScale++ : 255;
      Serial.println(speedScale);
    }
    if (ch == '-') {
      speedScale > 0 ? speedScale-- : 0;
      Serial.println(speedScale);
    }
  }
#endif
  if (updateDisplayFlag) {
    topSpeedDisplayHitCtr++;
    float tmpSpeed = prevAvgSpeed;
    if (tmpSpeed < averageSpeed) {
      tmpSpeed += (averageSpeed - tmpSpeed) / 2;
    }
    else if (tmpSpeed > averageSpeed) {
      tmpSpeed -= (tmpSpeed - averageSpeed) / 2;
    }
    else {

    }
    if(topSpeedDisplayHitCtr > TOP_SPEED_HIT_DISPLAY_START && topSpeedDisplayHitCtr<= TOP_SPEED_HIT_DISPLAY_START+3){
      topSpeedFlag=true;
    }
    else if(topSpeedDisplayHitCtr>TOP_SPEED_HIT_DISPLAY_START+3){
       topSpeedDisplayHitCtr=0;
       topSpeedFlag=false;
      }
    else{
      topSpeedFlag=false;
      }
    updateDisplay(tmpSpeed, distanceCovered, false, currentAcceleration);
    updateDisplayFlag = false;
  }




}
