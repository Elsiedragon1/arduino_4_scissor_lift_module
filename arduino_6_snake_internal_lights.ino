#include <Adafruit_NeoPixel.h>

//  Adjust LED Brightness here
#define LED_BRIGHTNESS 40     // From 0 (off) to 255 (brightest)


#define NUM_LEDS 50

Adafruit_NeoPixel leds = Adafruit_NeoPixel(NUM_LEDS, 6, NEO_RGB + NEO_KHZ800);

uint8_t lastMode = 7;
uint8_t mode = 3;

uint32_t currentTick = 0;
uint32_t lastModeCheck = 0;
uint32_t lastLEDUpdate = 0;
uint32_t modeCheckInterval = 100;   //  In milliseconds
uint32_t ledUpdateInterval = 1000/30;

uint32_t animationStart = 0;

void setup()
{
  Serial.begin(115200);

  pinMode(2, INPUT);
  pinMode(3, INPUT);

  leds.begin();
  leds.setBrightness(LED_BRIGHTNESS);
  leds.show();
}

void loop()
{
  currentTick = millis();
  checkMode();
  updateLEDs();
}

void checkMode()
{
  if (currentTick - lastModeCheck > modeCheckInterval)
  {
    lastMode = mode;
    mode = 0;
    if (digitalRead(2))
    {
      mode = mode + 2;
    }
    if (digitalRead(3))
    {
      mode = mode + 1;
    }

    if (lastMode != mode)
    {
      animationStart = currentTick;
    }

    lastModeCheck = currentTick;
  }
}

void updateLEDs()
{
  if (currentTick - lastLEDUpdate > ledUpdateInterval)
  {
    switch(mode)
    {
      case 0: //  LOWERED / DEFAULT
        redPurplePulse();           //  Change the functions here to change the LED behaviour for each state
        break;
      case 1: //  RISING
        purpleBluePulse();
        break;
      case 2: // RISEN
        greenWhite();
        break;
      case 3: // LOWERING
        red();
        break;
      default:
        red();
        break;
    }
    lastLEDUpdate = currentTick;
  }
}

// Red / Purple Slow Pulse/Chase  //////////////////////////////////////////////////////////////////////

//  Duration of the pulse from red to purple and back to red, in milliseconds.
uint32_t pulseDuration = 3000;

//  Colours for the pulse. This uses a colour wheel that will cycle through the rainbow:
//  0 - RED
//  10000 - YELLOW
//  20000 - GREEN
//  30000 - CYAN
//  40000 - BLUE
//  50000 - PINK
//  60000 - PURPLE
//  65536 / 0 - RED

//  These can be changed to any number but it will cycle through all the inbetween colours: Going from
//  cyan to pink will go through blue.
uint16_t maximumColourRP = 0;       //  RED
uint16_t minimumColourRP = 58000;   //  PURPLE
//////////////////////////////////////////////////////////////////////////////////////////////////////

void redPurplePulse()
{
  uint32_t animationDuration = currentTick - animationStart;
  float pulsePosition = (float)(animationDuration%pulseDuration) / (float)pulseDuration;

  //Serial.println(pulsePosition);

  uint16_t hue = minimumColourRP;
  
  if (pulsePosition > 0.5)
  {
    hue = minimumColourRP + ((pulsePosition*2)-1)*(maximumColourRP-minimumColourRP);
  }
  else
  {
    hue = maximumColourRP - (pulsePosition*2)*(maximumColourRP-minimumColourRP);
  }

  leds.fill(leds.ColorHSV(hue,255,255));

  leds.show();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////

//  Duration of the pulse going from purple to blue back to purple, in milliseconds.
uint32_t pulseDurationBlue = 1000;

//  Colours that are cycled between:
uint16_t maximumColourPB = 55000; //  Purple
uint16_t minimumColourPB = 45000; //  Blue
////////////////////////////////////////////////////////////////////////////////////////////////////////

void purpleBluePulse()
{
  uint32_t animationDuration = currentTick - animationStart;
  float pulsePosition = (float)(animationDuration%pulseDurationBlue) / (float)pulseDurationBlue;

  uint16_t hue = minimumColourPB;
  
  if (pulsePosition > 0.5)
  {
    hue = minimumColourPB + ((pulsePosition*2)-1)*(maximumColourPB-minimumColourPB);
  }
  else
  {
    hue = maximumColourPB - (pulsePosition*2)*(maximumColourPB-minimumColourPB);
  }

  leds.fill(leds.ColorHSV(hue,255,255));

  leds.show();
};

////////////////////////////////////////////////////////////////////////////////////////////////////////
//  Duration of the pulse from White to Green back to White
uint32_t pulseDurationGreen = 1500;
//  Colour to pulse
uint16_t colourPulse = 20000;
//  Saturation: The lower the number the closer to white the colour is:
//  0   White
//  255 Fully coloured
uint8_t minimumSaturation = 155;
uint8_t maximumSaturation = 255;
//////////////////////////////////////////////////////////////////////////////////////////////////////

void greenWhite()
{
    uint32_t animationDuration = currentTick - animationStart;
  float pulsePosition = (float)(animationDuration%pulseDurationGreen) / (float)pulseDurationGreen; // 0.0 - 1.0

  uint8_t saturation = maximumSaturation;
  
  if (pulsePosition > 0.5)
  {
    saturation = minimumSaturation + (1-((pulsePosition-0.5)*(pulsePosition-0.5)*4))*(maximumSaturation-minimumSaturation);
  }
  else
  {
    saturation = minimumSaturation - (1-(pulsePosition*pulsePosition*4))*(minimumSaturation-minimumSaturation);
  }

  leds.fill(leds.ColorHSV(colourPulse,saturation,255));

  leds.show();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
//  The duration of a blink from red to black. The light duration will be half of this number, in milliseconds
uint32_t blinkDuration = 500;
//  Colour of the light when on
uint16_t redBlink = 0;
//////////////////////////////////////////////////////////////////////////////////////////////////////

void red()
{
  uint32_t animationDuration = currentTick - animationStart;
  float pulsePosition = (float)(animationDuration%blinkDuration) / (float)blinkDuration; // 0.0 - 1.0

  if (pulsePosition > 0.5)
  {
    leds.fill(leds.ColorHSV(redBlink, 255, 255));
  }
  else
  {
    leds.fill(leds.ColorHSV(redBlink, 255, 0));
  }
  leds.show();
}