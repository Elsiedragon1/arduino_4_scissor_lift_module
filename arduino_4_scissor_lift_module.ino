#include <ModbusRTUSlave.h>

// Timing
uint32_t currentTick = 0;

//  Modbus ID
const uint16_t id = 4;

//  Modbus data structures
const uint8_t holdingRegisters = 1; // To set/read state
const uint8_t coils = 3;            // To read sensor state!

//  Modbus serial 
const uint32_t baud = 115200;
const uint8_t config = SERIAL_8E1;
const uint16_t bufferSize = 256;
const uint8_t dePin = A0;

uint8_t buffer[bufferSize];
ModbusRTUSlave modbus(Serial, buffer, bufferSize, dePin);

#define LOWERED 0
#define RISE 1
#define RISEN 2
#define LOWER 3
#define STOP 4
#define ERROR 5

const uint8_t dirPin = 4;
const uint8_t enablePin = 5;

//  Set the currentState to 2: RISEN ... that way on startup the 
volatile uint16_t currentState = 0;    //  Read by the controller - has the actual mode so that mechanical issues don't occur
uint16_t targetState = 0;              //  Written to by the controller! What the scissor lift is currently trying to do!
volatile bool newState = false;

long registerRead(uint16_t address)
{
    return currentState;
}

int16_t registerWrite(uint16_t address, bool data)
{
    targetState = data;
    newState = true;
    return true;
}

uint32_t lastSensorCheck = 0;
uint32_t sensorInterval = 1000/60;

uint32_t sensorTriggerDuration = 50;

//  This prevents the scissor lift moving on startup! It has to be under instruction of the game or debug menu!
//  Safety first!
bool resetOnStart = false;

bool initialTriggerBottom = false;
uint32_t triggerTimeBottom = 0;
bool triggerBottom = false;
bool initialTriggerTop = false;
uint32_t triggerTimeTop = 0;
bool triggerTop = false;

uint32_t emergencyStopDuration = 1000;
bool emergencyStop = false;

// Coils
int8_t coilRead(uint16_t address)
{
  switch(address)
  {
    case 0:
      return triggerBottom;
    case 1:
      return triggerTop;
    case 2:
      return emergencyStop;
    default:
      return -1;
  }
}

bool coilWrite(uint16_t address, uint8_t data)
{
  return false;
}

uint32_t lastTick = 0;
uint32_t interval = 1000/10;

void stopMotor()
{
    digitalWrite(enablePin, LOW);
}

void startMotor()
{
    digitalWrite(enablePin, HIGH);
}

enum motorDirection {
    DOWN = 0,
    UP = 1
};

void setMotorDirection( uint8_t dir )
{
    if (dir == DOWN)
    {
        digitalWrite(dirPin, HIGH);
    } else {
        digitalWrite(dirPin, LOW);
    }
}

void updateLift()
{
    if (currentTick - lastTick > interval)
    {
        if (triggerBottom)
        {
            if (currentState == LOWER)
            {
                stopMotor();
                currentState = LOWERED;
            }
        }
        if (triggerTop)
        {
            if(currentState == RISE)
            {
                stopMotor();
                currentState = RISEN;
            }
        }
        if (emergencyStop && currentState != RISEN && currentState != LOWERED)
        {
            stopMotor();
            currentState = STOP;
        }
        if (newState == true && currentState != targetState)
        {
            switch (targetState)
            {
                case LOWERED:
                    setMotorDirection(DOWN);
                    startMotor();
                    resetTriggers();          // Allows some time for the end stop to leave the detection zone
                    currentState = LOWER;
                    break;
                case RISEN:
                    setMotorDirection(UP);
                    startMotor();
                    resetTriggers();
                    currentState = RISE;
                    break;
                case STOP:
                    stopMotor();
                    currentState = STOP;
                    break;
                default:
                    //  All other states, RISE/LOWWER, although valid, are more transitions so are ignored! Error state?
                    break;
            }

            newState = false;
        }

        lastTick = currentTick;
    }
}

bool downwards = true;

uint32_t lastTestTick = 0;

void test()
{
  if (currentTick - lastTestTick > 3000)
  {
    resetTriggers();
    downwards = !downwards;

    if (downwards == true) {
      currentState = RISEN;
      targetState = LOWERED;
      newState = true;
    } else {
      currentState = LOWERED;
      targetState = RISEN;
      newState = true;
    }

    lastTestTick = currentTick;
  }
}

void checkSensors()
{
  if (digitalRead(2) == LOW) {
    if (initialTriggerBottom)
    {
      if (currentTick - triggerTimeBottom > sensorTriggerDuration) {
          triggerBottom = true;
      }
      if (currentTick - triggerTimeBottom > emergencyStopDuration) {
          emergencyStop = true;
      }
    }
      else
    {
      // First Trigger
      initialTriggerBottom = true;
      triggerTimeBottom = currentTick;
    }
  } else {
    initialTriggerBottom = false;
  }
  
  if (digitalRead(3) == LOW) {
    if (initialTriggerTop)
    {
      if (currentTick - triggerTimeTop > sensorTriggerDuration) {
        triggerTop = true;
      }
      if (currentTick - triggerTimeTop > emergencyStopDuration) {
          emergencyStop = true;
      }
    }
      else
    {
      // First Trigger
      initialTriggerTop = true;
      triggerTimeTop = currentTick;
    }
  } else {
    initialTriggerTop = false;
  }
}

void resetTriggers()
{
  initialTriggerBottom = false;
  triggerBottom = false;
  initialTriggerTop = false;
  triggerTop = false;
  emergencyStop = false;
}

void setup()
{
    Serial.begin(baud, config);
    modbus.begin(id, baud, config);
    modbus.configureHoldingRegisters(holdingRegisters, registerRead, registerWrite);
    modbus.configureCoils(coils, coilRead, coilWrite);

    // Limit switch pins
    pinMode(2, INPUT_PULLUP);
    pinMode(3, INPUT_PULLUP);
    // Limit switch pins are checked using digitalRead because interrupts are failing because of electrical noise from switching the direction solenoid!

    // Motor control pins
    pinMode(dirPin, OUTPUT);
    pinMode(enablePin, OUTPUT);

    // Find out where the scissor lift is at startup!
    bool bottom = digitalRead(2);
    bool top = digitalRead(3);

    if ( !bottom && !top )
    {
        // Both are triggered! should be impossible!
        currentState = ERROR;
        //  Now what?
    } else {
        if (resetOnStart)
        {
          if ( !top )
          {  // RESET
              currentState = RISEN;
              targetState = LOWERED;
          }
          if ( !bottom )
          {
              currentState = LOWERED;
              targetState = LOWERED;
          }
          if ( top && bottom )
          {
              // Somewhere in between
              currentState = LOWER;
              targetState = LOWERED;
          }
          newState = true;
        }
    }
}

void loop()
{
    currentTick = millis();
    modbus.poll();
    checkSensors();
    updateLift();
    //test();
}
