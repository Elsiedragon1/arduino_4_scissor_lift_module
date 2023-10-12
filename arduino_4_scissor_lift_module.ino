#include <ModbusRTUSlave.h>

// Timing
uint32_t currentTick = 0;

//  Modbus ID
const uint16_t id = 4;

//  Modbus data structures
const uint8_t holdingRegisters = 1;

//  Modbus serial 
const uint32_t baud = 115200;
const uint8_t config = SERIAL_8E1;
const uint16_t bufferSize = 256;
const uint8_t dePin = A0;

uint8_t buffer[bufferSize];
ModbusRTUSlave modbus(Serial, buffer, bufferSize, dePin);

enum state {
    LOWERED = 0,
    RISE = 1,
    RISEN = 2,
    LOWER = 3,
    STOP = 4,
    ERROR = 5
};

const uint8_t dirPin = 4;
const uint8_t enablePin = 5;

//  Set the currentState to 2: RISEN ... that way on startup the 
volatile state currentState = 0;    //  Read by the controller - has the actual mode so that mechanical issues don't occur
state targetState = 0;              //  Written to by the controller! What the scissor lift is currently trying to do!
volatile bool newState = false;

int16_t registerRead(uint16_t address)
{
    return currentState;
}

int16_t registerWrite(uint16_t address, uint16_t data)
{
    targetState = data;
    newState = true;
    return true;
}

// ISR
void limitBottom()
{
    // Triggered when the bottom switch has been triggered!
    //  Stop ASAP!
    digitalWrite(enablePin, LOW);                              //  Assuming LOW is disabled!
    digitalWrite(13, LOW);

    currentState = LOWERED;
}

void limitTop()
{
    // Triggered when the top switch has been triggered!
    
    digitalWrite(enablePin, LOW);                              // Assuming LOW is disabled!
    digitalWrite(13, HIGH);
    currentState = RISEN;
}

uint32_t lastTick = 0;
uint32_t interval = 1000/10;

void stopMotor()
{
    digitalWrite(enablePin, LOW);
    //delay(50);                                                  //  Play with these figures!
}

void startMotor()
{
    digitalWrite(enablePin, HIGH);
    //delay(50);                                                  //  These delays might be asymetric
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
    //delay(50);
}

void updateLift()
{
    if (currentTick - lastTick > interval)
    {
        if (newState == true && currentState != targetState)
        {
            switch (targetState)
            {
                case LOWERED:
                    //stopMotor();
                    setMotorDirection(DOWN);
                    startMotor();
                    currentState = LOWER;
                    break;
                case RISEN:
                    //stopMotor();
                    setMotorDirection(UP);
                    startMotor();
                    currentState = RISE;
                    break;
                case STOP:
                    stopMotor();
                    currentState = STOP;
                    break;
                default:
                    //  All other states, RISE/LOWWER, although valid, are more transitions so are ignored!
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
    downwards = !downwards;

    if (downwards) {
      //currentState = RISEN;
      //targetState = LOWERED;
      motorDirection(DOWN);
      startMotor();
      //newState = true;
    } else {
      //currentState = LOWERED;
      //targetState = RISEN;
      startMotor();
      motorDirection(UP);
      
      //newState = true;
    }

    lastTestTick = currentTick;
  }
}

void setup()
{
    Serial.begin(baud, config);
    //modbus.begin(id, baud, config);
    //modbus.configureHoldingRegisters(holdingRegisters, registerRead, registerWrite);

    // pin 2 and 3 set for limit switches!
    pinMode(2, INPUT_PULLUP);
    pinMode(3, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(2), limitBottom, FALLING);
    attachInterrupt(digitalPinToInterrupt(3), limitTop, FALLING);

    pinMode(dirPin, OUTPUT);
    pinMode(enablePin, OUTPUT);
    pinMode(13, OUTPUT);

    // Find out where the scissor lift is at startup!
    /*
    bool bottom = digitalRead(2);
    bool top = digitalRead(3);

    if ( !bottom && !top )
    {
        // Both are triggered! should be impossible!
        currentState = ERROR;
    } else {
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
    */
}

void loop()
{
    currentTick = millis();
    //modbus.poll();
    //updateLift();
    test();
}
