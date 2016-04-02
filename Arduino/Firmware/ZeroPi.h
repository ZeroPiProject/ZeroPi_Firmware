#ifndef ZEROPI_H
#define ZEROPI_H

#include "Arduino.h"

#define FORCE_INLINE __INLINE

#define NUM_SLOTS 4
#define SLOT_NUM_PINS 6
#define NUM_EXTIO 11

#define SLOT_NULL 0x0
#define SLOT_STEPPER 0x1
#define SLOT_MOTOR 0x2

#define EXT_NULL 0x0
#define EXT_INPUT 0x1
#define EXT_OUTPUT 0x2
#define EXT_SERVO 0x3

// stepper functin define
#define STEP_EN 0x0
#define STEP_MS1 0x1
#define STEP_MS2 0x2
#define STEP_MS3 0x3
#define STEP_STP 0x4
#define STEP_DIR 0x5

// tb6612 pin define
#define MOTOR_BIN2 0x0
#define MOTOR_BIN1 0x1
#define MOTOR_AIN1 0x2
#define MOTOR_AIN2 0x3
#define MOTOR_PWMB 0x4
#define MOTOR_PWMA 0x5

typedef struct _SLOT
{
  uint8_t pin[SLOT_NUM_PINS];
} SLOT_t;

typedef enum _DRIVER
{
  A4982		= 0,
  A4988		= 1,
  DRV8825	= 2,
  TB67S269	= 3,
  TB67S109	= 4
} DRIVER_t;

typedef enum _SLOT_NUM
{
  SLOT1	= 0,
  SLOT2	= 1,
  SLOT3	= 2,
  SLOT4	= 3,
} SLOT_NUM_t;

typedef enum _CHANNEL
{
  CH_A  = 0,
  CH_B = 1,
} CHANNEL_t;


// define for ext io silk print
#define MO 23
#define MI 22
#define SCK 24
#define SDA 32 // PA22
#define SCL 33 // PA23

// define Temperature IO
#define TEMP0 A4
#define TEMP1 A5
#define MOSFET0 56
#define MOSFET1 57


/* Pins table to be instantiated into zeroPi.cpp */
extern const PinDescription j18pinDescrip[];

#define SET_INPUT(pin) do{PORT->Group[j18pinDescrip[pin].ulPort].PINCFG[j18pinDescrip[pin].ulPin].reg=(uint8_t)(PORT_PINCFG_INEN) ; \
                       PORT->Group[j18pinDescrip[pin].ulPort].DIRCLR.reg = (uint32_t)(1<<j18pinDescrip[pin].ulPin);}while(0);
#define SET_OUTPUT(pin) do{PORT->Group[j18pinDescrip[pin].ulPort].PINCFG[j18pinDescrip[pin].ulPin].reg&=~(uint8_t)(PORT_PINCFG_INEN) ; \
                        PORT->Group[j18pinDescrip[pin].ulPort].DIRSET.reg = (uint32_t)(1<<j18pinDescrip[pin].ulPin) ;}while(0);
#define SET_INPUT_PULLUP(pin) do{PORT->Group[j18pinDescrip[pin].ulPort].PINCFG[j18pinDescrip[pin].ulPin].reg=(uint8_t)(PORT_PINCFG_INEN|PORT_PINCFG_PULLEN) ; \
                              PORT->Group[j18pinDescrip[pin].ulPort].DIRCLR.reg = (uint32_t)(1<<j18pinDescrip[pin].ulPin) ; \
                              PORT->Group[j18pinDescrip[pin].ulPort].OUTSET.reg = (uint32_t)(1<<j18pinDescrip[pin].ulPin) ;while(0);

#define READ(pin) (((PORT->Group[j18pinDescrip[pin].ulPort].IN.reg)>>j18pinDescrip[pin].ulPin) & 1ul)
#define WRITE(pin, v) 	do{SET_OUTPUT(pin);if(v) {PORT->Group[j18pinDescrip[pin].ulPort].OUTSET.reg = (1ul << j18pinDescrip[pin].ulPin);} \
						else {PORT->Group[j18pinDescrip[pin].ulPort].OUTCLR.reg = (1ul << j18pinDescrip[pin].ulPin); }}while(0)
#define TOGGLE(pin) WRITE(pin,!READ(pin))

#ifndef CRITICAL_SECTION_START
#define CRITICAL_SECTION_START  noInterrupts()//unsigned char _sreg = SREG; cli();
#define CRITICAL_SECTION_END    interrupts()//SREG = _sreg;
#endif //CRITICAL_SECTION_START
#ifndef min
#define min(a,b) ((a)<(b)?(a):(b))
#endif // min

#ifndef max
#define max(a,b) ((a)>(b)?(a):(b))
#endif // max

#ifndef square
#define square(x) ((x)*(x))
#endif // square	


class ZeroPiDCMotor
{
public:

  ZeroPiDCMotor();
  ZeroPiDCMotor(SLOT_NUM_t slot, CHANNEL_t channel);
  void change(SLOT_NUM_t slot, CHANNEL_t channel);
  void run(int pwm);
  void stop(void);
  //    void brake(void);
private:
  SLOT_NUM_t slot_num;
  CHANNEL_t motor_channel;
};

class ZeroPiStepper
{
public:
  ZeroPiStepper();
  ZeroPiStepper(SLOT_NUM_t slot);
  void change(SLOT_NUM_t slot);
  void setResolution(int value, DRIVER_t = DRV8825);//set Resolution
  void setDirection(int dir);
  void step(int dir);
  void enable(void);
  void disable(void);
  void move(long steps,float speed);
  void moveSpeed(float speed);
  void moveTo(long steps,float speed);
  void setSpeed(float speed);
  void setAcceleration(long acceleration);
  long currentPosition();
  long distanceToGo();
  void run();
  void stop();
private:
  SLOT_NUM_t slot_num;
  int _mode;
  long _position;
  long _targetPosition;
  float _speed;
  float _targetSpeed;
  float _acceleration;
  bool _isMovingFinish;
};

#endif
