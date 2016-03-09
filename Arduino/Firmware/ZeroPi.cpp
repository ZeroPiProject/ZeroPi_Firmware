#include "Arduino.h"
#include "zeropi.h"

const SLOT_t slots[NUM_SLOTS] =
{
  {44, 45, 46, 47, 4, 3},
  {48, 49, 50, 51, 6, 5},
  {52, 53, 54, 55, 12, 10},
  {11, 7, 2, 38, 8, 9}
};

//T0  PA05
//T1  PB02
// temperature sensor
void tempInit(void)
{
  //	int portT0 = PORTA;
  //	int pinT0 = 5;
  //	int portT1 = PORTB;
  //	int pinT1 = 2;


  uint32_t temp ;

  // Get whole current setup for both odd and even pins and remove odd one
  temp = (PORT->Group[PORTA].PMUX[5 >> 1].reg) & PORT_PMUX_PMUXE( 0xF ) ;
  // Set new muxing
  PORT->Group[PORTA].PMUX[5 >> 1].reg = temp | PORT_PMUX_PMUXO( PORT_PMUX_PMUXO_B_Val ) ;
  // Enable port mux
  PORT->Group[PORTA].PINCFG[5].reg |= PORT_PINCFG_PMUXEN ;


  temp = (PORT->Group[PORTB].PMUX[2 >> 1].reg) & PORT_PMUX_PMUXO( 0xF ) ;
  PORT->Group[PORTB].PMUX[2 >> 1].reg = temp | PORT_PMUX_PMUXE( PORT_PMUX_PMUXE_B_Val ) ;
  PORT->Group[PORTB].PINCFG[2].reg |= PORT_PINCFG_PMUXEN ; // Enable port mux


  ADC->CTRLA.bit.ENABLE = 0x01;             // Enable ADC
  while (ADC->STATUS.bit.SYNCBUSY == 1);
  ADC->CTRLB.bit.RESSEL = ADC_CTRLB_RESSEL_10BIT_Val;//10bit ADC
  while (ADC->STATUS.bit.SYNCBUSY == 1);
  ADC->CTRLA.bit.ENABLE = 0x01;
  while (ADC->STATUS.bit.SYNCBUSY == 1);

}

uint32_t analogReadChannel(int channel)
{

  uint32_t valueRead = 0;

  ADC->INPUTCTRL.bit.MUXPOS = channel;
  while (ADC->STATUS.bit.SYNCBUSY == 1);
  // Start conversion

  ADC->SWTRIG.bit.START = 1;
  while (ADC->STATUS.bit.SYNCBUSY == 1);
  // Clear the Data Ready flag
  ADC->INTFLAG.bit.RESRDY = 1;
  while (ADC->STATUS.bit.SYNCBUSY == 1);
  ADC->SWTRIG.bit.START = 1;
  while (ADC->STATUS.bit.SYNCBUSY == 1);
  // Store the value
  while ( ADC->INTFLAG.bit.RESRDY == 0 );   // Waiting for conversion to complete
  valueRead = ADC->RESULT.reg;
}





// mosfet control
//void mosfetInit(void)
//{
//	uint32_t temp ;
//	GPIO_PinMode(PORTB, 30, OUTPUT);
//	GPIO_PinMode(PORTB, 31, OUTPUT);

//
////PB31
//    // Get whole current setup for both odd and even pins and remove odd one
//    temp = (PORT->Group[PORTB].PMUX[31 >> 1].reg) & PORT_PMUX_PMUXE( 0xF ) ;
//    // Set new muxing
//    PORT->Group[PORTB].PMUX[31 >> 1].reg = temp | PORT_PMUX_PMUXO( PORT_PMUX_PMUXO_E_Val ) ;
//    // Enable port mux
//    PORT->Group[PORTB].PINCFG[31].reg |= PORT_PINCFG_PMUXEN ;

////PB30
//    temp = (PORT->Group[PORTB].PMUX[30 >> 1].reg) & PORT_PMUX_PMUXO( 0xF ) ;
//    PORT->Group[PORTB].PMUX[30 >> 1].reg = temp | PORT_PMUX_PMUXE( PORT_PMUX_PMUXE_E_Val ) ;
//    PORT->Group[PORTB].PINCFG[30].reg |= PORT_PINCFG_PMUXEN ; // Enable port mux
//
//
//  GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID( GCM_TCC0_TCC1 )) ;
//  TCC0->CTRLA.reg &= ~(TCC_CTRLA_ENABLE);       //disable TCC module
//  TCC0->CTRLA.reg |= TCC_CTRLA_PRESCALER_DIV256;  //setting prescaler to divide by 256
//  TCC0->WAVE.reg |= TCC_WAVE_WAVEGEN_NPWM;      //
//  TCC0->CC[0].reg = 0;                //
//  TCC0->CC[1].reg = 0;                //
//  TCC0->PER.reg = 254;
//  TCC0->CTRLA.reg |= TCC_CTRLA_ENABLE ;	      //ENABLE
//}

//void mosfetSet(int index, int value)
//{
//	TCC0->CC[index].reg = value;
//}

ZeroPiDCMotor::ZeroPiDCMotor(){
	
}
ZeroPiDCMotor::ZeroPiDCMotor(SLOT_NUM_t slot, CHANNEL_t channel)
{
  slot_num = slot;
  motor_channel = channel;
  for (int n = 0; n < SLOT_NUM_PINS; n++)
  {
    SET_OUTPUT(slots[slot_num].pin[n]);
  }
}
void ZeroPiDCMotor::change(SLOT_NUM_t slot, CHANNEL_t channel){
  slot_num = slot;
  motor_channel = channel;
  for (int n = 0; n < SLOT_NUM_PINS; n++)
  {
    SET_OUTPUT(slots[slot_num].pin[n]);
  }
}
static void syncTC_8(Tc *TCx)
{
  while (TCx->COUNT8.STATUS.bit.SYNCBUSY);
}
void ZeroPiDCMotor::run(int pwm)
{
  pwm = constrain(pwm, -255, 255);
  switch (motor_channel)
  {
    case CH_A:
      if (pwm > 0)
      {
        WRITE(slots[slot_num].pin[MOTOR_AIN1], 1);
        WRITE(slots[slot_num].pin[MOTOR_AIN2], 0);
      }
      else
      {
        WRITE(slots[slot_num].pin[MOTOR_AIN1], 0);
        WRITE(slots[slot_num].pin[MOTOR_AIN2], 1);
      }
      pwm = abs(pwm);
      analogWrite(slots[slot_num].pin[MOTOR_PWMA], pwm);

      break;
    case CH_B:
      if (pwm > 0)
      {
        WRITE(slots[slot_num].pin[MOTOR_BIN1], 1);
        WRITE(slots[slot_num].pin[MOTOR_BIN2], 0);
      }
      else
      {
        WRITE(slots[slot_num].pin[MOTOR_BIN1], 0);
        WRITE(slots[slot_num].pin[MOTOR_BIN2], 1);
      }
      pwm = abs(pwm);
      analogWrite(slots[slot_num].pin[MOTOR_PWMB], pwm);
      break;
  }

  GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID( GCM_TCC0_TCC1 )) ;
  while ( GCLK->STATUS.bit.SYNCBUSY == 1 ) ;

  TCC0->CTRLA.reg &= ~TCC_CTRLA_ENABLE;
  while (TCC0->SYNCBUSY.reg & TCC_SYNCBUSY_MASK);
  TCC0->CTRLA.reg |= TCC_CTRLA_PRESCALER_DIV16;  //setting prescaler to divide by 16
  TCC0->CTRLA.reg |= TCC_CTRLA_ENABLE ;
  while (TCC0->SYNCBUSY.reg & TCC_SYNCBUSY_MASK);

  TCC1->CTRLA.reg &= ~TCC_CTRLA_ENABLE;
  while (TCC1->SYNCBUSY.reg & TCC_SYNCBUSY_MASK);
  TCC1->CTRLA.reg |= TCC_CTRLA_PRESCALER_DIV16;  //setting prescaler to divide by 16
  TCC1->CTRLA.reg |= TCC_CTRLA_ENABLE ;
  while (TCC1->SYNCBUSY.reg & TCC_SYNCBUSY_MASK);

  GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID( GCM_TCC2_TC3 )) ;
  while ( GCLK->STATUS.bit.SYNCBUSY == 1 ) ;

  TC3->COUNT8.CTRLA.reg &= ~TC_CTRLA_ENABLE;
  while (TC3->COUNT8.STATUS.bit.SYNCBUSY);
  TC3->COUNT8.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV16;  //setting prescaler to divide by 16
  TC3->COUNT8.CTRLA.reg |= TC_CTRLA_ENABLE;
  while (TC3->COUNT8.STATUS.bit.SYNCBUSY);
}
/*
void ZeroPiDCMotor::stop(void)
{
  switch (motor_channel)
  {
    case CH_A:
      WRITE(slots[slot_num].pin[MOTOR_AIN1], 0);
      WRITE(slots[slot_num].pin[MOTOR_AIN2], 0);
      break;
    case CH_B:
      WRITE(slots[slot_num].pin[MOTOR_BIN1], 0);
      WRITE(slots[slot_num].pin[MOTOR_BIN2], 0);
      break;
  }
}
*/
void ZeroPiDCMotor::stop(void)
{
  switch (motor_channel)
  {
    case CH_A:
      WRITE(slots[slot_num].pin[MOTOR_AIN1], 1);
      WRITE(slots[slot_num].pin[MOTOR_AIN2], 1);
      break;
    case CH_B:
      WRITE(slots[slot_num].pin[MOTOR_BIN1], 1);
      WRITE(slots[slot_num].pin[MOTOR_BIN2], 1);
      break;
  }
}

ZeroPiStepper::ZeroPiStepper()
{
	_acceleration = 10;
  _speed = 0;
  _targetSpeed = 0;
  _position = 0;
}
ZeroPiStepper::ZeroPiStepper(SLOT_NUM_t slot)
{
  slot_num = slot;
  for (int n = 0; n < SLOT_NUM_PINS; n++)
  {
    SET_OUTPUT(slots[slot_num].pin[n]);
  }
  disable();
}

void ZeroPiStepper::change(SLOT_NUM_t slot)
{
	  
  slot_num = slot;
  for (int n = 0; n < SLOT_NUM_PINS; n++)
  {
    SET_OUTPUT(slots[slot_num].pin[n]);
  }
  disable();
}
void ZeroPiStepper::setResolution(int value, DRIVER_t type)
{
  switch (type)
  {
    case A4982://only MS1 MS2
      switch (value)
      {
        case 1:
          WRITE(slots[slot_num].pin[STEP_MS1], 0);
          WRITE(slots[slot_num].pin[STEP_MS2], 0);
          break;
        case 2:
          WRITE(slots[slot_num].pin[STEP_MS1], 1);
          WRITE(slots[slot_num].pin[STEP_MS2], 0);
          break;
        case 4:
          WRITE(slots[slot_num].pin[STEP_MS1], 0);
          WRITE(slots[slot_num].pin[STEP_MS2], 1);
          break;
        case 16:
          WRITE(slots[slot_num].pin[STEP_MS1], 1);
          WRITE(slots[slot_num].pin[STEP_MS2], 1);
          break;
        default:
          WRITE(slots[slot_num].pin[STEP_MS1], 0);
          WRITE(slots[slot_num].pin[STEP_MS2], 0);
          break;
      }
      break;
    case A4988:
      switch (value)
      {
        case 1:
          WRITE(slots[slot_num].pin[STEP_MS1], 0);
          WRITE(slots[slot_num].pin[STEP_MS2], 0);
          WRITE(slots[slot_num].pin[STEP_MS3], 0);
          break;
        case 2:
          WRITE(slots[slot_num].pin[STEP_MS1], 1);
          WRITE(slots[slot_num].pin[STEP_MS2], 0);
          WRITE(slots[slot_num].pin[STEP_MS3], 0);
          break;
        case 4:
          WRITE(slots[slot_num].pin[STEP_MS1], 0);
          WRITE(slots[slot_num].pin[STEP_MS2], 1);
          WRITE(slots[slot_num].pin[STEP_MS3], 0);
          break;
        case 8:
          WRITE(slots[slot_num].pin[STEP_MS1], 1);
          WRITE(slots[slot_num].pin[STEP_MS2], 1);
          WRITE(slots[slot_num].pin[STEP_MS3], 0);
          break;
        case 16:
          WRITE(slots[slot_num].pin[STEP_MS1], 1);
          WRITE(slots[slot_num].pin[STEP_MS2], 1);
          WRITE(slots[slot_num].pin[STEP_MS3], 1);
          break;
        default:
          WRITE(slots[slot_num].pin[STEP_MS1], 0);
          WRITE(slots[slot_num].pin[STEP_MS2], 0);
          WRITE(slots[slot_num].pin[STEP_MS3], 0);
          break;
      }
      break;
    case DRV8825:
      switch (value)
      {
        case 1:
          WRITE(slots[slot_num].pin[STEP_MS1], 0);
          WRITE(slots[slot_num].pin[STEP_MS2], 0);
          WRITE(slots[slot_num].pin[STEP_MS3], 0);
          break;
        case 2:
          WRITE(slots[slot_num].pin[STEP_MS1], 1);
          WRITE(slots[slot_num].pin[STEP_MS2], 0);
          WRITE(slots[slot_num].pin[STEP_MS3], 0);
          break;
        case 4:
          WRITE(slots[slot_num].pin[STEP_MS1], 0);
          WRITE(slots[slot_num].pin[STEP_MS2], 1);
          WRITE(slots[slot_num].pin[STEP_MS3], 0);
          break;
        case 8:
          WRITE(slots[slot_num].pin[STEP_MS1], 1);
          WRITE(slots[slot_num].pin[STEP_MS2], 1);
          WRITE(slots[slot_num].pin[STEP_MS3], 0);
          break;
        case 16:
          WRITE(slots[slot_num].pin[STEP_MS1], 0);
          WRITE(slots[slot_num].pin[STEP_MS2], 0);
          WRITE(slots[slot_num].pin[STEP_MS3], 1);
          break;
        case 32:
          WRITE(slots[slot_num].pin[STEP_MS1], 1);
          WRITE(slots[slot_num].pin[STEP_MS2], 1);
          WRITE(slots[slot_num].pin[STEP_MS3], 1);
          break;
        default:
          WRITE(slots[slot_num].pin[STEP_MS1], 0);
          WRITE(slots[slot_num].pin[STEP_MS2], 0);
          WRITE(slots[slot_num].pin[STEP_MS3], 0);
          break;
      }
      break;
    case TB67S269:
    case TB67S109:
      switch (value)
      {
        case 1:
          WRITE(slots[slot_num].pin[STEP_MS1], 0);
          WRITE(slots[slot_num].pin[STEP_MS2], 0);
          WRITE(slots[slot_num].pin[STEP_MS3], 1);
          break;
        case 2://Half step resolution(Type (A))
          WRITE(slots[slot_num].pin[STEP_MS1], 0);
          WRITE(slots[slot_num].pin[STEP_MS2], 1);
          WRITE(slots[slot_num].pin[STEP_MS3], 0);
          break;
        case 4:
          WRITE(slots[slot_num].pin[STEP_MS1], 0);
          WRITE(slots[slot_num].pin[STEP_MS2], 1);
          WRITE(slots[slot_num].pin[STEP_MS3], 1);
          break;
        case 8:
          WRITE(slots[slot_num].pin[STEP_MS1], 1);
          WRITE(slots[slot_num].pin[STEP_MS2], 0);
          WRITE(slots[slot_num].pin[STEP_MS3], 1);
          break;
        case 16:
          WRITE(slots[slot_num].pin[STEP_MS1], 1);
          WRITE(slots[slot_num].pin[STEP_MS2], 1);
          WRITE(slots[slot_num].pin[STEP_MS3], 0);
          break;
        case 32:
          WRITE(slots[slot_num].pin[STEP_MS1], 1);
          WRITE(slots[slot_num].pin[STEP_MS2], 1);
          WRITE(slots[slot_num].pin[STEP_MS3], 1);
          break;
        default:
          WRITE(slots[slot_num].pin[STEP_MS1], 0);
          WRITE(slots[slot_num].pin[STEP_MS2], 0);
          WRITE(slots[slot_num].pin[STEP_MS3], 1);
          break;
      }
      break;
    default:
      break;
  }
}
void ZeroPiStepper::setDirection(int dir)
{
  WRITE(slots[slot_num].pin[STEP_DIR], dir);
}
void ZeroPiStepper::step(int dir)
{
  setDirection(dir);
  WRITE(slots[slot_num].pin[STEP_STP], HIGH);
  __NOP();
  WRITE(slots[slot_num].pin[STEP_STP], LOW);
  _position+=dir>0?1:-1;
}
void ZeroPiStepper::enable(void)
{
  WRITE(slots[slot_num].pin[STEP_EN], 0);
}
void ZeroPiStepper::disable(void)
{
  WRITE(slots[slot_num].pin[STEP_EN], 1);
}
void ZeroPiStepper::move(long steps,float speed)
{
  _isMovingFinish = false;
  setSpeed(speed);
  _targetPosition += steps;
  _mode = 2;
}
void ZeroPiStepper::moveTo(long position,float speed)
{
  _isMovingFinish = false;
  setSpeed(speed);
  _targetPosition = position;
  _mode = 2;
}
void ZeroPiStepper::moveSpeed(float speed){
  _isMovingFinish = false;
  setSpeed(speed);
  _mode = 1;
}
void ZeroPiStepper::setSpeed(float speed)
{
  _targetSpeed = speed;
}
void ZeroPiStepper::stop()
{
  _targetSpeed = 0;
  _mode = 0;
}
void ZeroPiStepper::setAcceleration(long acceleration)
{
  _acceleration = acceleration/100;
}
long ZeroPiStepper::currentPosition()
{
  return _position;
}
long ZeroPiStepper::distanceToGo()
{
  return _targetPosition - _position;
}
void ZeroPiStepper::run(){
  long dist = distanceToGo();
  _speed += _targetSpeed>_speed?_acceleration:(_targetSpeed<_speed?-_acceleration:0);
  float mSpeed = abs(_speed)+80;
  long delayTime = 0;
  int dir;
  switch(_mode){
    case 1:
      if(mSpeed>0.01){
        dir = _speed>0;
        step(dir);
        delayTime = (long)(1000000/mSpeed);
        if(delayTime>1000){
          delay(delayTime/1000);
          delayMicroseconds(delayTime%1000);
        }else{
          delayMicroseconds(delayTime);
        }
      }
    break;
    case 2:
      if(dist!=0){
        dir = dist>0;
        if(abs(dist)<200){
          _targetSpeed = 0;
        }
        step(dir);
        delayTime = (long)(1000000/mSpeed);
        if(delayTime>1000){
          delay(delayTime/1000);
          delayMicroseconds(delayTime%1000);
        }else{
          delayMicroseconds(delayTime);
        }
      }else{
        if(!_isMovingFinish){
          _isMovingFinish = true;
          String s = "\r\r\r\r\r\r\r\nR52 D";
          s += slot_num;
          s += " OK\n";
          Serial.print(s);
          SerialUSB.println(s);
        }
      }
    break;
  }
}

/*
 * Pins descriptions
 */
const PinDescription j18pinDescrip[] =
{
  // 0..13 - Digital pins
  // ----------------------
  // 0/1 - SERCOM/UART (Serial1)
  { PORTA, 11, PIO_SERCOM, (PIN_ATTR_DIGITAL), No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_11 }, // RX: SERCOM0/PAD[3]
  { PORTA, 10, PIO_SERCOM, (PIN_ATTR_DIGITAL), No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_10 }, // TX: SERCOM0/PAD[2]

  // 2..12
  // Digital Low
  { PORTA, 14, PIO_DIGITAL, (PIN_ATTR_DIGITAL), No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_14 },
  { PORTA,  9, PIO_TIMER, (PIN_ATTR_DIGITAL | PIN_ATTR_PWM | PIN_ATTR_TIMER), No_ADC_Channel, PWM0_CH1, TCC0_CH1, EXTERNAL_INT_9 }, // TCC0/WO[1]
  { PORTA,  8, PIO_TIMER, (PIN_ATTR_DIGITAL | PIN_ATTR_PWM | PIN_ATTR_TIMER), No_ADC_Channel, PWM0_CH0, TCC0_CH0, EXTERNAL_INT_NMI }, // TCC0/WO[0]
  { PORTA, 15, PIO_TIMER, (PIN_ATTR_DIGITAL | PIN_ATTR_PWM | PIN_ATTR_TIMER), No_ADC_Channel, PWM3_CH1, TC3_CH1, EXTERNAL_INT_15 }, // TC3/WO[1]
  { PORTA, 20, PIO_TIMER_ALT, (PIN_ATTR_DIGITAL | PIN_ATTR_PWM | PIN_ATTR_TIMER), No_ADC_Channel, PWM0_CH6, TCC0_CH6, EXTERNAL_INT_4 }, // TCC0/WO[6]
  { PORTA, 21, PIO_DIGITAL, (PIN_ATTR_DIGITAL), No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_5 },

  // Digital High
  { PORTA,  6, PIO_TIMER, (PIN_ATTR_DIGITAL | PIN_ATTR_PWM | PIN_ATTR_TIMER), No_ADC_Channel, PWM1_CH0, TCC1_CH0, EXTERNAL_INT_4 }, // TCC1/WO[0]
  { PORTA,  7, PIO_TIMER, (PIN_ATTR_DIGITAL | PIN_ATTR_PWM | PIN_ATTR_TIMER), No_ADC_Channel, PWM1_CH1, TCC1_CH1, EXTERNAL_INT_5 }, // TCC1/WO[1]
  { PORTA, 18, PIO_TIMER, (PIN_ATTR_DIGITAL | PIN_ATTR_PWM | PIN_ATTR_TIMER), No_ADC_Channel, PWM3_CH0, TC3_CH0, EXTERNAL_INT_2 }, // TC3/WO[0]
  { PORTA, 16, PIO_TIMER, (PIN_ATTR_DIGITAL | PIN_ATTR_PWM | PIN_ATTR_TIMER), No_ADC_Channel, PWM2_CH0, TCC2_CH0, EXTERNAL_INT_0 }, // TCC2/WO[0]
  { PORTA, 19, PIO_TIMER_ALT, (PIN_ATTR_DIGITAL | PIN_ATTR_PWM | PIN_ATTR_TIMER), No_ADC_Channel, PWM0_CH3, TCC0_CH3, EXTERNAL_INT_3 }, // TCC0/WO[3]

  // 13 (LED)
  { PORTA, 17, PIO_PWM, (PIN_ATTR_DIGITAL | PIN_ATTR_PWM), No_ADC_Channel, PWM2_CH1, NOT_ON_TIMER, EXTERNAL_INT_1 }, // TCC2/WO[1]

  // 14..19 - Analog pins
  // --------------------
  { PORTA,  2, PIO_ANALOG, PIN_ATTR_ANALOG, ADC_Channel0, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_2 }, // ADC/AIN[0]
  { PORTB,  8, PIO_ANALOG, PIN_ATTR_ANALOG, ADC_Channel2, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_8 }, // ADC/AIN[2]
  { PORTB,  9, PIO_ANALOG, PIN_ATTR_ANALOG, ADC_Channel3, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_9 }, // ADC/AIN[3]
  { PORTA,  4, PIO_ANALOG, PIN_ATTR_ANALOG, ADC_Channel4, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_4 }, // ADC/AIN[4]
  { PORTA,  5, PIO_ANALOG, PIN_ATTR_ANALOG, ADC_Channel5, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_5 }, // ADC/AIN[5]
  { PORTB,  2, PIO_ANALOG, PIN_ATTR_ANALOG, ADC_Channel10, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_2 }, // ADC/AIN[10]

  // 20..21 I2C pins (SDA/SCL and also EDBG:SDA/SCL)
  // ----------------------
  { PORTA, 22, PIO_SERCOM, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_6 }, // SDA: SERCOM3/PAD[0]
  { PORTA, 23, PIO_SERCOM, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_7 }, // SCL: SERCOM3/PAD[1]

  // 22..24 - SPI pins (ICSP:MISO,SCK,MOSI)
  // ----------------------
  { PORTA, 12, PIO_SERCOM_ALT, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_12 }, // MISO: SERCOM4/PAD[0]
  { PORTB, 10, PIO_SERCOM_ALT, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_10 }, // MOSI: SERCOM4/PAD[2]
  { PORTB, 11, PIO_SERCOM_ALT, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_11 }, // SCK: SERCOM4/PAD[3]

  // 25..26 - RX/TX LEDS (PB03/PA27)
  // --------------------
  { PORTB,  3, PIO_OUTPUT, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // used as output only
  { PORTA, 27, PIO_OUTPUT, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // used as output only

  // 27..29 - USB
  // --------------------
  { PORTA, 28, PIO_COM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // USB Host enable
  { PORTA, 24, PIO_COM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // USB/DM
  { PORTA, 25, PIO_COM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // USB/DP

  // 30..41 - EDBG
  // ----------------------
  // 30/31 - EDBG/UART
  { PORTB, 22, PIO_SERCOM_ALT, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // TX: SERCOM5/PAD[2]
  { PORTB, 23, PIO_SERCOM_ALT, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // RX: SERCOM5/PAD[3]

  // 32/33 I2C (SDA/SCL and also EDBG:SDA/SCL)
  { PORTA, 22, PIO_SERCOM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // SDA: SERCOM3/PAD[0]
  { PORTA, 23, PIO_SERCOM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // SCL: SERCOM3/PAD[1]

  // 34..37 - EDBG/SPI
  { PORTA, 19, PIO_SERCOM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // MISO: SERCOM1/PAD[3]
  { PORTA, 16, PIO_SERCOM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // MOSI: SERCOM1/PAD[0]
  { PORTA, 18, PIO_SERCOM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // SS: SERCOM1/PAD[2]
  { PORTA, 17, PIO_SERCOM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // SCK: SERCOM1/PAD[1]

  // 38..41 - EDBG/Digital
  { PORTA, 13, PIO_PWM, (PIN_ATTR_DIGITAL | PIN_ATTR_PWM), No_ADC_Channel, PWM0_CH5, NOT_ON_TIMER, EXTERNAL_INT_13 }, // EIC/EXTINT[13] *TCC2/WO[1] TCC0/WO[7]
  { PORTA, 21, PIO_PWM_ALT, (PIN_ATTR_DIGITAL | PIN_ATTR_PWM), No_ADC_Channel, PWM0_CH7, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // Pin 7
  { PORTA,  6, PIO_PWM, (PIN_ATTR_DIGITAL | PIN_ATTR_PWM), No_ADC_Channel, PWM1_CH0, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // Pin 8
  { PORTA,  7, PIO_PWM, (PIN_ATTR_DIGITAL | PIN_ATTR_PWM), No_ADC_Channel, PWM1_CH1, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // Pin 9

  // 42 (AREF)
  { PORTA, 3, PIO_ANALOG, PIN_ATTR_ANALOG, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // DAC/VREFP

  // ----------------------
  // 43 - Alternate use of A0 (DAC output)
  { PORTA,  2, PIO_ANALOG, PIN_ATTR_ANALOG, DAC_Channel0, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_2 }, // DAC/VOUT


  //44..47
  { PORTB,  4, PIO_ANALOG, PIN_ATTR_ANALOG, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_4 }, //
  { PORTB,  5, PIO_ANALOG, PIN_ATTR_ANALOG, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_5 }, //
  { PORTB,  6, PIO_ANALOG, PIN_ATTR_ANALOG, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_6 }, //
  { PORTB,  7, PIO_ANALOG, PIN_ATTR_ANALOG, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_7 }, //
  //48..51
  { PORTB,  17, PIO_TIMER, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_1 }, //
  { PORTB,  16, PIO_TIMER, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_0 }, //
  { PORTB,  1, PIO_SERCOM, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_1 }, //
  { PORTB,  0, PIO_SERCOM, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_0 }, //
  //52..55
  { PORTB,  12, PIO_TIMER, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_12 }, //
  { PORTB,  13, PIO_TIMER, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_13 }, //
  { PORTB,  14, PIO_TIMER, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_14 }, //
  { PORTB,  15, PIO_TIMER, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_15 }, //
  //56..57
  { PORTB,  30, PIO_TIMER, (PIN_ATTR_DIGITAL | PIN_ATTR_PWM | PIN_ATTR_TIMER), No_ADC_Channel, PWM0_CH0, TCC0_CH0, EXTERNAL_INT_14 }, // TCC0/WO[0]
  { PORTB,  31, PIO_TIMER, (PIN_ATTR_DIGITAL | PIN_ATTR_PWM | PIN_ATTR_TIMER), No_ADC_Channel, PWM0_CH1, TCC0_CH1, EXTERNAL_INT_15 }, // TCC0/WO[1]
  //58..59
  { PORTA,  30, PIO_DIGITAL, (PIN_ATTR_DIGITAL), No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // SWCLK
  { PORTA,  31, PIO_DIGITAL, (PIN_ATTR_DIGITAL), No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // SWDIO

} ;
