#include "ZeroPi.h"
#include <Servo.h>

Servo svrs[4] = {Servo(),Servo(),Servo(),Servo()};
ZeroPiStepper steppers[4] = {ZeroPiStepper(),ZeroPiStepper(),ZeroPiStepper(),ZeroPiStepper()};

void setup(){
  SerialUSB.begin(115200);
  Serial.begin(115200);
  //initTC3(); 
}//end setup

char buf[255];
int8_t bufIndex = 0;

void loop() {
  // put your main code here, to run repeatedly:
  char c = -1;
  bool isAvailable = false;
  if(Serial.available()){
    c = Serial.read();
    isAvailable = true;
  }else if(SerialUSB.available()){
    c = SerialUSB.read();
    isAvailable = true;
  }
  if(isAvailable){
    buf[bufIndex++]=c; 
    if(c=='\n'){
      buf[bufIndex]='\0';              
      parseCmd(buf);
      memset(buf,0,255);
      bufIndex = 0;
    }
    if(bufIndex>=255){
      bufIndex=0;
    }
  }
  steppersRun();
}
void parseGCode(char * cmd)
{
  int code;
  code = atoi(cmd);
  switch(code){
    case 1: // xyze move
      parseCordinate(cmd);
      break;
    case 28: // home
      goHome();
      break; 
    case 90: 
      setAbsoluteCoord();
      break; 
    case 91: // absol
      setRelativeCoord();
      break; 
    case 92: // absol
      setZeroPoint();
      break; 
  }
}
void parseMCode(char * cmd)
{
  int code;
  code = atoi(cmd);
  switch(code){
    case 11:
      parsePinWrite(cmd+1);
    break;
    case 12:
      parseDigitalRead(cmd+1);
    break;
    case 13:
      parseAnalogRead(cmd+1);
    break;
    case 21:
      parseDCMotorRun(cmd+1);
    break;
    case 22:
      parseDCMotorStop(cmd+1);
    break;
    case 31:
      parseEncoderMotorRun(cmd+1);
    break;
    case 32:
      parseEncoderMotorMove(cmd+1);
    break;
    case 33:
      parseEncoderMotorStop(cmd+1);
    break;
    case 34:
      parseEncoderMotorStatus(cmd+1);
    break;
    case 41:
      parseServoRun(cmd+1);
    break;
    case 51:
      parseStepperRun(cmd+1);
    break;
    case 52:
      parseStepperMove(cmd+1);
    break;
    case 53:
      parseStepperSetting(cmd+1);
    break;
    case 54:
      parseStepperStop(cmd+1);
    break;
    case 56:
      enableSteppers();
    break;
    case 57:
      disableSteppers();
    break;
  }
}
void parsePinWrite(char *cmd){
  char * tmp;
  char * str;
  str = strtok_r(cmd, " ", &tmp);
  int pin = 0;
  int level = -1;
  int pwm = -1;
  while(str!=NULL){
    str = strtok_r(0, " ", &tmp);
    if(str[0]=='D'){
      pin = atoi(str+1);
    }else if(str[0]=='L'){
      level = atof(str+1);
    }else if(str[0]=='P'){
      pwm = atof(str+1);
    }
  }
  if(level>-1){
    digitalWrite(pin,level>0?HIGH:LOW);
  }else if(pwm>-1){
    analogWrite(pin,pwm);
  }
}
void parseDigitalRead(char *cmd){
  char * tmp;
  char * str;
  str = strtok_r(cmd, " ", &tmp);
  int pin = 0;
  int level = -1;
  while(str!=NULL){
    str = strtok_r(0, " ", &tmp);
    if(str[0]=='D'){
      pin = atoi(str+1);
      level = digitalRead(pin);
    }
  }
  String s = "M12 D";
  s+=pin;
  s+=" L";
  s+=level;
  s+=" OK";
  Serial.println(s);
  SerialUSB.println(s);
}
void parseAnalogRead(char *cmd){
  char * tmp;
  char * str;
  str = strtok_r(cmd, " ", &tmp);
  int pin = 0;
  int level = -1;
  int amaps[4] = {A0,A1,A2,A3};
  while(str!=NULL){
    str = strtok_r(0, " ", &tmp);
    if(str[0]=='A'){
      pin = atoi(str+1);
      level = analogRead(amaps[pin]);
    }
  }
  String s = "M13 A";
  s+=pin;
  s+=" L";
  s+=level;
  s+=" OK";
  Serial.println(s);
  SerialUSB.println(s);
}
void parseDCMotorRun(char *cmd){
  char * tmp;
  char * str;
  str = strtok_r(cmd, " ", &tmp);
  int port[8] = {0};
  int pwm[8] = {0};
  int portIndex = 0;
  int pwmIndex = 0;
  while(str!=NULL){
    str = strtok_r(0, " ", &tmp);
    if(str[0]=='D'){
      port[portIndex] = atoi(str+1);
      portIndex++;
    }else if(str[0]=='P'){
      pwm[pwmIndex] = atoi(str+1);
      pwmIndex++;
    }
  }
  int len = min(portIndex,pwmIndex);
  ZeroPiDCMotor motor;
  for(int i=0;i<len;i++){
    motor.change((SLOT_NUM_t)floor(port[i]/2),(CHANNEL_t)(port[i]%2));
    motor.run(pwm[i]);
  }
}
void parseDCMotorStop(char *cmd){
  char * tmp;
  char * str;
  str = strtok_r(cmd, " ", &tmp);
  int port[8] = {0};
  int portIndex = 0;
  while(str!=NULL){
    str = strtok_r(0, " ", &tmp);
    if(str[0]=='D'){
      port[portIndex] = atoi(str+1);
      portIndex++;
    }
  }
  ZeroPiDCMotor motor;
  for(int i=0;i<portIndex;i++){
    motor.change((SLOT_NUM_t)floor(port[i]/2),(CHANNEL_t)(port[i]%2));
    motor.stop();
  }
}
void parseEncoderMotorRun(char *cmd){
  char * tmp;
  char * str;
  str = strtok_r(cmd, " ", &tmp);
  int port[4] = {0};
  int portIndex = 0;
  int pwm[4] = {0};
  int pwmIndex = 0;
  while(str!=NULL){
    str = strtok_r(0, " ", &tmp);
    if(str[0]=='D'){
      port[portIndex] = atoi(str+1);
      portIndex++;
    }else if(str[0]=='P'){
      pwm[pwmIndex] = atoi(str+1);
      pwmIndex++;
    }
  }
  int len = min(portIndex,pwmIndex);
  for(int i=0;i<len;i++){
    SerialUSB.print(port[i]);
    SerialUSB.print(" ");
    SerialUSB.println(pwm[i]);
  }
}
void parseEncoderMotorMove(char *cmd){
  char * tmp;
  char * str;
  str = strtok_r(cmd, " ", &tmp);
  int port[4] = {0};
  int portIndex = 0;
  long distance[4] = {0};
  int distIndex = 0;
  int type[4] = {0};
  int typeIndex = 0;
  int pwm[4] = {128,128,128,128};
  int pwmIndex = 0;
  while(str!=NULL){
    str = strtok_r(0, " ", &tmp);
    if(str[0]=='D'){
      port[portIndex] = atoi(str+1);
      portIndex++;
    }else if(str[0]=='A'){
      distance[distIndex] = atol(str+1);
      distIndex++;
      type[typeIndex] = 0;
      typeIndex++;
    }else if(str[0]=='R'){
      distance[distIndex] = atol(str+1);
      distIndex++;
      type[typeIndex] = 1;
      typeIndex++;
    }else if(str[0]=='P'){
      pwm[pwmIndex] = atoi(str+1);
      pwmIndex++;
    }
  }
  int len = min(min(portIndex,pwmIndex),typeIndex);
  for(int i=0;i<len;i++){
    SerialUSB.print(port[i]);
    SerialUSB.print(" ");
    SerialUSB.print(distance[i]);
    SerialUSB.print(" ");
    SerialUSB.print(pwm[i]);
    SerialUSB.print(" ");
    SerialUSB.println(type[i]);
  }
}
void parseEncoderMotorStop(char *cmd){
  char * tmp;
  char * str;
  str = strtok_r(cmd, " ", &tmp);
  int port[4] = {0};
  int portIndex = 0;
  while(str!=NULL){
    str = strtok_r(0, " ", &tmp);
    if(str[0]=='D'){
      port[portIndex] = atoi(str+1);
      portIndex++;
    }
  }
  for(int i=0;i<portIndex;i++){
    SerialUSB.println(port[i]);
  }
}
void parseEncoderMotorStatus(char *cmd){
  char * tmp;
  char * str;
  str = strtok_r(cmd, " ", &tmp);
  int port[4] = {0};
  int portIndex = 0;
  while(str!=NULL){
    str = strtok_r(0, " ", &tmp);
    if(str[0]=='D'){
      port[portIndex] = atoi(str+1);
      portIndex++;
    }
  }
  for(int i=0;i<portIndex;i++){
    SerialUSB.println(port[i]);
  }
}
void parseServoRun(char *cmd){
  char * tmp;
  char * str;
  str = strtok_r(cmd, " ", &tmp);
  int pinMap[9] = {A0,A1,A2,A3,MO,MI,SCK,SDA,SCL};
  int pin[9] = {0,0,0,0,0,0,0,0,0};
  int pinIndex = 0;
  int angle[9] = {0,0,0,0,0,0,0,0,0};
  int angleIndex = 0;
  while(str!=NULL){
    str = strtok_r(0, " ", &tmp);
    if(str[0]=='D'){
      pin[pinIndex] = atoi(str+1);
      pinIndex++;
    }else if(str[0]=='A'){
      angle[angleIndex] = atoi(str+1);
      angleIndex++;
    }
  }
  int len = min(pinIndex,angleIndex);
  for(int i=0;i<len;i++){
    svrs[i].attach(pinMap[pin[i]]);
    svrs[i].write(angle[i]);
  }
}
void parseStepperRun(char *cmd){
  char * tmp;
  char * str;
  str = strtok_r(cmd, " ", &tmp);
  int port[4] = {0,0,0,0};
  int portIndex = 0;
  long feedSpeed[4] = {0,0,0,0};
  int feedIndex = 0;
  while(str!=NULL){
    str = strtok_r(0, " ", &tmp);
    if(str[0]=='D'){
      port[portIndex] = atoi(str+1);
      portIndex++;
    }else if(str[0]=='F'){
      feedSpeed[feedIndex] = atol(str+1);
      feedIndex++;
    }
  }
  int len = min(portIndex,feedIndex);
  for(int i=0;i<len;i++){
    steppers[port[i]].change((SLOT_NUM_t)port[i]);
    steppers[port[i]].enable();
    steppers[port[i]].moveSpeed(feedSpeed[i]);
  }
}
void parseStepperMove(char *cmd){
  char * tmp;
  char * str;
  str = strtok_r(cmd, " ", &tmp);
  int port[4] = {0};
  int portIndex = 0;
  long distance[4] = {0};
  int distIndex = 0;
  int type[4] = {0};
  int typeIndex = 0;
  int feedSpeed[4] = {0};
  int feedIndex = 0;
  while(str!=NULL){
    str = strtok_r(0, " ", &tmp);
    if(str[0]=='D'){
      port[portIndex] = atoi(str+1);
      portIndex++;
    }else if(str[0]=='A'){
      distance[distIndex] = atol(str+1);
      distIndex++;
      type[typeIndex] = 0;
      typeIndex++;
    }else if(str[0]=='R'){
      distance[distIndex] = atol(str+1);
      distIndex++;
      type[typeIndex] = 1;
      typeIndex++;
    }else if(str[0]=='F'){
      feedSpeed[feedIndex] = atoi(str+1);
      feedIndex++;
    }
  }
  int len = min(min(portIndex,feedIndex),typeIndex);
  for(int i=0;i<len;i++){
    steppers[port[i]].change((SLOT_NUM_t)port[i]);
    steppers[port[i]].enable();
    if(type[i]==0){
      steppers[port[i]].moveTo(distance[i],feedSpeed[i]);
    }else{
      steppers[port[i]].move(distance[i],feedSpeed[i]);
    }
  }
}
void parseStepperSetting(char *cmd){
  char * tmp;
  char * str;
  str = strtok_r(cmd, " ", &tmp);
  int port[4] = {0};
  int portIndex = 0;
  int microstep[4] = {0};
  int microstepIndex = 0;
  long accelerate[4] = {0};
  int accelerateIndex = 0;
  long feedSpeed[4] = {0};
  int feedIndex = 0;
  while(str!=NULL){
    str = strtok_r(0, " ", &tmp);
    if(str[0]=='D'){
      port[portIndex] = atoi(str+1);
      portIndex++;
    }else if(str[0]=='A'){
      accelerate[accelerateIndex] = atol(str+1);
      accelerateIndex++;
    }else if(str[0]=='S'){
      microstep[microstepIndex] = atoi(str+1);
      microstepIndex++;
    }else if(str[0]=='F'){
      feedSpeed[feedIndex] = atoi(str+1);
      feedIndex++;
    }
  }
  for(int i=0;i<portIndex;i++){
    steppers[port[i]].change((SLOT_NUM_t)port[i]);
    if(microstep[i]>0){
      steppers[port[i]].setResolution(microstep[i]);
    }
    if(accelerate[i]>0){
      steppers[port[i]].setAcceleration(accelerate[i]);
    }
  }
}
void parseStepperStop(char *cmd){
  char * tmp;
  char * str;
  str = strtok_r(cmd, " ", &tmp);
  int port[4] = {0};
  int portIndex = 0;
  while(str!=NULL){
    str = strtok_r(0, " ", &tmp);
    if(str[0]=='D'){
      port[portIndex] = atoi(str+1);
      portIndex++;
    }
  }
  for(int i=0;i<portIndex;i++){
    steppers[port[i]].change((SLOT_NUM_t)port[i]);
    steppers[port[i]].setSpeed(0);
    steppers[port[i]].disable();
  }
}
void steppersRun(){
  for(int i=0;i<4;i++){
    steppers[i].run();
  }
  doMove();
}
void parseCmd(char * cmd)
{
  if(cmd[0]=='G'){ // gcode
    parseGCode(cmd+1);  
  }else if(cmd[0]=='M'){ // mcode
    parseMCode(cmd+1);
  }
}
void TC3_Handler()
{ 
  doMove();
}
void initTC3(){
  // Enable clock for TC 
  REG_GCLK_CLKCTRL = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID_TCC2_TC3) ;
  while ( GCLK->STATUS.bit.SYNCBUSY == 1 ); // wait for sync 

  // The type cast must fit with the selected timer mode 
  TcCount16* TC = (TcCount16*) TC3; // get timer struct

  TC->CTRLA.reg &= ~TC_CTRLA_ENABLE;   // Disable TCx
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync 

  TC->CTRLA.reg |= TC_CTRLA_MODE_COUNT16;  // Set Timer counter Mode to 16 bits
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync 
  TC->CTRLA.reg |= TC_CTRLA_WAVEGEN_NFRQ; // Set TC as normal Normal Frq
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync 

  TC->CTRLA.reg |= TC_CTRLA_PRESCALER_DIV256;   // Set perscaler
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync 
  
  // TC->PER.reg = 0xFF;   // Set counter Top using the PER register but the 16/32 bit timer counts allway to max  
  // while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync 

  TC->CC[0].reg = 0xFFF;
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync 
  
  // Interrupts 
  TC->INTENSET.reg = 0;              // disable all interrupts
  TC->INTENSET.bit.OVF = 1;          // enable overfollow
  TC->INTENSET.bit.MC0 = 1;          // enable compare match to CC0

  // Enable InterruptVector
  NVIC_EnableIRQ(TC3_IRQn);

  // Enable TC
  TC->CTRLA.reg |= TC_CTRLA_ENABLE;
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync 
}
