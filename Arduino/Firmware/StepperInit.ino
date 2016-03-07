// arduino only handle X,Y,Z,E stepper mapping
float curSpd,tarSpd = 1000; // speed profile
float curX,curY,curZ,curE;
float tarX,tarY,tarZ,tarE; // target xyz position
long stepsX,stepsY,stepsZ,stepsE; // target xyz steps
// step value
long posX,posY,posZ,posE; // target stepper position
int8_t motorXfw=1,motorXbk=0;
int8_t motorYfw=1,motorYbk=0;
int8_t motorZfw=1,motorZbk=0;
int8_t motorEfw=1,motorEbk=0;
int8_t _modeCoordinate = 0;
bool _isMovingFinish = false;
ZeroPiStepper stpX(SLOT1);
ZeroPiStepper stpY(SLOT2);
ZeroPiStepper stpZ(SLOT3);
ZeroPiStepper stpE(SLOT4);

/************** calculate movements ******************/
int stepAuxDelay=0;
int stepdelay_min=200;
int stepdelay_max=1000;
#define ACCELERATION 2 // mm/s^2 don't get inertia exceed motor could handle
#define SEGMENT_DISTANCE 10 // 1 mm for each segment
#define SPEED_STEP 1

/******** mapping xy position to steps ******/
#define STEPS_PER_CIRCLE 3200.0f
#define DIAMETER 11 // the diameter of stepper wheel
//#define STEPS_PER_MM (STEPS_PER_CIRCLE/PI/DIAMETER) 
#define STEPS_PER_MM 87.58/4 // the same as 3d printer

/************** motor movements ******************/
void stepperMoveX(int dir)
{
  stpX.step(dir>0);
}
void stepperMoveY(int dir)
{
  stpY.step(dir>0);
}
void stepperMoveZ(int dir)
{
  stpZ.step(dir>0);
}
void stepperMoveE(int dir)
{
  stpE.step(dir>0);
}

void doMove()
{
  if(_isMovingFinish){
    return;
  }
  long mDelay=stepdelay_min;
  long temp_delay;
  int speedDiff = -SPEED_STEP;
  int dX,dY,dZ,dE,maxD;
  float stepX,stepY,stepZ,stepE;
  float cntX=0,cntY=0,cntZ=0,cntE=0;
  int d;
  dX = stepsX - posX;
  dY = stepsY - posY;
  dZ = stepsZ - posZ;
  dE = stepsE - posE;
  maxD = max(abs(dX),max(abs(dY),max(abs(dZ),abs(dE))));
  stepX = (float)abs(dX)/(float)maxD;
  stepY = (float)abs(dY)/(float)maxD;
  stepZ = (float)abs(dZ)/(float)maxD;
  stepE = (float)abs(dE)/(float)maxD;
  // move X
  if(posX!=stepsX){
    cntX+=stepX;
    if(cntX>=1){
      d = dX>0?motorXfw:motorXbk;
      posX+=(dX>0?1:-1);
      stepperMoveX(d);
      cntX-=1;
    }
  }
  // move Y
  if(posY!=stepsY){
    cntY+=stepY;
    if(cntY>=1){
      d = dY>0?motorYfw:motorYbk;
      posY+=(dY>0?1:-1);
      stepperMoveY(d);
      cntY-=1;
    }
  }
  // move Z
  if(posZ!=stepsZ){
    cntZ+=stepZ;
    if(cntZ>=1){
      d = dZ>0?motorZfw:motorZbk;
      posZ+=(dZ>0?1:-1);
      stepperMoveZ(d);
      cntZ-=1;
    }
  }
  // move E
  if(posE!=stepsE){
    cntE+=stepE;
    if(cntE>=1){
      d = dE>0?motorEfw:motorEbk;
      posE+=(dE>0?1:-1);
      stepperMoveE(d);
      cntE-=1;
    }
  }
  mDelay=constrain(mDelay+speedDiff,stepdelay_min,stepdelay_max);
  temp_delay = mDelay + stepAuxDelay;
  if(temp_delay > stepdelay_max)
  {
    temp_delay = stepAuxDelay;
    delay(temp_delay/1000);
    delayMicroseconds(temp_delay%1000);
  }
  else
  {
    delayMicroseconds(temp_delay);
  }
  if(posX==stepsX&&posY==stepsY&&posZ==stepsZ&&posE==stepsE){
    if(!_isMovingFinish){
      _isMovingFinish = true;
      gcodeFinish();
    }
  }else{
  }
  
//  if(maxD<((stepdelay_max-stepdelay_min)/SPEED_STEP)){
//    speedDiff=SPEED_STEP;
//  }
}

void gcodeFinish(){
  curX = tarX;
  curY = tarY;
  curZ = tarZ;
  curE = tarE;
  SerialUSB.println("OK");
  Serial.println("OK");
}
void prepareMove()
{
//  float dx = tarX - curX;
//  float dy = tarY - curY;
//  float distance = sqrt(dx*dx+dy*dy);
//  if (distance < 0.001)
//    return;
  if(_modeCoordinate){
    stepsX = floor((tarX-curX)*STEPS_PER_MM);
    stepsY = floor((tarY-curY)*STEPS_PER_MM);
    stepsZ = floor((tarZ-curZ)*STEPS_PER_MM);
    stepsE = floor((tarE-curE)*STEPS_PER_MM);
  }else{
    stepsX = floor((curX+tarX)*STEPS_PER_MM);
    stepsY = floor((curY+tarY)*STEPS_PER_MM);
    stepsZ = floor((curZ+tarZ)*STEPS_PER_MM);
    stepsE = floor((curE+tarE)*STEPS_PER_MM);
  }
  posX = floor(curX*STEPS_PER_MM);
  posY = floor(curY*STEPS_PER_MM);
  posZ = floor(curZ*STEPS_PER_MM);
  posE = floor(curE*STEPS_PER_MM);
  _isMovingFinish = false;
}
void goHome()
{
  tarX = 0;
  tarY = 0;
  tarZ = 0;
  tarE = 0;
  prepareMove();
}
void setAbsoluteCoord()
{
  _modeCoordinate = 1;
}
void setRelativeCoord()
{
  _modeCoordinate = 0;
}
void setZeroPoint()
{
  initPosition();
}
void initPosition()
{
  curX=0; 
  curY=0;
  curZ=0;
  curE=0;
  posX = 0;
  posY = 0;
  posZ = 0;
  posE = 0;
  stepsX = 0;
  stepsY = 0;
  stepsZ = 0;
  stepsE = 0;
}

/************** calculate movements ******************/
void parseCordinate(char * cmd)
{
  char * tmp;
  char * str;
  str = strtok_r(cmd, " ", &tmp);
//  tarX = curX;
//  tarY = curY;
//  tarZ = curZ;
//  tarE = curE;
  while(str!=NULL){
    str = strtok_r(0, " ", &tmp);
    if(str[0]=='X'){
        tarX = atof(str+1);
    }else if(str[0]=='Y'){
        tarY = atof(str+1);
    }else if(str[0]=='Z'){
        tarZ = atof(str+1);
    }else if(str[0]=='E'){
        tarE = atof(str+1);
    }else if(str[0]=='F'){
      float speed = atof(str+1);
      tarSpd = speed/60; // mm/min -> mm/s
    }else if(str[0]=='A'){
      stepAuxDelay = atol(str+1);
    }
  }
  prepareMove();
}
void enableSteppers(){
  stpX.enable();
  stpY.enable();
  stpZ.enable();
  stpE.enable();
  stpX.setResolution(16);
  stpY.setResolution(16);
  stpZ.setResolution(16);
  stpE.setResolution(16);
}
void disableSteppers(){
  stpX.disable();
  stpY.disable();
  stpZ.disable();
  stpE.disable();
}
