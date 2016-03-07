# ZeroPi_Firmware
## How To Use 
1.git clone https://github.com/zeropiproject/zeropi_firmware
2.Open firmware.ino By Arduino IDE，compile and upload to ZeroPi board
3.Connect to RaspberrPi with Connection 
4.Copy the folder Python into Raspberry Pi, then enter the folder and sudo python motorRun.py

## Start
1.zeropi()
2.start()

## GPIO
1.digitalWrite ( Pin, Level ) 
2.pwmWrite ( Pin, Pwm )  
3.digitalRead ( Pin, def onRead )
4.analogRead ( Pin, def onRead )
 
## DC Motor
1.motorRun ( Device, Pwm ) 
a)Device : 0 = Slot 1(1A+,1A-), 1 = Slot1(1B+,1B-), ... , 6 = Slot4(4A+,4A-), 7 = Slot4(4B+,4B-)  
b)Pwm : -255 ~ 255

## Servo Motor
1.servoRun ( Pin, Angle)
a)Pin : 0 - 8 ( A0, A1, A2, A3, MO, MI, SCK, SDA, SCL )
b)Angle : 0 ~ 180

## Stepper Motor
1.stepperRun(Device, Speed)
a)Device : 0 ~ 3 ( Slot1~4 )
b)Speed : 0 ~ 20000
2.stepperStop ( Device)
3.stepperMove ( Device, Distance, Speed, def onFinish )
4.stepperMoveTo ( Device, Position, Speed, def onFinish )
5.stepperSetting ( Device, Microstep, Accelation )
a)Microstep : 1, 2, 4, 8, 16
b)Accelation : 100 ~ 10000
