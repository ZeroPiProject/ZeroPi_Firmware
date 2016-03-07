# ZeroPi_Firmware
## How To Use 
* **git clone** https://github.com/zeropiproject/zeropi_firmware
* Open firmware.ino By **Arduino IDE**，compile and upload to ZeroPi board ( [Setup Environment for ZeroPi](https://github.com/ZeroPiProject/ZeroPi_package) ).
* Connect to RaspberrPi with Connection 
* Copy the folder "Python" into Raspberry Pi, then enter the folder and **sudo python** motorRun.py

## Start
* **zeropi**()
* **start**()

## GPIO
* **digitalWrite** ( Pin, Level ) 
* **pwmWrite** ( Pin, Pwm )  
* **digitalRead** ( Pin, **def** onRead )
* **analogRead** ( Pin, **def** onRead )
 
## DC Motor
* **motorRun** ( Device, Pwm ) 
 * Device : 0 = Slot1( 1A+,1A- ), 1 = Slot1( 1B+,1B- ), ... , 6 = Slot4( 4A+,4A- ), 7 = Slot4( 4B+,4B- )  
 * Pwm : -255 ~ 255

## Servo Motor
* **servoRun** ( Pin, Angle)
 * Pin : 0 - 8 ( A0, A1, A2, A3, MO, MI, SCK, SDA, SCL )
 * Angle : 0 ~ 180

## Stepper Motor
* **stepperRun**( Device, Speed )
 * Device : 0 ~ 3 ( Slot1~4 )
 * Speed : 0 ~ 20000
* **stepperStop** ( Device )
* **stepperMove** ( Device, Distance, Speed, **def** onFinish )
* **stepperMoveTo** ( Device, Position, Speed, **def** onFinish )
* **stepperSetting** ( Device, Microstep, Accelation )
 * Microstep : 1, 2, 4, 8, 16
 * Accelation : 100 ~ 10000
