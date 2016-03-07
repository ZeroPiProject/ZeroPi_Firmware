from lib.zeropi import *
#ser = mSerial();
#print ser.serialPorts();
if __name__ == '__main__':
	bot = zeropi()
	bot.start()
	bot.stepperStop(1);
	sleep(1);
	while 1:
		sleep(1);
		bot.stepperRun(1,600);
		sleep(2);
		bot.stepperRun(1,100);
		sleep(2);
		bot.stepperStop(1);
		sleep(1);
		bot.stepperRun(1,-600);
		sleep(2);
		bot.stepperRun(1,-100);
		sleep(2);
		bot.stepperStop(1);