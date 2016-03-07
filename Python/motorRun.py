from lib.zeropi import *
#ser = mSerial();
#print ser.serialPorts();
if __name__ == '__main__':
	bot = zeropi()
	bot.start()
	bot.motorRun(1,0);
	sleep(1);
	while 1:
		sleep(1);
		bot.motorRun(1,50);
		sleep(1);
		bot.motorRun(1,0);
		sleep(1);
		bot.motorRun(1,-50);
		sleep(1);
		bot.motorRun(1,0);