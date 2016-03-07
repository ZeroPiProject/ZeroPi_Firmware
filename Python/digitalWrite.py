from lib.zeropi import *
#ser = mSerial();
#print ser.serialPorts();
if __name__ == '__main__':
	bot = zeropi()
	bot.start()
	while True:
		sleep(0.2);
		bot.digitalWrite(13,1);
		sleep(0.2);
		bot.digitalWrite(13,0);