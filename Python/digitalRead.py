from lib.zeropi import *

def onRead(level):
	print level;
	sleep(0.5);
	bot.digitalRead(12,onRead);

if __name__ == '__main__':
	bot = zeropi()
	bot.start()
	bot.digitalRead(12,onRead);
	while 1:
		sleep(1);