from lib.zeropi import *

def onRead(level):
	print level;
	sleep(0.5);
	bot.analogRead(1,onRead);

if __name__ == '__main__':
	bot = zeropi()
	bot.start()
	bot.analogRead(1,onRead);
	while 1:
		sleep(1);