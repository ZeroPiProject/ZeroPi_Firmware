from lib.zeropi import *

def onForwardFinish():
	sleep(1);
	bot.stepperMove(1,-5000,2000,onBackwardFinish);

def onBackwardFinish():
	sleep(1);
	bot.stepperMove(1,5000,2000,onForwardFinish);

if __name__ == '__main__':
	bot = zeropi()
	bot.start()
	bot.stepperSetting(1,8,5000);
	bot.stepperStop(1);
	sleep(1);
	onForwardFinish();
	while 1:
		sleep(1);