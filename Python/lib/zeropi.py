import serial
import sys,time
import signal
from time import ctime,sleep
import glob,struct
from multiprocessing import Process,Manager,Array
import threading


class mSerial():
	ser = None
	def __init__(self):
		print self

	def start(self):
		self.ser = serial.Serial('/dev/ttyAMA0',115200,timeout=0.5)
	
	def device(self):
		return self.ser

	def serialPorts(self):
		if sys.platform.startswith('win'):
			ports = ['COM%s' % (i + 1) for i in range(256)]
		elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
			ports = glob.glob('/dev/tty[A-Za-z]*')
		elif sys.platform.startswith('darwin'):
			ports = glob.glob('/dev/tty.*')
		else:
			raise EnvironmentError('Unsupported platform')
		result = []
		for port in ports:
			s = serial.Serial()
			s.port = port
			s.close()
			result.append(port)
		return result

	def writePackage(self,package):
		self.ser.write(package)
		sleep(0.01)

	def read(self,n):
		return self.ser.read(n)

	def isOpen(self):
		return self.ser.isOpen()

	def inWaiting(self):
		return self.ser.inWaiting()

	def close(self):
		self.ser.close()
		
	
class zeropi():
	def __init__(self):
		print "init zeropi"
		signal.signal(signal.SIGINT, self.exit)
		self.manager = Manager()
		self.__selectors = self.manager.dict()
		self.buffer = []
		self.bufferIndex = 0
		self.message = "";
		self.isParseStart = False
		self.exiting = False
		self.isParseStartIndex = 0
		
	def start(self):
		self.device = mSerial()
		self.device.start()
		sleep(0.1)
		self.run()
	
	
	def excepthook(self, exctype, value, traceback):
		self.close()
		
	def run(self):
		sys.excepthook = self.excepthook
		th = threading.Thread(target=self.__onRead,args=(self.onParse,))
		th.start()
		
	def close(self):
		self.device.close()
		
	def exit(self, signal, frame):
		self.exiting = True
		sys.exit(0)
		
	def __onRead(self,callback):
		while True:
			if(self.exiting==True):
				break
			try:	
				if self.device.isOpen()==True:
					n = self.device.inWaiting()
					if n>0:
						callback(self.device.read(n))
					sleep(0.1)
				else:	
					sleep(0.5)
			except Exception,ex:
				print 'onRead...'+str(ex)
				self.exiting = True;
				#self.close()
				sleep(0.01)
				
	def __writePackage(self,pack):
		self.device.writePackage(pack)

	def motorRun(self,port,speed):
		self.__writePackage('M21 D'+str(port)+' P'+str(speed)+'\n')

	def stepperRun(self,port,speed):
		self.__writePackage('M51 D'+str(port)+' F'+str(speed)+'\n')

	def stepperStop(self,port):
		self.__writePackage('M54 D'+str(port)+'\n')

	def stepperMove(self,port,distance,speed,callback):
		self.__doCallback('M52 D'+str(port),callback)
		self.__writePackage('M52 D'+str(port)+' R'+str(distance)+' F'+str(speed)+'\n')
	
	def stepperMoveTo(self,port,position,speed,callback):
		self.__doCallback('M52 D'+str(port),callback)
		self.__writePackage('M52 D'+str(port)+' A'+str(position)+' F'+str(speed)+'\n')

	def stepperSetting(self,port,microstep,accelate):
		self.__writePackage('M53 D'+str(port)+' S'+str(microstep)+' A'+str(accelate)+'\n')

	def servoRun(self,port,angle):
		self.__writePackage('M41 D'+str(port)+' A'+str(angle)+'\n')
		
	def digitalWrite(self,pin,level):
		self.__writePackage('M11 D'+str(pin)+' L'+str(level)+'\n')

	def pwmWrite(self,pin,pwm):
		self.__writePackage('M11 D'+str(pin)+' P'+str(level)+'\n')

	def digitalRead(self,pin,callback):
		self.__doCallback('M12 D'+str(pin),callback)
		self.__writePackage('M12 D'+str(pin)+'\n')
	
	def analogRead(self,pin,callback):
		self.__doCallback('M13 A'+str(pin),callback)
		self.__writePackage('M13 A'+str(pin)+'\n')
	
	def onParse(self,msg):
		self.message=msg;
		if len(self.message)>3:
			if len(self.message.split("OK"))>1:
				if len(self.message.split("L"))>1:
					self.__selectors["callback_"+self.message[0:(self.message.index("L")-1)]](int(self.message.split("L")[1].split(" ")[0]));
				else:
					self.__selectors["callback_"+self.message[0:(self.message.index("OK")-1)]]();
		self.message = "";
				
	def readFloat(self, position):
		v = [self.buffer[position], self.buffer[position+1],self.buffer[position+2],self.buffer[position+3]]
		return struct.unpack('<f', struct.pack('4B', *v))[0]
	def readShort(self, position):
		v = [self.buffer[position], self.buffer[position+1]]
		return struct.unpack('<h', struct.pack('2B', *v))[0]
	def readString(self, position):
		l = self.buffer[position]
		position+=1
		s = ""
		for i in Range(l):
			s += self.buffer[position+i].charAt(0)
		return s
	def readDouble(self, position):
		v = [self.buffer[position], self.buffer[position+1],self.buffer[position+2],self.buffer[position+3]]
		return struct.unpack('<f', struct.pack('4B', *v))[0]

	def responseValue(self, extID, value):
		self.__selectors["callback_"+str(extID)](value)
		
	def __doCallback(self, extID, callback):
		self.__selectors["callback_"+str(extID)] = callback

	def float2bytes(self,fval):
		val = struct.pack("f",fval)
		return [ord(val[0]),ord(val[1]),ord(val[2]),ord(val[3])]

	def short2bytes(self,sval):
		val = struct.pack("h",sval)
		return [ord(val[0]),ord(val[1])]