import RPi.GPIO as GPIO
import time, random

LEDdPIN = 36
LEDlPIN = 38
LEDcPIN = 40
SWdPIN = 35
SWlPIN = 33
SWcPIN = 37
MSBFIRST = 1

GPIO.setmode(GPIO.BOARD)
GPIO.setup((LEDdPIN,LEDlPIN,LEDcPIN,SWdPIN, SWcPIN, SWlPIN), GPIO.OUT)
GPIO.setup(SWdPIN,GPIO.IN)

#shiftOut(LdataPin,LclockPin,MSBFIRST,bus.state >> 8);
def shiftOut(dpin,cpin,order,idata):
  #MSBOrder is 1 always for us
  for i in range(7, -1, -1):
    GPIO.output(cpin,False)
    GPIO.output(dpin, (idata >> i) & 0x01 )    
    GPIO.output(cpin,True)
  return

while (True):
  busdata = random.randrange(0xFF)
  busaddress = random.randrange(0xFFFF)
  busstate = random.randrange(0xFFFF)
  GPIO.output(LEDlPIN ,False)
  shiftOut(LEDdPIN,LEDcPIN,MSBFIRST,busstate >> 8)
  shiftOut(LEDdPIN,LEDcPIN,MSBFIRST,busaddress >> 8)
  shiftOut(LEDdPIN,LEDcPIN,MSBFIRST,busaddress)
  shiftOut(LEDdPIN,LEDcPIN,MSBFIRST,busstate)
  shiftOut(LEDdPIN,LEDcPIN,MSBFIRST,busdata)
  GPIO.output(LEDlPIN ,True)
  time.sleep(.5)

GPIO.cleanup()
