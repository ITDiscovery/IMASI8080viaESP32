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

def shiftOut(dpin,cpin,order,idata):
  #MSBOrder is 1 always for us
  for i in range(7, -1, -1):
    GPIO.output(cpin,False)
    GPIO.output(dpin, (idata >> i) & 0x01 )    
    GPIO.output(cpin,True)
  return

def shiftIn(dpin,cpin,order):
    #LSBOrder is 1 always for us
    retval = 0
    for i in range(0, 8, 1):
        bitVal = GPIO.input(dpin)
        retval |= (bitVal << (7 - i))
        GPIO.output(cpin,True)
        time.sleep(.01)
        GPIO.output(cpin,False)
    return retval

while (True):

  #Write Pulse to Latch Pin
  GPIO.output(SWlPIN,False)
  time.sleep(.005)
  GPIO.output(SWlPIN,True)
  time.sleep(.005)

  # Get Address Low, Address High, Control Low, Control High
  addrsw = shiftIn(SWdPIN,SWcPIN,MSBFIRST)
  addrsw = addrsw + (shiftIn(SWdPIN,SWcPIN,MSBFIRST) << 8)
  ctrlsw = shiftIn(SWdPIN,SWcPIN,MSBFIRST)
  ctrlsw = ctrlsw + (shiftIn(SWdPIN,SWcPIN,MSBFIRST) << 8)
  GPIO.output(SWlPIN ,True)
  print("Address: ",addrsw," Control:",ctrlsw)
  time.sleep(.5)
