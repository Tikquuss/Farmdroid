import esp
import serial
"""
- Appeler deepsleep() sans argument mettra l'appareil en veille indéfiniment
- Une réinitialisation du logiciel ne change pas la cause de la réinitialisation
- Il peut y avoir un courant de fuite traversant les tractions internes activées. 
  Pour réduire davantage la consommation d'énergie, il est possible de désactiver les 
  trames internes:
  p1 = Pin(4, Pin.IN, Pin.PULL_HOLD)

 Après avoir dormi profondément, il peut être nécessaire de déverrouiller explicitement 
 la  broche (par exemple, s'il s'agit d'une broche de sortie) via:
 p1 = Pin(4, Pin.OUT, None)

 The following code can be used to sleep, wake and check the reset cause:
"""

import machine
from machine import Pin

def deep_sleep(t):
    """t en milliseconde"""
    # check if the device woke from a deep sleep
    if machine.reset_cause() == machine.DEEPSLEEP_RESET:
        print('woke from a deep sleep')
    else:
        # put the device to sleep for 10 seconds
        try :
            machine.deepsleep(t)
        except :
            print('error')

def disablePin(pin_number):
    try :
        Pin(pin_number, Pin.IN, Pin.PULL_HOLD)
    except :
        print('error')

def enablePin(pin_number):
    try :
        Pin(pin_number, Pin.OUT, None)
    except :
        print('error')
  
def scan():
    """scan for available ports. return a list of tuples (num, name)"""
    available = []
    for i in range(0,256):
        try:
            s = serial.Serial('COM'+str(i))
            available.append((i, s.portstr))
            s.close()
        except serial.SerialException:
            pass
    return available

def open_port() :
  a=serial.Serial()
  a.setPort('COM3')
  #a.setPort(2)  
  a.open()
  variable = a.read(n) # n est le nombre de bits qu'on veut lire
  a.close

class Led(object):
    """Classes à utiliser pour controlers les LEDS : utiliser des resistances"""
    _pin = 2
    _sleep = 0 #pas de clignotement (0 pour le continue)
    _value = 0 #1 si allumé, 0 si éteint

    def __init__(self, pin=2, sleep=0, value = 0):
        self._pin=pin
        self._sleep=sleep
        if value == 1 :
            self.bling()

    def get_value(self):
        return self._value

    def set_value(self, value):
        #self._value = value
        if value != self._value :
            if value == 0 :
                self.stop()
            else :
                self.bling()

    def value(self, value = None) :
        if(value == None) :
            return self._value
        else :
            #self._value = value
            if value != self._value :
                if value == 0 :
                    self.stop()
                else :
                    self.bling()
        
    def bling(self) :
        if(self._sleep == 0) :
            self._value=1
            machine.Pin(self._pin, machine.Pin.OUT).value(1)
        else :
            led = Pin(self._pin, Pin.OUT)
            self._value = led.value()
            while True :
                led.value(not led.value())
                self._value = led.value()
                sleep(self._sleep)
    
    def stop(self, after = 0) :
        # arreter le LED apres after
        self._value=0
        sleep(after)
        machine.Pin(self._pin, machine.Pin.OUT).value(0)

class Flash(object):
    def flash_size() :
        return esp.flash_size()
    def flash_erase() :
        esp.flash_user_start()
    def flash_write(byte_offset, buffer) :
        esp.flash_write(byte_offset, buffer)
    def flash_read(byte_offset, buffer) :
        esp.flash_read(byte_offset, buffer)
