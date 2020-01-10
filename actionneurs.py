try:
    from machine import Pin, PWM
    import esp
    esp.osdebug(None)
    import gc
    gc.collect()
except ImportError:
    raise ImportError("import not found")

class microServo(object):
    """ Classes à utiliser pour controler les Microservo"""
    _angle = 0
    _pin = 13
    _frequence = 50

    def __init__(self, angle = 0, pin=13, frequence=50):
        self._angle=angle
        self._pin=pin
        self._frequence=frequence
        self.servo = PWM(Pin(self._pin), self._frequence)
    
    def _get_angle(self):
        return self._angle

    def _set_angle(self, angle):
        self._angle = angle

    angle=property(_get_angle, _set_angle)
    
    def _get_pin(self):
        return self._pin

    def _set_pin(self, pin):
        self._pin = pin

    pin=property(_get_pin, _set_pin)
    
    def _get_frequence(self):
        return self._frequence

    def _set_frequence(self, frequence):
        self._frequence = frequence

    frequence=property(_get_frequence, _set_frequence)
    
    def turn(self, angle = None) :
        """Fonction qui place le servo connect茅 au pin 脿 une position de angle degr茅e"""
        if(angle != None & angle != self._angle) :
            self._angle = angle
            self.servo.duty(self._angle)

class Moteur(object):
    """ Classes à utiliser pour controler les moteurs"""
    _puissance = 0
    _pin = 13
    _frequence = 50

    def __init__(self, puissance = 0, pin=13, frequence=50):
        self._puissance=puissance
        self._pin=pin
        self._frequence=frequence
        self.esc = PWM(Pin(self._pin), self._frequence)    
    
    def _get_puissance(self):
        return self._puissance

    def _set_puissance(self, puissance):
        self._puissance = puissance

    puissance=property(_get_puissance, _set_puissance)
    
    def _get_pin(self):
        return self._pin

    def _set_pin(self, pin):
        self._pin = pin

    pin=property(_get_pin, _set_pin)
    
    def _get_frequence(self):
        return self._frequence

    def _set_frequence(self, frequence):
        self._frequence = frequence

    frequence=property(_get_frequence, _set_frequence)

    def turn(self, puissance = None) :
        """Fonction qui place le servo connect茅 au pin 脿 une position de angle degr茅e"""
        if(puissance & puissance != self._puissance) :
            self._puissance = puissance
            self.esc.duty(self._puissance*60/100+40)