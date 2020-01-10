"""
-- Sources + montages
https://wiki.mchobby.be/index.php?title=MicroPython-Hack-moteur

https://arduino103.blogspot.com/2015/12/micropython-commander-un-moteur-via.html

"""

# Contrôle d'un moteur jouet à l'aide d'un transistor.
from pyb import Timer, delay

# Brancher le moteur sur X3
MOTOR_PIN = pyb.Pin.board.X3 

# fonction qui permet de passer d'un range de valeur (in_) à une autre
#    (out_) en appliquant une règle de trois. 
def arduino_map(x, in_min, in_max, out_min, out_max):
    return int( (x - in_min) * (out_max - out_min) // (in_max - in_min) + out_min ) 
    
# Eteint simplement le moteur 
def motorOff():
	onTime  = 2500 # temps allumé 
	offTime = 1000 # temps éteint
	
	p = pyb.Pin( MOTOR_PIN, pyb.Pin.OUT_PP )
	p.low()
		
# Allume puis éteind le moteur a interval régulier 
def motorOnThenOff():
	onTime  = 2500 # temps allumé 
	offTime = 1000 # temps éteint
	
	p = pyb.Pin( MOTOR_PIN, pyb.Pin.OUT_PP )
    # Répéter l'opération encore et encore
	while True:
		p.high()
		delay( onTime )
		p.low()
		delay( offTime )
		
# Contrôle le moteur avec une gestion de la vitesse
def motorOnThenOffWithSpeed():
    # Creer un timer à une fréquence de 100 Hz (le timer 5)
    # Créer un canal (channel) PWM avec le Timer.
    #   Voyez le graphique PyBoard, sur PIN_MOTOR (X3), il faut utiliser
    #   "Channel 3" sur le "Timer 5"  
    tim = pyb.Timer( 5, freq=100)
    tchannel = tim.channel(3, Timer.PWM, pin=MOTOR_PIN, pulse_width=0)

    # Minimum et Maximum de largeur d'impulsion correspondant au minimum
    # et maximum de signal
    max_width = 200000
    min_width = 20000
    
    onTime = 2500  # temps allumé (a vitesse max)
    offTime = 2500 # temps eteind (a vitesse min)
    
    onSpeed  = 250  # vitesse max (entre 0 et 255)
    offSpeed = 150   # vitesse min (entre 0 et 255)
    
    # Répéter l'opération encore et encore
    while True:
        tchannel.pulse_width( arduino_map( onSpeed, 0, 255, min_width, max_width ) )
        delay( onTime )
        tchannel.pulse_width( arduino_map( offSpeed, 0, 255, min_width, max_width ) )
        delay( offTime )

# Contrôler un moteur avec accéleration
def motorWithAcceleration():
    # Creer un timer à une fréquence de 100 Hz (le timer 5)
    # Créer un canal (channel) PWM avec le Timer.
    #   Voyez le graphique PyBoard, sur PIN_MOTOR (X3), il faut utiliser
    #   "Channel 3" sur le "Timer 5"  
    tim = pyb.Timer( 5, freq=100)
    tchannel = tim.channel(3, Timer.PWM, pin=MOTOR_PIN, pulse_width=0)

    # Minimum et Maximum de largeur d'impulsion correspondant au minimum
    # et maximum de signal
    max_width = 200000
    min_width = 20000
    
    delay_time = 50 # temps entre deux modification
    
    # Répéter l'opération encore et encore
    while True:   
	    # boucler toutes les vitesses de 255 à 150
	    for speed in range( 255, 150, -1 ):
			tchannel.pulse_width( arduino_map( speed, 0, 255, min_width, max_width ) )
			delay( delay_time ) 
        # boucler toutes les vitesses de 150 à 255
	    for speed in range( 150, 256 ):
		    tchannel.pulse_width( arduino_map( speed, 0, 255, min_width, max_width ) )
		    delay( delay_time )
    
# motorOff()
motorOnThenOff()
# motorOnThenOffWithSpeed()
# motorWithAcceleration()