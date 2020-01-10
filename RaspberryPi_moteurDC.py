"""
-- source 
http://electroniqueamateur.blogspot.com/2014/09/controler-un-moteur-dc-en-python-avec.html
"""

## Controle d'un moteur DC par le Raspberry Pi
# Plus de details:
# http://electroniqueamateur.blogspot.com/2014/09/controler-un-moteur-dc-en-python-avec.html

import RPi.GPIO as GPIO
from time import sleep

GPIO.setmode(GPIO.BOARD)   ##je prefere la numerotation BOARD plutot que BCM

Moteur1A = 16      ## premiere sortie du premier moteur, pin 16
Moteur1B = 18      ## deuxieme sortie de premier moteur, pin 18
Moteur1E = 22      ## enable du premier moteur, pin 22

GPIO.setup(Moteur1A,GPIO.OUT)  ## ces trois pins du Raspberry Pi sont des sorties
GPIO.setup(Moteur1B,GPIO.OUT)
GPIO.setup(Moteur1E,GPIO.OUT)

pwm = GPIO.PWM(Moteur1E,50)   ## pwm de la pin 22 a une frequence de 50 Hz
pwm.start(100)   ## on commemnce avec un rapport cyclique de 100%


print ("Rotation sens direct, vitesse maximale (rapport cyclique 100%)")
GPIO.output(Moteur1A,GPIO.HIGH)
GPIO.output(Moteur1B,GPIO.LOW)
GPIO.output(Moteur1E,GPIO.HIGH)

sleep(5)  ## on laisse tourner le moteur 5 secondes avec des parametres

pwm.ChangeDutyCycle(20)  ## modification du rapport cyclique a 20%

print("Rotation sens direct, au ralenti (rapport cyclique 20%)")

sleep(5)

print("Rotation sens inverse, au ralenti (rapport cyclique 20%)")
GPIO.output(Moteur1A,GPIO.LOW)
GPIO.output(Moteur1B,GPIO.HIGH)

sleep(5)

pwm.ChangeDutyCycle(100)
print("Rotation sens inverse, vitesse maximale (rapport cyclique 100%)")
sleep(5)


print("Arret du moteur")
GPIO.output(Moteur1E,GPIO.LOW)

pwm.stop()    ## interruption du pwm

GPIO.cleanup()
