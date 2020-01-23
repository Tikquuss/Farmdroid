from farmdroid import Led, Servo, Batterie, mpu6050
###
# Initialisation des variables
###

# pédiode d'envoi des données
periode =  1/100

led0 = Led(pin = 2, sleep=0, value = 0)
mpu0 = mpu6050(slc_pin_value = 5, sda_pin_value = 15) # VCC=3V3, GND=GND
s1 = Servo(angle = 0, pin=13, frequence=50)
s2 = Servo(angle = 0, pin=12, frequence=50)
batterie = Batterie() 