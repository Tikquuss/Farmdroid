"""
Source
https://github.com/Tikquuss/Drone/blob/micropython/DHT_driver.py
"""

# Le pilote DHT est implémenté dans le logiciel et fonctionne sur toutes les broches :
import dht
import machine

d = dht.DHT11(machine.Pin(4))
d.measure()
print(d.temperature()) # eg. 23 (°C)
print(d.humidity())    # eg. 41 (% RH)

d = dht.DHT22(machine.Pin(4))
d.measure()
print(d.temperature()) # eg. 23.6 (°C)
print(d.humidity())    # eg. 41.3 (% RH)