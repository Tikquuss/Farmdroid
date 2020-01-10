import dht
import machine
import machine
import urequests
import time

def getTemperature():
    d = dht.DHT22(machine.Pin(4))
    d.measure()
    return d.temperature() # eg. 23 (°C)

def getAirHumidity():
    d = dht.DHT22(machine.Pin(4))
    d.measure()
    return d.humidity() # eg. 41.3 (% RH)

def getSoilHumidity(pin_number=0):
    rtc = machine.RTC() # Clock for deepsleep
    rtc.irq(trigger=rtc.ALARM0, wake=machine.DEEPSLEEP)
    adc = machine.ADC(pin_number) # Pin to Read sensor voltage
    try:

        ######################
        # Sensor calibration #
        ######################

        # values on right are inverse * 1000 values on left
        # dry air = 759 (0%) = 1.31752305665349143610013175231
        # water = 382 (100%) = 2.61780104712041884816753926702
        # The Difference     = 1.30027799046692741206740751471
        # 1 %                = 0.0130027799046692741206740751471

        SoilMoistVal = (((1 / adc.read())* 1000) / 0.0130027799046692741206740751471) - 101
        if SoilMoistVal > 100:
            SoilMoistVal = 100
        if SoilMoistVal < 0:
            SoilMoistVal = 0

        return SoilMoistVal
    except:
        machine.reset()