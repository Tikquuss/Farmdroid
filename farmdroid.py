# coding: utf-8
# Auteur : Pascal Tikeng 
# https://github.com/Tikquuss/Farmdroid
# https://github.com/Tikquuss/IoT

###
#  Les imports
###
try:
    # machine et esp sont les classes principale de l'esp
    import machine
    from machine import Pin, PWM, I2C, Timer
    import esp
    #désactiver les messages de débogage du système d'exploitation du fournisseur
    esp.osdebug(None)
    import esp32
    # capteur de temperature & humidité
    import dht
    # communication avec les microsockets
    try:
        import usocket as socket
    except:
        import socket
    # échanges de données
    import serial
    import network
    from network import mDNS, ftp, telnet, STA_IF, WLAN
    from machine import idle

    import json
    import urequests
    
    from time import sleep, time
    from math import cos, sin, acos, pi, sqrt, pow
    """
    Un ramasse-miettes est une forme de gestion automatique de la mémoire. 
    C'est un moyen de récupérer la mémoire occupée par des objets qui ne sont plus 
    utilisés par le programme. Ceci est utile pour économiser de l'espace dans la mémoire 
    flash.
    """
    import gc
    gc.collect()
except ImportError:
    # en cas d'erreur d'import on arrete
    raise ImportError("import not found")

### 
# Les parametres de configuration
###
chemin_fichier_html = "module3.html" # chemin du fichier html de référence
# les point d'acces des machines succeptibles de commander l'esp
table_ssid_password_client = [
    ['Wifi_Tikeng', 'aaaaaaaa']
]
servo_key = ['servo0', 'servo1'] # les clés des servos crées des arrosoires
mpu_key = ['mpu0'] # les clés des mpu crées
led_key = ['led0'] # les clés des leds crées
esc_key = ['moteur'] # les clés des esc crées
url_receive_data = "GET /ref" # url de réference pour l'envoi des données
url_receive_file = "GET /file" # url de réference pour l'envoi des fichiers
first_asking=True #apres une premiere requette elle passe à false

### 
# Les classes
###

class calcul:
    """
    Classe des calculs mathématiques
    Permet d'avoir la vitesse connaissant l'acceleration, ou la position connaissant la vitesse.
    Il etait impossible d'installer numpy sur l'esp pour faire les calculs mathematiques
    """
    def __init__(self):
        super().__init__(self)

    def norm(vecteur, dimension = 3) :
        norme = 0
        for i in range(dimension) :
            norme = norme + pow(vecteur[i], 2)
        return sqrt(norme)

    def matrice_passage(alpha, beta):
        alpha , beta = alpha*pi/180, beta*pi/180
        return [
                    [cos(alpha)*cos(beta), -cos(alpha)*sin(beta), -sin(alpha)],
                    [sin(beta), cos(beta), 0], 
                    [sin(alpha)*cos(beta), -sin(alpha)*cos(beta), cos(alpha)]
                ]
                    
    
    def dot(A, B, m = 3, n = 3):
        C = []
        for i in range(m):
            C.append(0)
            for k in range(n):
                C[i] += A[i][k]*B[k]
        return C

    def integration(f, initial_value, t1, t2):
        delta_t = t2-t1
        return [
                f[0]*delta_t+initial_value[0],
                f[1]*delta_t+initial_value[1],
                f[2]*delta_t+initial_value[2]
            ]

    def simpson(f_t1, f_milieu_t1_t2, f_t2, initial_value, t1, t2):
        delta_t = (t2-t1)/6
        return [
                delta_t*(f_t1[0]+4*f_milieu_t1_t2[0]+f_t2[0])+initial_value[0],
                delta_t*(f_t1[1]+4*f_milieu_t1_t2[1]+f_t2[1])+initial_value[1],
                delta_t*(f_t1[2]+4*f_milieu_t1_t2[2]+f_t2[2])+initial_value[2]
            ]

    def simpson_bon(a, b, n, f): 
        """méthode d'approximation de simpson""" 
        S = 0 
        for i in range(0, n): 
            x1 = a + (b - a) * i / float(n) 
            x2 = a + (b - a) * (i + 1) / float(n) 
            S += (f(x1) + 4 * f((x1 + x2) / 2.0) + f(x2)) * (x2 - x1) / 6.0 
        return S


class capteurs :
    """ température, humidité,... """
    def getAirTemperature():
        return esp32.raw_temperature()

    def getAirHumidity():
        d = dht.DHT22(machine.Pin(4))
        d.measure()
        return d.humidity() # eg. 41.3 (% RH)

    def getSoilTemperature():
        d = dht.DHT22(machine.Pin(4))
        d.measure()
        return d.temperature() # eg. 23 (°C)

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

###
# Actionneurs
###
class Servo(object):
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

class ESC(object):
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

class Arrosage(object):
    microServo = None
    moteur = None

    def start(self):
        pass

    def stop(self):
        pass


"""
* Un accéléromètre mesure l'accélération correcte, c'est-à-dire le taux de changement de vitesse par rapport 
à sa propre base de repos. Cela contraste avec l'accélération de coordonnées, qui est relative à un système
de coordonnées fixe. Le résultat pratique en est qu’au repos sur la Terre, un accéléromètre mesurera 
l’accélération due à la gravité de la Terre, de g ≈ 9,81 m / s. Un accéléromètre en chute libre mesurera 
zéro. Ceci peut être ajusté avec avec calibration.

* Un gyroscope mesure en revanche l'orientation et la vitesse angulaire, ou rotation autour d'un axe. 
La vitesse angulaire sera toujours nulle au repos.

La disponibilité de packs accéléromètre-gyroscope à puce unique bon marché les rend pratiques pour tout projet.
Dans ce projet j'utilise le MPU6050

Dans la suite : 

AcX : Accélération suivant l'axe X
AcY : Accélération selon l'axe Y
AcZ : Accélération selon l'axe Z
GyX : Rotation autour de l'axe X
GyY : Rotation autour de l'axe Y
GyZ : Rotation autour de l'axe Z
Tmp : Température °C

# Accelerometer/Gyroscope values are in int16 range (-32768 to 32767) If the mpu6050 loses power, 
# you have to call init() again
"""
class mpu6050(object):
    """Classes à utiliser pour controlers les packs accéléromètre-gyroscope"""
    def __init__(self, slc_pin_value, sda_pin_value, addr=0x68):      
        self.iic = I2C(scl=Pin(slc_pin_value), sda=Pin(sda_pin_value))
        self.addr = addr
        self.iic.start()
        self.iic.writeto(self.addr, bytearray([107, 0]))
        self.iic.stop()

    def get_raw_values(self):
        self.iic.start()
        a = self.iic.readfrom_mem(self.addr, 0x3B, 14)
        self.iic.stop()
        return a

    def get_ints(self):
        b = self.get_raw_values()
        c = []
        for i in b:
            c.append(i)
        return c

    def bytes_toint(self, firstbyte, secondbyte):
        if not firstbyte & 0x80:
            return firstbyte << 8 | secondbyte
        return - (((firstbyte ^ 255) << 8) | (secondbyte ^ 255) + 1)

    def get_values(self):
        raw_ints = self.get_raw_values()
        vals = {}
        vals["AcX"] = self.bytes_toint(raw_ints[0], raw_ints[1])
        vals["AcY"] = self.bytes_toint(raw_ints[2], raw_ints[3])
        vals["AcZ"] = self.bytes_toint(raw_ints[4], raw_ints[5])
        vals["Tmp"] = self.bytes_toint(raw_ints[6], raw_ints[7]) / 340.00 + 36.53
        vals["GyX"] = self.bytes_toint(raw_ints[8], raw_ints[9])
        vals["GyY"] = self.bytes_toint(raw_ints[10], raw_ints[11])
        vals["GyZ"] = self.bytes_toint(raw_ints[12], raw_ints[13])
        return vals  # returned in range of Int16
        # -32768 to 32767

    #lissage : lisser ces fluctuations aléatoires pour nous laisser de vraies données représentatives.
    #Il s'agit de lire plusieurs valeurs et de prendre la moyenne/médiane de toutes les valeurs. 
    #Le capteur renvoie plusieurs valeurs. Nous devons donc les calculer toutes individuellement.
    def get_smoothed_values(self, n_samples=10):
        """Obtenir des valeurs lissées du capteur par échantillonnage le capteur `n_samples` fois et retourne la moyenne"""
        result = {}
        for _ in range(n_samples):
            data =  self.get_values()
            for key in data.keys():
                # Ajouter de la valeur / des échantillons (pour générer une moyenne)
                # avec la valeur 0 par défaut pour la première boucle.
                result[key] = result.get(key, 0) + (data[key] / n_samples)
        return result
        
    #Etalonnage : si nous prenons un certain nombre de mesures de capteurs répétées dans le temps, 
    #nous pouvons déterminer l'écart type ou moyen par rapport à zéro dans le temps. 
    #Ce décalage peut ensuite être soustrait des mesures futures pour les corriger. 
    #L'appareil doit être au repos et ne pas changer pour que cela fonctionne de manière fiable.
    def calibrate(self, threshold=50, n_samples=100):
        """
        Obtenir la date d'étalonnage du capteur, en mesurant à plusieurs reprises tandis que le capteur est
        stable.  L'étalonnage résultant dictionnaire contient des décalages pour ce capteur dans sa
        position actuelle.
        """
        while True:
            v1 = self.get_smoothed_values(n_samples)
            v2 = self.get_smoothed_values(n_samples)
            # Vérifiez que toutes les mesures consécutives sont dans le seuil. Nous utilisons abs();  
            # donc tous les différences sont positives.
            if all(abs(v1[key] - v2[key]) < threshold for key in v1.keys()):
                return v1  # Calibrated.

    def get_smoothed_values_calibrate(self, n_samples=10, calibration=None):
        """
        Obtenir des valeurs lissées du capteur par échantillonnage le capteur `n_samples` fois et 
        retourne la moyenne. Si passé un dictionnaire `calibration`, soustrayez ces les valeurs 
        de la valeur finale du capteur avant de revenir.
        """    
        result = {}
        for _ in range(n_samples):
            data =  self.get_values()
            for key in data.keys():
                # Add on value / n_samples to produce an average
                # over n_samples, with default of 0 for first loop.
                result[key] = result.get(key, 0) + (data[key] / n_samples)

        if calibration: 
            # Remove calibration adjustment.
            for key in calibration.keys():
                result[key] -= calibration[key]

        return result

    def val_test(self):  # ONLY FOR TESTING! Also, fast reading sometimes crashes IIC
        from time import sleep
        while 1:
            print(self.get_values())
            sleep(0.05)

    
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
    
    def stop(self, afterbb = 0) :
        # arreter le LED apres after
        self._value=0
        sleep(after)
        machine.Pin(self._pin, machine.Pin.OUT).value(0)

class Batterie(object):
    """Classe de controle des batteries"""
    level = 0
    def __init__(self, level=0):
        self.level=level

    def get_level(self):
        import random
        return random.randint(0, 100)

    def set_level(self, level = 0):
        self.level = level
            
class network_config(object):
    def __init__(self):  
        super().__init__(self)

    def access_point_config(ssid) : # ssid='ESP-AP' par exemple
        ap = network.WLAN(network.AP_IF) # create access-point interface
        ap.config(ssid) # set the ESSID of the access point
        ap.active(True)  # activate the interface

    def do_connect(ssid, password):
        """Une fonction utile pour la connexion 脿 votre r茅seau WiFi local"""
        wlan = network.WLAN(network.STA_IF)
        wlan.active(True)
        if not wlan.isconnected():
            print('connecting to network...')
            wlan.connect(ssid, password)
            while not wlan.isconnected():
                pass
        print('Connection successful')
        print('network config:', wlan.ifconfig())
        return wlan.ifconfig()
        #todo : recuperer l'adresse ip (premier element), envoyer au client pour qu'il
        #lance son navigateur avec

    def config_DNS_ftp_telnet(e_ssid, password, username):
        mdns = mDNS()
        wlan = WLAN(STA_IF)
        wlan.active(True)
        nets = wlan.scan()
        for net in nets:
            ssid = net[0]
            if ssid == e_ssid.encode():
                wlan.connect(ssid, password)
                while not wlan.isconnected():
                    idle() # save power while waiting
                print('WLAN connection succeeded!')
                mdns.start('micropython', 'ensp_4gi_famdroid.com')
                ftp.start(user=username, password=password)
                telnet.start(user=username, password=password)
                break

class Utils :
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

class Flash(object):
    def flash_size() :
        return esp.flash_size()
    def flash_erase() :
        esp.flash_user_start()
    def flash_write(byte_offset, buffer) :
        esp.flash_write(byte_offset, buffer)
    def flash_read(byte_offset, buffer) :
        esp.flash_read(byte_offset, buffer)

def start() :
    """On deploit ici le module  2 (protocole,...)"""
    print('sleep is call \n')
    pass

def update():
    """Mise à jour des valeurs périodiquement"""
    print('update is call \n')
    pass

class Local_socket(object):
    """Classes à utiliser pour controlers les packs accéléromètre-gyroscope"""
    def __init__(self, socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)):
        #un socket STREAM TCP:
        self.socket = socket
    
    def start(self, IP_adress='', port=80):
        global periode
        #Ensuite, liez le socket à une adresse (interface réseau et numéro de port). 
        #la chaîne vide fait référence à l'adresse IP localhost"""
        try:
            self.socket.bind((IP_adress, port))
        except:
            self.socket.close()
        #listen() permet au serveur d'accepter les connexions; cela fait une prise "d'écoute". 
        self.socket.listen(5)
        start()
        #tim = Timer(-1)
        Timer(-1).init(period=int(1/periode), mode=Timer.PERIODIC, callback=lambda t:self.communicate())
        # equivalent à
        """
        while True :
            self.communicate()
            sleep(self.periode)  
        """

    def communicate(self):
        global led0, batterie, mpu0 
        global periode, mpu0, position
        global first_asking
        # Au premier appel on deploit le module 2
        if first_asking :
            start()
        try:
            """Lorsqu'un client se connecte, le serveur appelle la méthode accept() pour accepter la connexion. 
            Lorsqu'un client se connecte, il enregistre un nouvel objet socket pour accepter et envoyer 
            des données sur la variable conn, ainsi que l'adresse du client pour se connecter au serveur
            via la variable addr."""
            conn, addr = self.socket.accept()
            #Imprimez ensuite l'adresse du client enregistr茅e dans la variable addr.
            print('Got a connection from %s' % str(addr))
            """Les données sont échangées entre le client et le serveur à l'aide des méthodes 
            send() et recv(). On récupère la demande recue sur le socket nouvellement créee et 
            la sauvegarde dans la variable request"""
            request = conn.recv(1024)
            request = str(request)
            """
            La methode recv() recoit les donnees du socket client (rappelez-vous que nous avons 
            creee un nouvel objet socket sur la variable conn). L'argument de la methode recv() 
            specifie le maximum de donnees pouvant etre recues simultanement."""
            #La ligne suivante affiche simplement le contenu de la requete:
            print('Content = %s' % request)
          
            # Pour les leds
            if led_key[0] in request :
                if led_key[0]+'=on' in request :
                    print('LED ON')
                    led0.value(1)
                if led_key[0]+'=off' in request :
                    print('LED OFF')
                    led0.value(0)
                self.response(conn)
            
            # Pour les esc
            
            if "moteur" in request :
                if esc_key[0] in request :
                val = request.split(esc_key[0]+"=")[1].split('&')[0]
                try:
                    val = int(val)
                except:
                    val = 0
                #esc._set_puissance(val)
                #esc.turn()
                print("puissance de l'esc : "+str(val)+"\n")
            
            # Pour les MicroServo
            if "servo" in request :
                if servo_key[0] in request:
                    #val = request.split('/?value=')[1].split('&')[0]
                    val = request.split(servo_key[0]+"=")[1].split('&')[0]
                    try:
                        val = int(val)
                    except:
                        val = 0
                    s1._set_angle(val)
                    s1.turn()
                    print("angle du servo : "+str(val)+"\n")

                if servo_key[1] in request :
                    #val = request.split('/?value=')[1].split('&')[0]
                    val = request.split(servo_key[1]+"=")[1].split('&')[0]
                    try:
                        val = int(val)
                    except:
                        val = 0
                    s2._set_angle(val)
                    s2.turn()
                    print("angle du servo : "+str(val)+"\n")
                self.response(conn)
            
            if url_receive_data in request:
                data = {}
                data['mpu0'] = mpu0.get_values()
                data['servo1'] = s1._get_angle()
                data['servo2'] = s2._get_angle()
                data['batterie'] = batterie.get_level()

                response = json.dumps(data).encode()
                conn.send(b'HTTP/1.1 200 OK\n')
                conn.send(b'Content-Type: application/json\n')
                conn.send(b'Connection: close\n\n')
                conn.sendall(response)
                print("envoi des données reussi\n")
                #En fin de compte, fermez le socket creee.
                #conn.close()
                #continue
            
            if url_receive_file in request :  
                val = request.split(url_receive_file+"/")[1].split(" ")[0] 
                response = self.web_page(file_name = val)
                if response == '404' :
                    conn.send(b'HTTP/1.1 200 404\n')
                else :
                    conn.send(b'HTTP/1.1 200 OK\n')
                    if val.split(".")[-1] == "js" :
                        conn.send(b'Content-Type: application/javascript\n')
                    if val.split(".")[-1] == "css" :
                        conn.send(b'Content-Type: text/css\n')
                    conn.send(b'Connection: close\n\n')
                    conn.sendall(response.encode())

            if first_asking :
                #self.response(conn)
                #Creez ensuite une variable appelee response contenant le texte HTML client
                response = self.web_page(chemin_fichier_html).encode()
                #Enfin, envoyez la reponse au client de socket à l'aide des methodes send() et sendall():
                conn.send(b'HTTP/1.1 200 OK\n')
                conn.send(b'Content-Type: text/html\n')
                conn.send(b'Connection: close\n\n')
                conn.sendall(response)
                #En fin de compte, fermez le socket cr茅茅.
                conn.close()
                first_asking=False
        
        except Exception as ex:
            #self.communicate()
            self.socket.close()
            first_asking=True
            #import traceback
            #traceback.print_tb(ex.__traceback__)
            print(ex)
            print('communication failed')
            import sys
            sys.exit(1)
        update()

    def response(self, conn) :
        print('response is call \n')
        conn.send(b'HTTP/1.1 200 OK\n')
        conn.send(b'Content-Type: application/json\n')
        conn.send(b'Connection: close\n\n')
        conn.sendall(b'OK')

    def web_page(self, file_name):
        print('web_page is call \n')
        #global mpu0, led0
        try : 
            return open(file_name, "r").read()
            #return b"<html><head> <title>ESP</title> </head><body> <h1>ESP</h1></body></html>"
            #return self.manage_web_page(html = open(chemin_fichier_html, "r").read(), 
                                    #mpu_value=mpu0.get_values(), 
                                    #gpio_state = "ON" if led0.value() == 1 else "OFF")
        except:
            return '404'
        
    def manage_web_page(self, html, mpu_value, gpio_state):
        #mettre à jour les valeurs incluses dans html
        print('manage_web_page is call \n')
        return html.replace("gpio_state", gpio_state
            ).replace(
                "mpu_value_AcX", str(mpu_value["AcX"])
            ).replace(
                "mpu_value_AcY", str(mpu_value["AcY"])
            ).replace(
                "mpu_value_AcZ", str(mpu_value["AcZ"])
            ).replace(
                "mpu_value_GyX", str(mpu_value["GyX"])
            ).replace(
                "mpu_value_GyY", str(mpu_value["GyY"])
            ).replace(
                "mpu_value_GyZ", str(mpu_value["GyZ"])
            ).replace(
                "mpu_value_Tmp", str(mpu_value["Tmp"])
            ) 
"""          
for ssid_password in table_ssid_password_client :
	print(network_config.do_connect(ssid_password[0], ssid_password[1]))
Local_socket().start(IP_adress='0.0.0.0', port=80)
"""