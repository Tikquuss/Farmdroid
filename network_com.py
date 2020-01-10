try:
    from machine import Pin, PWM, I2C, Timer
    from network import mDNS, ftp, telnet, STA_IF, WLAN
    from machine import idle
    from time import sleep, time
    try:
        import usocket as socket
    except:
        import socket

    import network
    import json
    #désactiver les messages de débogage du système d'exploitation du fournisseur:
    import esp
    esp.osdebug(None)
    import gc
    gc.collect()
except ImportError:
    raise ImportError("import not found")

class network_config(object):
    def __init__(self):  
        super().__init__(self)

    def access_point_config(ssid, password) : # ssid='ESP-AP' par exemple
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

    def config_DNS_ftp_telnet():
        mdns = mDNS()
        wlan = WLAN(STA_IF)
        wlan.active(True)
        nets = wlan.scan()
        for net in nets:
            ssid = net[0]
            if ssid == b'YOUR_SSID':
                wlan.connect(ssid, 'YOUR_PASSWORD')
                while not wlan.isconnected():
                    idle() # save power while waiting
                print('WLAN connection succeeded!')
                mdns.start('mPy', 'MicroPython ESP32')
                ftp.start(user='YOUR_USERNAME', password='YOUR_PASSWORD')
                telnet.start(user='YOUR_USERNAME', password='YOUR_PASSWORD')
                break

class Local_socket(object):
    """Classes à utiliser pour controlers les packs accéléromètre-gyroscope"""
    def __init__(self, 
                socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM),
                data_dic,
                periode = 1 # en seconde
                ):

        #un socket STREAM TCP:
        self.socket = socket
        self.data = data_dic
        self.periode = periode
        self.in_communication = True
    
    def setData_dic(self, data_dic):
         self.data = data_dic

    def start(self, IP_adress='', port=80):
        #Ensuite, liez le socket à une adresse (interface réseau et numéro de port). 
        #la chaîne vide fait référence à l'adresse IP localhost"""
        try:
            self.socket.bind((IP_adress, port))
        except:
            self.socket.close()
        #listen() permet au serveur d'accepter les connexions; cela fait une prise "d'écoute". 
        self.socket.listen(5)
        try:
            """Lorsqu'un client se connecte, le serveur appelle la méthode accept() pour accepter la connexion. 
            Lorsqu'un client se connecte, il enregistre un nouvel objet socket pour accepter et envoyer 
            des données sur la variable conn, ainsi que l'adresse du client pour se connecter au serveur
            via la variable addr."""
            self.conn, addr = self.socket.accept()
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
        except :
            self.socket.close()
            #import traceback
            #traceback.print_tb(ex.__traceback__)
            print(ex)
            print('communication failed')
            import sys
            sys.exit(1)

        while self.in_communication :
            self.communicate()
            sleep(self.periode)           
            
    def stop(self):
        self.in_communication = False
        self.conn.close()

    def communicate(self):
        try:
            response = json.dumps(self.data).encode()
            self.conn.send(b'HTTP/1.1 200 OK\n')
            self.conn.send(b'Content-Type: application/json\n')
            self.conn.send(b'Connection: close\n\n')
            self.conn.sendall(response)
            print("envoi des données reussi\n")
        except Exception as ex:
            self.socket.close()
            #import traceback
            #traceback.print_tb(ex.__traceback__)
            print(ex)
            print('communication failed')
            import sys
            sys.exit(1)