from farmdroid import table_ssid_password_client, network_config, Local_socket
for ssid_password in table_ssid_password_client :
    	print(network_config.do_connect(ssid_password[0], ssid_password[1]))
Local_socket().start(IP_adress='0.0.0.0', port=80)
