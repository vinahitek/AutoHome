#ifndef _USER_CONFIG_H_
#define _USER_CONFIG_H_

#define CFG_HOLDER	0x00FF55A4	/* Change this value to load default configurations */
#define CFG_LOCATION	0x7C	/* Please don't change or if you know what you doing */
#define CLIENT_SSL_ENABLE

/*DEFAULT CONFIGURATIONS*/

#define MQTT_HOST			"m12.cloudmqtt.com" //or "mqtt.yourdomain.com"
#define MQTT_PORT			17935
#define MQTT_BUF_SIZE		1024
#define MQTT_KEEPALIVE		5	 /*second*/

#define MQTT_CLIENT_ID		"HungVo"
#define MQTT_USER			"lzwrnmwu"
#define MQTT_PASS			"qODam8loOrFl"

#define STA_SSID "Innova-4"
#define STA_PASS "Inn0v@VN"
#define STA_TYPE AUTH_WPA2_PSK

#define MQTT_RECONNECT_TIMEOUT 	120	/*second*/

#define DEFAULT_SECURITY	0
#define QUEUE_BUFFER_SIZE		 		2048

#define PROTOCOL_NAMEv31	/*MQTT version 3.1 compatible with Mosquitto v0.15*/
//PROTOCOL_NAMEv311			/*MQTT version 3.11 compatible with https://eclipse.org/paho/clients/testing/*/
#endif
