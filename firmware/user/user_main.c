/* main.c -- MQTT client example
*
* Copyright (c) 2014-2015, Tuan PM <tuanpm at live dot com>
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* * Redistributions of source code must retain the above copyright notice,
* this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in the
* documentation and/or other materials provided with the distribution.
* * Neither the name of Redis nor the names of its contributors may be used
* to endorse or promote products derived from this software without
* specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*/
#include "ets_sys.h"
#include "driver/uart.h"
#include "osapi.h"
#include "mqtt.h"
#include "wifi.h"
#include "config.h"
#include "debug.h"
#include "gpio.h"
#include "user_interface.h"
#include "mem.h"
#include "driver/gpio16.h"

MQTT_Client mqttClient;
void  intr_callback(unsigned pin, unsigned level);
void LedCtrl(void *pclient,void *args,void*TopicVal);
typedef enum _enum_espio
{
	ESP_GPIO16,
	ESP_GPIO5,
	ESP_GPIO4,
	ESP_GPIO0,
	ESP_GPIO2,
	ESP_GPIO14,
	ESP_GPIO12,
	ESP_GPIO13,
	ESP_GPIO15,
	ESP_GPIO3,
	ESP_GPIO1,
	ESP_GPIO9,
	ESP_GPIO10,
}enum_espio;

#define GPIO_LED_PIN_1 	ESP_GPIO14
#define GPIO_LED_PIN_2 	ESP_GPIO16
#define GPIO_LED_PIN_3 	ESP_GPIO13

#define GPIO_LED_PIN   	ESP_GPIO0
#define GPIO_BUTTON1	ESP_GPIO12 /*PIO12*/



typedef enum _enumiotype
{
	espio_led=GPIO_OUTPUT,
	espio_button=GPIO_INT,
	//espio_sensor,
	espio_unknow=0xff
}enumiotype;
typedef struct _structUserIO
{
	enum_espio 		io;
	enumiotype 		type;
    char*		ptopic;
	void (*f_cb)(void *pclient,void *args,void*TopicVal);
}structUserIO;
#define TOPIC_KEEPALIVE	"/mqtt/led/live"

structUserIO  strtuserioprofiles[]=
{
		{GPIO_LED_PIN_1,espio_led,"/mqtt/led/1",LedCtrl},
		{GPIO_LED_PIN_2,espio_led,"/mqtt/led/2",LedCtrl},
		{GPIO_LED_PIN_3,espio_led,"/mqtt/led/3",LedCtrl},

		{GPIO_LED_PIN,espio_led,TOPIC_KEEPALIVE,NULL},

		{GPIO_BUTTON1,espio_button,"/mqtt/button/1",NULL},

		{0,espio_unknow,0,0}
};

ICACHE_FLASH_ATTR structUserIO *__espio_getstructprofile(enum_espio io)
{
	int i=0;
	for( i=0;strtuserioprofiles[i].type!=espio_unknow;i++)
	{
		if(strtuserioprofiles[i].io == io)
			return &strtuserioprofiles[i];
	}
	return NULL;
}


void LedCtrl(void *pclient,void *args,void*TopicVal)
{
	structUserIO *pstrtUserIO=args;
	bool bitval =false;
	if(*(char*)TopicVal =='1')
	{
		bitval=true;
	}
	INFO("IO %d Val %d",pstrtUserIO->io,bitval);
	gpio_write(pstrtUserIO->io,bitval);
}

ICACHE_FLASH_ATTR void ButtonCtrl(enum_espio io)
{
	structUserIO *pstrtUserIO=__espio_getstructprofile(io);
	if(pstrtUserIO==NULL) return;
	INFO("Publish topic: %s, data: %s \r\n", pstrtUserIO->ptopic,"pressed");
	MQTT_Publish(&mqttClient, pstrtUserIO->ptopic,"pressed", 7, 1, 0);
}




void UserIOImplementCallback(MQTT_Client*pclient,char* topicBuf, char* dataBuf)
{
	int i=0;
	INFO("Receive topic: %s, data: %s \r\n", topicBuf, dataBuf);
	structUserIO*pstrtuserio =pclient->pstrtUserIOProfiles;
	for(i=0;pstrtuserio[i].type!=espio_unknow;i++)
	{
		if(pstrtuserio[i].f_cb==NULL) continue;

		if(strcmp(pstrtuserio[i].ptopic,topicBuf))
		{
			INFO("implement callback ",pstrtuserio[i].ptopic,dataBuf);
			pstrtuserio[i].f_cb(pclient,&pstrtuserio[i],dataBuf);
			return;
		}
	}
	INFO("Not Implement Callback ");
}

void config_peripheral(MQTT_Client*pclient)
{
	int i=0;
	structUserIO*pstrtuserio =pclient->pstrtUserIOProfiles;
	if(pstrtuserio!=NULL)
	{
		INFO("Configure Peripheral");
		int iqs=1;
		for( i=0;pstrtuserio[i].type!=espio_unknow;i++)
		{
			set_gpio_mode(pstrtuserio[i].io, GPIO_PULLUP, pstrtuserio[i].type);
			if(pstrtuserio[i].type==espio_button)
			{
					INFO("Config Interrupt");
					gpio_intr_init(GPIO_BUTTON1, GPIO_PIN_INTR_NEGEDGE);
					gpio_intr_attach(intr_callback);
			}
			//if(pstrtuserio[i].type==espio_led)
			//{
				if(pstrtuserio[i].ptopic!=NULL)
				{
					INFO("MQTT_Subscribe %s",pstrtuserio[i].ptopic);
					MQTT_Subscribe(pclient,pstrtuserio[i].ptopic,iqs++);
					//MQTT_Publish(pclient,pstrtuserio[i].ptopic, "test11", 6, 1, 0);
					//	MQTT_Publish(client, "/mqtt/topic/2", "hello2", 6, 2, 0);
				}
			//}
		}
	}
}


enum_espio flasingledio = GPIO_LED_PIN;
void ToggleLed(void*client)
{

	INFO("\r\nToggleLed\r\n");
	gpio_write(GPIO_LED_PIN, ~gpio_read(GPIO_LED_PIN) );
	MQTT_Client* pclient = (MQTT_Client*)client;

   if(pclient->keepAliveTick > pclient->mqtt_state.connect_info->keepalive)
   {
	   MQTT_Publish(client, TOPIC_KEEPALIVE, "1",1, 0, 0);
   }

}
void wifiConnectCb(uint8_t status)
{
	if(status == STATION_GOT_IP){
		MQTT_Connect(&mqttClient);
	} else {
		MQTT_Disconnect(&mqttClient);
	}
}
void mqttConnectedCb(uint32_t *args)
{
	MQTT_Client* client = (MQTT_Client*)args;
	INFO("MQTT: Connected\r\n");
	config_peripheral(client);
//	MQTT_Subscribe(client, LED_TOPIC, 0);
//	MQTT_Subscribe(client, SWITCH_TOPIC, 1);
//	MQTT_Subscribe(client, "/mqtt/topic/2", 2);

//	MQTT_Publish(client, LED_TOPIC, "hello0", 6, 0, 0);
//	MQTT_Publish(client, "/mqtt/topic/1", "hello1", 6, 1, 0);
//	MQTT_Publish(client, "/mqtt/topic/2", "hello2", 6, 2, 0);

}

void mqttDisconnectedCb(uint32_t *args)
{
	MQTT_Client* client = (MQTT_Client*)args;
	INFO("MQTT: Disconnected\r\n");
}

void mqttPublishedCb(uint32_t *args)
{
	MQTT_Client* client = (MQTT_Client*)args;
	INFO("MQTT: Published\r\n");
}

void mqttDataCb(uint32_t *args, const char* topic, uint32_t topic_len, const char *data, uint32_t data_len)
{
	char *topicBuf = (char*)os_zalloc(topic_len+1),
			*dataBuf = (char*)os_zalloc(data_len+1);



	MQTT_Client* client = (MQTT_Client*)args;

	os_memcpy(topicBuf, topic, topic_len);
	topicBuf[topic_len] = 0;

	os_memcpy(dataBuf, data, data_len);
	dataBuf[data_len] = 0;



	UserIOImplementCallback(client,topicBuf,dataBuf);

	os_free(topicBuf);
	os_free(dataBuf);
}

uint32 ICACHE_FLASH_ATTR user_rf_cal_sector_set(void)
{
    enum flash_size_map size_map = system_get_flash_size_map();
    uint32 rf_cal_sec = 0;

    switch (size_map) {
        case FLASH_SIZE_4M_MAP_256_256:
            rf_cal_sec = 128 - 8;
            break;

        case FLASH_SIZE_8M_MAP_512_512:
            rf_cal_sec = 256 - 5;
            break;

        case FLASH_SIZE_16M_MAP_512_512:
        case FLASH_SIZE_16M_MAP_1024_1024:
            rf_cal_sec = 512 - 5;
            break;

        case FLASH_SIZE_32M_MAP_512_512:
        case FLASH_SIZE_32M_MAP_1024_1024:
            rf_cal_sec = 1024 - 5;
            break;

        default:
            rf_cal_sec = 0;
            break;
    }

    return rf_cal_sec;
}

void ICACHE_FLASH_ATTR user_rf_pre_init(void)
{
}


void ICACHE_FLASH_ATTR intr_callback(unsigned pin, unsigned level)
{
	int i;
	for( i=0;i<100;i++)
	{
		os_delay_us(1000);
		if(gpio_read(pin)!=0)
		{
			return ;
		}
	}
	if(level==0)
	{
		INFO("INTERRUPT: GPIO16 %d\r\n", level);
		ButtonCtrl(pin);
	}
}

void ICACHE_FLASH_ATTR user_init(void)
{
	uart_init(BIT_RATE_115200, BIT_RATE_115200);
	os_delay_us(1000000);

	CFG_Load();
	/*Make Established Link With Broker*/
	MQTT_InitConnection(&mqttClient, sysCfg.mqtt_host, sysCfg.mqtt_port, sysCfg.security);

	MQTT_InitClient(&mqttClient, sysCfg.device_id, sysCfg.mqtt_user, sysCfg.mqtt_pass, sysCfg.mqtt_keepalive, 1);
	/*Initialize Profile*/
	mqttClient.ToggleLed = ToggleLed;
	mqttClient.pstrtUserIOProfiles=(void*)strtuserioprofiles;

	MQTT_InitLWT(&mqttClient, "/lwt", "offline", 0, 0);
	MQTT_OnConnected(&mqttClient, mqttConnectedCb);
	MQTT_OnDisconnected(&mqttClient, mqttDisconnectedCb);
	MQTT_OnPublished(&mqttClient, mqttPublishedCb);
	MQTT_OnData(&mqttClient, mqttDataCb);

	WIFI_Connect(sysCfg.sta_ssid, sysCfg.sta_pwd, wifiConnectCb);

//	INFO("\r\nConfigure LED GPIO ...\r\n");

	// Configure pin as a GPIO

//	set_gpio_mode(GPIO_LED_PIN_1, GPIO_PULLUP, GPIO_OUTPUT);
//	set_gpio_mode(GPIO_LED_PIN_2, GPIO_PULLUP, GPIO_OUTPUT);
//	set_gpio_mode(GPIO_LED_PIN_3, GPIO_PULLUP, GPIO_OUTPUT);
//	//gpio_pin = 3;
//	//gpio_type = GPIO_PIN_INTR_POSEDGE;
//	INFO("\r\nConfigure Button ...\r\n");
//	if (set_gpio_mode(GPIO_BUTTON1, GPIO_PULLUP, GPIO_INT))
//	{
//		INFO("Config Interrupt");
//		gpio_intr_init(GPIO_BUTTON1, GPIO_PIN_INTR_NEGEDGE);
//		gpio_intr_attach(intr_callback);
//	} else {
//		INFO("Error: GPIO not set interrupt mode\r\n");
//	}


	INFO("\r\nSystem started ...\r\n");
}
