/**
 * @file main.cpp
 *
 * @brief Lora Node SDK Sample
 *
 * example show how to set/save config and read/send sensor data via LoRa periodically;
 *
 * @author AdvanWISE
*/


#include "mbed.h"
#include "node_api.h"

#define WISE_VERSION	            "1510-MM-X0102-VJ-PowerConsumption"
#define NODE_AUTOGEN_APPKEY

#define NODE_SENSOR_TEMP_HUM_ENABLE 1  ///< Enable or disable TEMP/HUM sensor report, default disable
#define NODE_SENSOR_CO2_VOC_ENABLE 0  ///< Enable or disable CO2/VOC sensor report, default disable
#define NODE_GPIO_ENABLE 0  ///< Enable or disable GPIO report, default enable

#define NODE_DEBUG(x,args...) node_printf_to_serial(x,##args)

#define NODE_DEEP_SLEEP_MODE_SUPPORT 1///< Flag to Enable/Disable deep sleep mode
#define NODE_ACTIVE_PERIOD_IN_SEC 10    ///< Period time to read/send sensor data  >= 3sec
#define NODE_RXWINDOW_PERIOD_IN_SEC 4    ///< Rx windown time  
#define NODE_ACTIVE_TX_PORT 1          ///< Lora Port to send data

RawSerial debug_serial(PA_9, PA_10);	///< Debug serial port

DigitalIn gp2(PC_8);
DigitalIn gp3(PC_7);
DigitalIn gp4(PC_5);
DigitalIn gp5(PB_0);
DigitalIn gp6(PA_2);
DigitalIn gp7(PA_3);
DigitalIn gp8(PB_6);				
#if NODE_GPIO_ENABLE
///< Control downlink GPIO0
static DigitalOut led(PA_8);
static unsigned int gpio0;
#else
DigitalIn gp1(PA_8);	 
#endif

typedef enum
{
	NODE_STATE_INIT,            ///< Node init state
	NODE_STATE_LOWPOWER = 1,    ///< Node low power state
	NODE_STATE_ACTIVE,          ///< Node active state
	NODE_STATE_TX,              ///< Node tx state
	NODE_STATE_RX,	      ///<Node rx state
	NODE_STATE_RX_DONE,         ///< Node rx done state
}node_state_t;

struct node_api_ev_rx_done node_rx_done_data;
volatile node_state_t node_state = NODE_STATE_INIT;
static char node_class=1;
static char node_op_mode=1;
static char node_act_mode=1;
static char node_beacon_state=NODE_BCN_STATE_LOTTERY1;

#if NODE_SENSOR_TEMP_HUM_ENABLE
static unsigned int  node_sensor_temp_hum=0; ///<Temperature and humidity sensor global
#endif
#if NODE_SENSOR_CO2_VOC_ENABLE
static  unsigned int node_sensor_voc_co2=0; ///<Voc and CO2 sensor global
#endif

I2C i2c(PC_1, PC_0); ///<i2C define

/** @brief print message via serial
 *
 *  @param format message to print
 *  @returns 0 on success
 */
int node_printf_to_serial(const char * format, ...)
{
	unsigned int i;
	va_list ap;

	char buf[512 + 1];
	memset(buf, 0, 512+1);

	va_start(ap, format);
	vsnprintf(buf, sizeof(buf), (char *)format, ap);  
	va_end(ap);
	
	for(i=0; i < strlen(buf); i++)
	{
		debug_serial.putc(buf[i]);
	}
	
	return 0;
}

#if NODE_SENSOR_CO2_VOC_ENABLE
/** @brief TVOC and CO2 sensor read
 *
 */
unsigned int iaq_core_sensor(void)
{
	static unsigned int vc = 0;
	char data_read[9];
	i2c.read(0xB5, data_read, 9, 1);
	if(data_read[2]==0x0||data_read[2]==0x10)
	{	
		//NODE_DEBUG(" IAQ status: %x \n\r",  data_read[2]);
		//NODE_DEBUG(" CO2:  %d ppm \n\r",   (data_read[0] << 8 | data_read[1]));
		//NODE_DEBUG(" TVOC: %d ppb \n\r",   (data_read[7] << 8 | data_read[8]));

		vc = (data_read[0]<<24|data_read[1]<<16|data_read[7]<<8|data_read[8]);
	}
	return vc;
}

/** @brief TVOC and CO2 sensor thread
 *
 */
static void node_sensor_voc_co2_thread(void const *args) 
{
	char data_write[1];
	data_write[0]=0x06;//0x2;
	i2c.write(0xe0, data_write, 1, 0); // i2c expander enable channel_1 and ch2,no stop
	 
	while(1) 
	{      
		node_sensor_voc_co2=( unsigned int)iaq_core_sensor();
		Thread::wait(2000);//need more than 2 sec to read 
	}
}
#endif

#if NODE_SENSOR_TEMP_HUM_ENABLE
/** @brief Temperature and humidity sensor read
 *
 */
static unsigned int hdc1510_sensor(void) 
{
	char data_write[3];
	char data_read[4];

	#define HDC1510_REG_TEMP  0x0
	#define HDC1510_ADDR 0x80

	data_write[0]=HDC1510_REG_TEMP;
	i2c.write(HDC1510_ADDR, data_write, 1, 1); 
	Thread::wait(50);
	i2c.read(HDC1510_ADDR, data_read, 4, 0);
	float tempval = (float)((data_read[0] << 8 | data_read[1]) * 165.0 / 65536.0 - 40.0);

	/*Temperature*/
	int ss = tempval*100;
	unsigned int yy=0;
	//printf("Temperature: %.2f C\r\n",tempval );
	/*Humidity*/
	float hempval = (float)((data_read[2] << 8 | data_read[3]) * 100.0 / 65536.0);   
	yy=hempval*100;
	// printf("Humidity: %.2f %\r\n",hempval);

	return (yy<<16)|ss;	
}

/** @brief Temperature and humidity sensor thread
 *
 */
static void node_sensor_temp_hum_thread(void const *args)
{
	while(1) 
	{
		Thread::wait(1000);
		node_sensor_temp_hum=(unsigned int )hdc1510_sensor();
	}
}
#endif

/** @brief node tx procedure done
 *
 */
int node_tx_done_cb(unsigned char rc)
{

	node_state=NODE_STATE_LOWPOWER;
	return 0;
}

/** @brief node got rx data
 *
 */
int node_rx_done_cb(struct node_api_ev_rx_done *rx_done_data, unsigned char rc)
{
	memset(&node_rx_done_data,0,sizeof(struct node_api_ev_rx_done));
	memcpy(&node_rx_done_data,rx_done_data,sizeof(struct node_api_ev_rx_done));
	node_state=NODE_STATE_RX_DONE;
	return 0;
}

/** @brief node got beacon
 *
 */
int node_beacon_cb(unsigned char state, short rssi, signed char snr)
{
	node_beacon_state=NODE_BCN_STATE_LOTTERY1;

	switch(state)
	{
		case NODE_BCN_STATE_LOTTERY1:
		//NODE_DEBUG("Beacon CB: LOT\r\n",nodeApiDeviceSpsEnabled());	
		if(!nodeApiDeviceSpsEnabled())
		{
			node_state=NODE_STATE_ACTIVE;
		}
			break;
		case NODE_BCN_STATE_SPS:
		//NODE_DEBUG("Beacon CB: SPS\r\n");	
		
			node_state=NODE_STATE_ACTIVE;
			break;
		case NODE_BCN_STATE_LOTTERY2:
			//NODE_DEBUG("Beacon CB: LOT(SPS not supported)\r\n");	
		if(!nodeApiDeviceSpsEnabled())
		{
			node_state=NODE_STATE_ACTIVE;
		}
			break;
	}
	
	node_beacon_state=state;

	return 0;
}

/** @brief An example to show version
 *  
 */
void node_show_version()
{
	char buf_out[256];
	unsigned short ret=NODE_API_OK;

	memset(buf_out, 0, 256);
	ret=nodeApiGetVersion(buf_out, 256);
	if(ret==NODE_API_OK)
	{
		NODE_DEBUG("Version=%s\r\n", buf_out);
	}	
}




/** @brief An example to set node config
 *  
 */
void node_set_config()
{
	char deveui[32]={};
	char devaddr[16]={};
	char appkey[33]={};	
	
	if(nodeApiGetFuseDevEui(deveui,16)!=NODE_API_OK)
	{
		NODE_DEBUG("Get fuse DevEui failed\r\n");
		return;
	}	
		
	strcpy(devaddr,&deveui[8]);
	nodeApiSetDevAddr(devaddr);
	//nodeApiSetSpsConf("1");
	//sprintf(appkey,"%s%s",deveui,deveui);		
	//nodeApiSetAppKey(appkey);	
	
	/*User configuration*/
	//nodeApiSetAppEui("00000000000000ab");
	//nodeApiSetAppKey("00000000000000000000000000000011");
	//nodeApiSetNwkSKey("00000000000000000000000000000011");
	//nodeApiSetAppSKey("00000000000000000000000000000011");
	nodeApiSetDevActMode("2");
	nodeApiSetDevOpMode("1");
	nodeApiSetDevClass("1");
	//nodeApiSetDevAdvwiseDataRate("4");
	//nodeApiSetDevAdvwiseFreq("923300000");
	//nodeApiSetDevAdvwiseTxPwr("20");
}





/** @brief An example to get node config
 *  
 */
void node_get_config()
{
	char buf_out[256];
	unsigned short ret=NODE_API_OK;

	memset(buf_out, 0, 256);
	ret=nodeApiGetFuseDevEui(buf_out, 256);
	if(ret==NODE_API_OK)
	{
		NODE_DEBUG("DevEui=%s\r\n", buf_out);
	}

	memset(buf_out, 0, 256);
	ret=nodeApiGetAppEui(buf_out, 256);
	if(ret==NODE_API_OK)
	{
		NODE_DEBUG("AppEui=%s\r\n", buf_out);
	}

	memset(buf_out, 0, 256);
	ret=nodeApiGetAppKey(buf_out, 256);
	if(ret==NODE_API_OK)
	{
		NODE_DEBUG("AppKey=%s\r\n", buf_out);
	}

	memset(buf_out, 0, 256);
	ret=nodeApiGetDevAddr(buf_out, 256);
	if(ret==NODE_API_OK)
	{
		NODE_DEBUG("DevAddr=%s\r\n", buf_out);
	}

	memset(buf_out, 0, 256);
	ret=nodeApiGetNwkSKey(buf_out, 256);
	if(ret==NODE_API_OK)
	{
		NODE_DEBUG("NwkSKey=%s\r\n", buf_out);
	}

	memset(buf_out, 0, 256);
	ret=nodeApiGetAppSKey(buf_out, 256);
	if(ret==NODE_API_OK)
	{
		NODE_DEBUG( "AppSKey=%s\r\n", buf_out);
	}

	memset(buf_out, 0, 256);
	ret=nodeApiGetDevOpMode(buf_out, 256);
	if(ret==NODE_API_OK)
	{
		node_op_mode=atoi(buf_out);
		NODE_DEBUG("DevOpMode=%d\r\n", node_op_mode);
	}

	memset(buf_out, 0, 256);
	ret=nodeApiGetDevActMode(buf_out, 256);
	if(ret==NODE_API_OK)
	{
		node_act_mode=atoi(buf_out);
		NODE_DEBUG("DevActMode=%s\r\n", buf_out);
	}

	memset(buf_out, 0, 256);
	ret=nodeApiGetDevClass(buf_out, 256);
	if(ret==NODE_API_OK)
	{
		node_class=atoi(buf_out);
		NODE_DEBUG("DevClass=%d\r\n", node_class);
	}

	if(node_op_mode==1||node_op_mode==2)
	{
		memset(buf_out, 0, 256);
		ret=nodeApiGetDevAdvwiseDataRate(buf_out, 256);
		if(ret==NODE_API_OK)
		{
			NODE_DEBUG("DevAdvwiseDataRate=%s\r\n", buf_out);
		}
	}
	if(node_op_mode==1)
	{
		memset(buf_out, 0, 256);
		ret=nodeApiGetDevAdvwiseFreq(buf_out, 256);
		if(ret==NODE_API_OK)
		{
			NODE_DEBUG("DevAdvwiseFreq=%sHz\r\n", buf_out);
		}
	}
	if(node_op_mode==4)
	{
		memset(buf_out, 0, 256);
		ret=nodeApiGetDevNetId(buf_out, 256);
		if(ret==NODE_API_OK)
		{
			NODE_DEBUG("DevNetId=%s\r\n", buf_out);
		}
	}

	memset(buf_out, 0, 256);
	ret=nodeApiGetDevAdvwiseTxPwr(buf_out, 256);
	if(ret==NODE_API_OK)
	{
		NODE_DEBUG("DevAdvwiseTxPwr=%sdBm\r\n", buf_out);
	}	
}

/** @brief Read sensor data
 *
 *  A simple sample to generate sensor data, user should implement read sensor data
 *  @param data sensor_data
 *  @returns data_length
 */
unsigned char node_get_sensor_data (char *data)
{
	unsigned char len=0;
	unsigned char sensor_data[32];

	memset(sensor_data,0x0,sizeof(sensor_data));
	#if NODE_SENSOR_TEMP_HUM_ENABLE
	sensor_data[len+2]=0x1;
	len++; // temperature
	sensor_data[len+2]=0x3;
	len++;  // len:3 bytes	
	sensor_data[len+2]=0x0;
	len++; //0 is positive, 1 is negative
	sensor_data[len+2]=(node_sensor_temp_hum>>8)&0xff;
	len++; 
	sensor_data[len+2]=node_sensor_temp_hum&0xff;
	len++; 		
	sensor_data[len+2]=0x2;
	len++;  // humidity
	sensor_data[len+2]=0x2; 
	len++; // len:2 bytes
	sensor_data[len+2]=(node_sensor_temp_hum>>24)&0xff;
	len++; 
	sensor_data[len+2]=(node_sensor_temp_hum>>16)&0xff;
	len++;
	#endif
	#if NODE_SENSOR_CO2_VOC_ENABLE
	sensor_data[len+2]=0x3;
	len++; // CO2
	sensor_data[len+2]=0x2;
	len++;  // len:2 bytes	
	sensor_data[len+2]=(node_sensor_voc_co2>>24)&0xff;
	len++; 
	sensor_data[len+2]=(node_sensor_voc_co2>>16)&0xff;
	len++; 		
	sensor_data[len+2]=0x4;
	len++;  // TVOC
	sensor_data[len+2]=0x2;
	len++; // len:2 bytes
	sensor_data[len+2]=(node_sensor_voc_co2>>8)&0xff;
	len++; 
	sensor_data[len+2]=node_sensor_voc_co2&0xff;
	len++;
	#endif
	#if NODE_GPIO_ENABLE
	sensor_data[len+2]=0x5;
	len++;  // GPIO
	sensor_data[len+2]=0x1;
	len++; // len:1 bytes
	sensor_data[len+2]=gpio0;
	len++;
	#endif
	
	#if ((!NODE_SENSOR_TEMP_HUM_ENABLE)&&(!NODE_SENSOR_CO2_VOC_ENABLE)&&(!NODE_GPIO_ENABLE))
	return 0;
	#else
	//header
	sensor_data[0]=len;
	sensor_data[1]=0xc; //publish
	memcpy(data, sensor_data,len+2);
	return len+2;		
	#endif
}	


/** @brief An loop to read and send sensor data via LoRa periodically
 *  
 */
void node_state_loop()
{
	static unsigned char join_state=0;/*0:init state, 1:not joined, 2: joined*/

	nodeApiSetTxDoneCb(node_tx_done_cb);
	nodeApiSetRxDoneCb(node_rx_done_cb);

	node_state=NODE_STATE_LOWPOWER;

	if(node_op_mode==4)
	{
		NODE_DEBUG("WISE link 2.0 \r\n");	
		nodeApiSetBeaconCb(node_beacon_cb);
	}

	while(1)
	{
		if(nodeApiJoinState()==0)
		{	
			if(join_state==2)
				NODE_DEBUG("LoRa is not joined.\r\n");	

			Thread::wait(1000);
			
			join_state=1;
			continue;
		}
		else
		{
			if(join_state<=1)
			{
				node_class=nodeApiDeviceClass();
				NODE_DEBUG("LoRa Joined.\r\n");		
				node_state=NODE_STATE_LOWPOWER;

				if(node_act_mode==1&&(node_op_mode==4||node_op_mode==1))
				{
					time_t seconds = time(NULL);
	        
					NODE_DEBUG("Time as seconds since January 1, 1970 = %d\n", seconds);
					NODE_DEBUG("Time as a basic string = %s", ctime(&seconds));
					
				}
     
			}

			join_state=2;		
		}
	
		switch(node_state)
		{
			case NODE_STATE_LOWPOWER:
			{
				if(node_class==3||node_op_mode==4)
				{
					static unsigned int count=0;
				
					Thread::wait(10);
					
					if(node_state!=NODE_STATE_RX_DONE)
					{				
						if(count%(NODE_ACTIVE_PERIOD_IN_SEC*100)==0)
							if(node_op_mode!=4)
								node_state=NODE_STATE_ACTIVE;
					}
					count++;
				}
				else
				{
					Thread::wait(NODE_RXWINDOW_PERIOD_IN_SEC*1000);

					/*Receive RX while sleep*/
					if(node_state==NODE_STATE_RX_DONE)
						continue;
					else
					{
						#if NODE_DEEP_SLEEP_MODE_SUPPORT
							nodeApiSetDevSleepRTCWakeup(NODE_ACTIVE_PERIOD_IN_SEC-NODE_RXWINDOW_PERIOD_IN_SEC);
						#else
							Thread::wait((NODE_ACTIVE_PERIOD_IN_SEC-NODE_RXWINDOW_PERIOD_IN_SEC)*1000);
						#endif
						if(node_state==NODE_STATE_RX_DONE)
							continue;
						else
							node_state=NODE_STATE_ACTIVE;	
					}
				}
			}
				break;
			case NODE_STATE_ACTIVE:
			{
				int i=0,ret=0;
				unsigned char frame_len=0;
				char frame[64]={};
				
				frame_len=node_get_sensor_data(frame);
			
				if(frame_len==0)
				{
					node_state=NODE_STATE_LOWPOWER;
					break;
				}


				if(node_beacon_state==NODE_BCN_STATE_SPS)
					ret=nodeApiSendDataHighPri(NODE_ACTIVE_TX_PORT, frame, frame_len);
				else
					ret=nodeApiSendData(NODE_ACTIVE_TX_PORT, frame, frame_len);

				if(ret==0)
				{
					NODE_DEBUG("TX: ");

					for(i=0;i<frame_len;i++)
					{
						NODE_DEBUG("%02X ",frame[i]);
					}
					
					NODE_DEBUG("\n\r");
					
					node_state=NODE_STATE_TX;
				}
				else
				{
					NODE_DEBUG("TX: Forbidden!\n\r ");
					node_state=NODE_STATE_LOWPOWER;
				}
			}
				break;
			case NODE_STATE_TX:
				break;
			case NODE_STATE_RX:
				break;
			case NODE_STATE_RX_DONE:
			{
				if(node_rx_done_data.data_len!=0)
				{
					int i=0;
					int j=0;
					char print_buf[512];
					memset(print_buf, 0, sizeof(print_buf));

					NODE_DEBUG("RX: ");
					for(i=0;i<node_rx_done_data.data_len;i++)
					{
						NODE_DEBUG("%02X ", node_rx_done_data.data[i]);
					}

					NODE_DEBUG("\r\n(Length: %d, Port%d)\r\n", node_rx_done_data.data_len,node_rx_done_data.data_port);
					
					// 
					// Downlink data handling
                                   // Data port of downlink is the same as uplinlk Tag ID in TLV format
					//
					#if NODE_GPIO_ENABLE
					if(node_rx_done_data.data_port==5 && node_rx_done_data.data_len==1)
                                   {
                    				if (node_rx_done_data.data[0] == '1')
                       			{
                        				led=1;
			                            gpio0=1;
			                      }
			                      else
			                      {
			                      	led=0;
			                            gpio0=0;
			                       }
			               }
					#endif // NODE_GPIO_ENABLE
				}
				node_state=NODE_STATE_LOWPOWER;
				break;
			}
			default:
				break;
				
		}
	}
}




/** @brief Main function
 */
int main () 
{
	/*Create sensor thread*/
	#if NODE_SENSOR_TEMP_HUM_ENABLE
	Thread *p_node_sensor_temp_hum_thread;
	#endif
	#if NODE_SENSOR_CO2_VOC_ENABLE
	Thread *p_node_sensor_co2_thread;
	#endif
	
	/* Init carrier board, must be first step */
	nodeApiInitCarrierBoard();

	debug_serial.baud(115200);

	nodeApiInit(&debug_serial, &debug_serial);
	#if NODE_SENSOR_TEMP_HUM_ENABLE
	p_node_sensor_temp_hum_thread=new Thread(node_sensor_temp_hum_thread);
	#endif
	#if NODE_SENSOR_CO2_VOC_ENABLE
	p_node_sensor_co2_thread=new Thread(node_sensor_voc_co2_thread);
	#endif
	
	/* Display version information */
	NODE_DEBUG("\f");
    	NODE_DEBUG("\t\t *************************************************\n\r");
    	NODE_DEBUG("\t\t\t\t");
    	NODE_DEBUG(WISE_VERSION);
    	NODE_DEBUG("\n\r");
    	NODE_DEBUG("\t\t *************************************************\n\r");
	node_show_version();
	
	/*
	 * Init configuration at beginning
	 */
	nodeApiLoadCfg();

	node_set_config();

	#ifdef NODE_AUTOGEN_APPKEY	

	char deveui[17]={};
	if(nodeApiGetFuseDevEui(deveui,16)==NODE_API_OK)	
	{		
		char appkey[33]={};		
		sprintf(appkey,"%s%s",deveui,deveui);		
		nodeApiSetAppKey(appkey);	
	}	
	#endif	
	
	/* Apply to module */
	nodeApiApplyCfg();

	/* Start Lora */
	nodeApiStartLora();

	node_get_config();

	Thread::wait(1000);

	#if (!NODE_SENSOR_TEMP_HUM_ENABLE)
	while(1)
	{
		Thread::wait(1000);
	}	
	#else
	/*
	 *  Node state loop
	 */
	node_state_loop();
	#endif

	/*Never reach here*/	
	return 0;
}
	




