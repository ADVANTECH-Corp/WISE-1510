/**
* @file node_api.h
* @brief Node SDK API 
* 
*
* @author AdavnWISE
*/


#ifndef _NODE_API_H_
#define _NODE_API_H_

#ifdef __cplusplus
extern "C"
{
#endif


#define NODE_API_OK	         	0x0	///< Node API Result: OK
#define NODE_API_INVALID_ARG 	0x1	///< Node API Result: Invalid Argument
#define NODE_API_CFG_DEC_ERROR 		0x2	///< Node API Result: Config Decode Error
#define NODE_API_CFG_LEN_ERROR 		0x3	///< Node API Result: Config Length Error
#define NODE_API_CFG_NULL 		0x4		///< Node API Result: Config NULL
#define NODE_API_CFG_ENC_ERROR 		0x5	///< Node API Result: Config Encode Error
#define NODE_API_MEM_ERROR 			0x6	///< Node API Result: Memory Error
#define NODE_API_NOK 	0x7	///< Node API Result: Not OK


typedef enum 
{
	NODE_TXDONE_RC_TXOK, ///< TX OK
	NODE_TXDONE_RC_TXNOK,///< TX not OK, due to some reason
	NODE_TXDONE_RC_TXOK_ACK,///< TX OK and acked
	NODE_TXDONE_RC_TXOK_NACK,///< TX OK, timeout and  nacked
	NODE_TXDONE_RC_LAST
}NODE_TXDONE_RC;


typedef enum 
{
	NODE_RXDONE_RC_NORMAL,///< Normal RX
	NODE_RXDONE_RC_TXOK_ACK, ///< RX with TX confirmed data acked
	NODE_RXDONE_RC_LAST
}NODE_RXDONE_RC;


typedef enum 
{
	NODE_BCN_STATE_LOTTERY1,  ///<Lottery frame, node SPS enabled
	NODE_BCN_STATE_SPS,    ///<SPS frame
	NODE_BCN_STATE_LOTTERY2, ///<Lottery frame, node SPS is not enabled
	NODE_BCN_STATE_SCAN_TIMEOUT,///<Beacon scan timeout
	NODE_BCN_STATE_BCN_FOUND,///<Beacon found
	NODE_BCN_STATE_BCN_LOST,///<Beacon lost
	NODE_BCN_STATE_LAST
}NODE_BCN_STATE;

struct node_api_ev_rx_done
{
	unsigned char data_port;///< Node LoRa rx done data port
	unsigned char data_len;///< Node LoRa rx done data len
	unsigned char data[256];///< Node LoRa rx done data 
	short data_rssi;///< Node LoRa rx done data rssi 
	signed char data_snr;///< Node LoRa rx done data snr
};

typedef int(*EventTxDoneFP)(unsigned char);///< Node LoRa event callback tx done function pointer
typedef int(*EventRxDoneFP)(struct node_api_ev_rx_done*,unsigned char);///< Node LoRa event callback rx done function pointer
typedef int(*EventBeaconFP)(unsigned char,short,signed char);///< Node LoRa event callback beacon function pointer


/** Init Carrier Board
 *
 *  Do this if node module connected with carrier board
 *  @returns NODE_API_OK on success
 */
unsigned short nodeApiInitCarrierBoard();


/** Init
 *
 *  Must do this to Init
 *  @param log_serial for log output, input NULL if don't need
 *  @param sapi_serial for sapi, input NULL if don't need
 *  @returns NODE_API_OK on success
 */
unsigned short nodeApiInit(RawSerial *log_serial, RawSerial *sapi_serial);


/** Start Lora
 *
 *  Must do this to Start LoRa
 *  @returns NODE_API_OK on success
 */
unsigned short nodeApiStartLora();


/** Send sensor data over LoRa
 *
 *  @param port port number
 *  @param data sensor data buffer
 *  @param data_len length of sensor data
 *  @returns NODE_API_OK on success; others on failed
 */
unsigned short nodeApiSendData(unsigned char port, char *data, unsigned short data_len);


/** Get LoRa Join State
 *
 *  @returns 0 if not joined; >0 if joined
 */
int nodeApiJoinState();


/** Get Node current running device class
 *
 *  @returns device class
 */
unsigned char nodeApiDeviceClass();


/** Get App EUI
 *
 *  Get Application EUI for OTAA mode; length is 16, only consist of "0~9" or "A~F"
 *  @param buf_out App EUI if return NODE_API_OK
 *  @param buf_len length of buf_out 
 *  @returns NODE_API_OK on success; others on failed
 */
unsigned short nodeApiGetAppEui(char * buf_out, unsigned short buf_len);


/** Get App Key
 *
 *  Get Application Key for OTAA mode; length is 32, only consist of "0~9" or "A~F"
 *  @param buf_out App Key if return NODE_API_OK
 *  @param buf_len length of buf_out 
 *  @returns NODE_API_OK on success; others on failed
 */
unsigned short nodeApiGetAppKey(char * buf_out, unsigned short buf_len);


/** Get Dev Addr
 *
 *  Get Device Addr for ABP mode; length is 8, only consist of "0~9" or "A~F"
 *  @param buf_out Dev Addr if return NODE_API_OK
 *  @param buf_len length of buf_out 
 *  @returns NODE_API_OK on success; others on failed
 */
unsigned short nodeApiGetDevAddr(char * buf_out, unsigned short buf_len);


/** Get Network Session Key
 *
 *  Get Network session key for ABP mode; length is 32, only consist of "0~9" or "A~F"
 *  @param buf_out Network Session Key if return NODE_API_OK
 *  @param buf_len length of buf_out 
 *  @returns NODE_API_OK on success; others on failed
 */
unsigned short nodeApiGetNwkSKey(char * buf_out, unsigned short buf_len);


/** Get App Session Key
 *
 *  Get Application session key for ABP mode; length is 32, only consist of "0~9" or "A~F"
 *  @param buf_out App Session Key if return NODE_API_OK
 *  @param buf_len length of buf_out 
 *  @returns NODE_API_OK on success; others on failed
 */
unsigned short nodeApiGetAppSKey(char * buf_out, unsigned short buf_len);


/** Get Dev Act Mode
 *
 *  Get Activation mode(1:OTAA| 2:ABP; Default:2)
 *  @param buf_out Dev Act Mode if return NODE_API_OK
 *  @param buf_len length of buf_out 
 *  @returns NODE_API_OK on success; others on failed
 */
unsigned short nodeApiGetDevActMode(char * buf_out, unsigned short buf_len);


/** Get Dev Op Mode
 *
 *  Get Device Operating Mode (1:Advanwise| 2:LoRaWan | 3:MACless; Default:1)
 *  @param buf_out Dev Op Mode if return NODE_API_OK
 *  @param buf_len length of buf_out 
 *  @returns NODE_API_OK on success; others on failed
 */
unsigned short nodeApiGetDevOpMode(char * buf_out, unsigned short buf_len);


/** Get Dev Class
 *
 *  Get Device Class (1:classA| 2:classB| 3:classC; Default:1)
 *  @param buf_out Dev Class if return NODE_API_OK
 *  @param buf_len length of buf_out 
 *  @returns NODE_API_OK on success; others on failed
 */
unsigned short nodeApiGetDevClass(char * buf_out, unsigned short buf_len);


/** Get Advanwise mode Frequency
 *
 *  Get Advanwise mode frequency
 *  @param buf_out Frequency if return NODE_API_OK
 *  @param buf_len length of buf_out 
 *  @returns NODE_API_OK on success; others on failed
 */
unsigned short nodeApiGetDevAdvwiseFreq(char * buf_out, unsigned short buf_len);


/** Get Advanwise mode Data Rate
 *
 *  Get Advanwise mode Date Rate
 *  @param buf_out Data Rate if return NODE_API_OK
 *  @param buf_len length of buf_out 
 *  @returns NODE_API_OK on success; others on failed
 */
unsigned short nodeApiGetDevAdvwiseDataRate(char * buf_out, unsigned short buf_len);

/** Get Net Id
 *
 *  Get Net Id
 *  @param buf_out Net Id if return NODE_API_OK
 *  @param buf_len length of buf_out 
 *  @returns NODE_API_OK on success; others on failed
 */
unsigned short nodeApiGetDevNetId(char * buf_out, unsigned short buf_len);


/** Get Advanwise mode Tx power
 *
 *  Get Advanwise mode Transmit power
 *  @param buf_out Tx Power if return NODE_API_OK
 *  @param buf_len length of buf_out 
 *  @returns NODE_API_OK on success; others on failed
 */
unsigned short nodeApiGetDevAdvwiseTxPwr(char * buf_out, unsigned short buf_len);

/** Get SpsConf
 *
 *  Get SPS confirm (0:Disable, 1:Enable; Default:0)
 *  @param buf_out SpsConf if return NODE_API_OK
 *  @param buf_len length of buf_out 
 *  @returns NODE_API_OK on success; others on failed
 */
unsigned short nodeApiGetSpsConf(char * buf_out, unsigned short buf_len);

/** Get braodcast Key
 *
 *  Get braodcast Key; length is 32, only consist of "0~9" or "A~F"
 *  @param buf_out broadcast Key if return NODE_API_OK
 *  @param buf_len length of buf_out 
 *  @returns NODE_API_OK on success; others on failed
 */
unsigned short nodeApiGetBKey(char * buf_out, unsigned short buf_len);


/** Get Version
 *
 *  Get Version
 *  @param buf_out Version if return NODE_API_OK
 *  @param buf_len length of buf_out 
 *  @returns NODE_API_OK on success; others on failed
 */
unsigned short nodeApiGetVersion(char * buf_out, unsigned short buf_len);

/** Set App EUI
 *
 *  Set Application EUI for OTAA mode; length is 16, only consist of "0~9" or "A~F"
 *  @param buf_in App EUI to set
 *  @returns NODE_API_OK on success; others on failed
 */
unsigned short nodeApiSetAppEui(char * buf_in);


/** Set App Key
 *
 *  Set Application Key for OTAA mode; length is 32, only consist of "0~9" or "A~F"
 *  @param buf_in App Key to set
 *  @returns NODE_API_OK on success; others on failed
 */
unsigned short nodeApiSetAppKey(char * buf_in);


/** Set Dev Addr
 *
 *  Set Device Addr for ABP mode; length is 8, only consist of "0~9" or "A~F"
 *  @param buf_in Dev Addr to set
 *  @returns NODE_API_OK on success; others on failed
 */
unsigned short nodeApiSetDevAddr(char * buf_in);


/** Set Network Session Key
 *
 *  Set Network session key for ABP mode; length is 32, only consist of "0~9" or "A~F"
 *  @param buf_in Network Session Key to set
 *  @returns NODE_API_OK on success; others on failed
 */
unsigned short nodeApiSetNwkSKey(char * buf_in);


/** Set App Session Key
 *
 *  Set Application session key for ABP mode; length is 32, only consist of "0~9" or "A~F"
 *  @param buf_in App Session Key to set
 *  @returns NODE_API_OK on success; others on failed
 */
unsigned short nodeApiSetAppSKey(char * buf_in);


/** Set Dev Act Mode
 *
 *  Set Activation mode(1:OTAA| 2:ABP)
 *  @param buf_in Dev Act Mode to set
 *  @returns NODE_API_OK on success; others on failed
 */
unsigned short nodeApiSetDevActMode(char * buf_in);


/** Set Dev Op Mode
 *
 *  Set Device Operating Mode (1:Advanwise| 2:LoRaWan | 3:MACless)
 *  @param buf_in Dev Op Mode to set
 *  @returns NODE_API_OK on success; others on failed
 */
unsigned short nodeApiSetDevOpMode(char * buf_in);


/** Set Dev Class
 *
 *  Set Device Class (1:classA| 2:classB| 3:classC)
 *  @param buf_in Dev Class to set
 *  @returns NODE_API_OK on success; others on failed
 */
unsigned short nodeApiSetDevClass(char * buf_in);


/** Set Advanwise mode Frequency
 *
 *  Set Advanwise mode frequency
 *  @param buf_in Frequency to set
 *  @returns NODE_API_OK on success; others on failed
 */
unsigned short nodeApiSetDevAdvwiseFreq(char * buf_in);


/** Set Advanwise mode Data Rate
 *
 *  Set Advanwise mode Date Rate
 *  @param buf_in Data Rate to set
 *  @returns NODE_API_OK on success; others on failed
 */
unsigned short nodeApiSetDevAdvwiseDataRate(char * buf_in);


/** Set Advanwise mode Tx Power
 *
 *  Set Advanwise mode Transmit power
 *  @param buf_in Tx Power to set
 *  @returns NODE_API_OK on success; others on failed
 */
unsigned short nodeApiSetDevAdvwiseTxPwr(char * buf_in);

/** Set SpsConf
 *
 *  Set SPS confirm (0: Disable |1:Enable; Default:0)
 *  @param buf_in SpsConf to set
 *  @returns NODE_API_OK on success; others on failed
 */
unsigned short nodeApiSetSpsConf(char * buf_in);

/** Set Dev Netid
 *
 *  Set Device net id for WISE-link 2.0; length is 2 bytes, only consist of "0~9" or "A~F"
 *  @param buf_in Dev net id to set
 *  @returns NODE_API_OK on success; others on failed
 */
unsigned short nodeApiSetDevNetId(char * buf_in);

/** Set broadcast Key
 *
 *  Set broadcast Key; length is 32, only consist of "0~9" or "A~F"
 *  @param buf_in braodcast Key to set
 *  @returns NODE_API_OK on success; others on failed
 */
unsigned short nodeApiSetBKey(char * buf_in);

/** Set Deep Sleep mode Wakeup timer
 *
 *  Set Into Deep Sleep mode Wakeup timer with RTC
 *  @param sec wakeup time in second
 *  @returns NODE_API_OK on success; others on failed
 */
unsigned short nodeApiSetDevSleepRTCWakeup(int sec);

/** Factory Reset
 *
 *  @returns NODE_API_OK on success; others on failed
 */
unsigned short nodeApiFactoryReset();

/** Save Config
 *
 *  Save Config to flash
 *  @returns NODE_API_OK on success; others on failed
 */
unsigned short nodeApiSaveCfg();


/** Reboot node
 *
 *  @returns NODE_API_OK on success; others on failed
 */
unsigned short nodeApiReboot();


/** Load Config
 *
 *  Load Config from flash
 *  @returns NODE_API_OK on success; others on failed
 */
unsigned short nodeApiLoadCfg();


/** Apply Config
 *
 *  @returns NODE_API_OK on success; others on failed
 */
unsigned short nodeApiApplyCfg();


/** Get Fuse Dev EUI
 *
 *  Get Fuse Device EUI; length is 16, only consist of "0~9" or "A~F"
 *  @param buf_out Fuse Dev EUI if return 0
 *  @param buf_len length of buf_out 
 *  @returns NODE_API_OK on success; others on failed
 */
unsigned short nodeApiGetFuseDevEui(char * buf_out, unsigned short buf_len);


/** Set Tx done event call back
 *
 *  Set event callback function get TxDone event
 *  @param txdone_cb event callback function
 *  @returns NODE_API_OK on success; others on failed
 */
unsigned short nodeApiSetTxDoneCb(EventTxDoneFP txdone_cb);

/** Set Rx done event call back
 *
 *  Set event callback function get RxDone event
 *  @param rxdone_cb event callback function
 *  @returns NODE_API_OK on success; others on failed
 */
unsigned short nodeApiSetRxDoneCb(EventRxDoneFP rxdone_cb);
/** Set Beacon event call back
 *
 *  Set event callback function get Beacon event
 *  @param beacon_cb event callback function
 *  @returns NODE_API_OK on success; others on failed
 */
unsigned short nodeApiSetBeaconCb(EventBeaconFP beacon_cb);

/** Send high priority sensor data which is SPS data over LoRa
 *
 *  @param port port number
 *  @param data sensor data buffer
 *  @param data_len length of sensor data
 *  @returns NODE_API_OK on success; others on failed
 */
unsigned short nodeApiSendDataHighPri(unsigned char port, char *data, unsigned short data_len);

/** Send high priority sensor data which is SPS and confirmed data over LoRa
 *
 *  @param port port number
 *  @param data sensor data buffer
 *  @param data_len length of sensor data
 *  @returns NODE_API_OK on success; others on failed
 */
 unsigned short nodeApiSendDataHighPriConfirm(unsigned char port, char *data, unsigned short data_len);


/** Node SPS status 
 *
 *  @returns 1 on enabled; 0 on disabled 
 */
unsigned char nodeApiDeviceSpsEnabled();

/** Restart Lora
 *
 *  Restart LoRa
 *  @param default_delay Using default delay period while set to 1 and custom_delay_period_ms is ignored(recommended)
 *  @param custom_delay_period_ms Only applied while default_delay is set to 0
 *  @returns NODE_API_OK on success
 */
unsigned short nodeApiRestartLora(unsigned char default_delay,unsigned int custom_delay_period_ms);


/** Send sensor data over LoRa
 *
 *  @param port port number
 *  @param data sensor data buffer
 *  @param data_len length of sensor data
 *  @returns NODE_API_OK on success; others on failed
 */
unsigned short nodeApiSendDataConfirm(unsigned char port, char *data, unsigned short data_len);


/** Stop Lora
 *
 *  Stop LoRa
 *  @returns NODE_API_OK on success
 */
unsigned short nodeApiStopLora();


/** Enable ExternalRTC
 *
 *  Enable external RTC enable for WISE-1510 WISE-link 2.0, must be execute before nodeApiStartLora(0: Disable |1:Enable; Default:0)
 *  @param buf_in ExternalRTC to Enable
  *  @param buf_in ExternalRTC i2c interface
 *  @returns NODE_API_OK on success
 */
unsigned short nodeApiEnableExternalRTC(unsigned char enable,void *i2c);

#ifdef __cplusplus
}
#endif

#endif
