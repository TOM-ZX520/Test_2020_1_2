#ifndef _AiThinker_A6_
#define _AiThinker_A6_

#include "syslib.h"

#define AT_CMD_AT_NO    1
#define AT_CMD_AT             "AT\r\n"
#define AT_CMD_ATE0_NO  2
#define AT_CMD_ATE0           "ATE0\r\n"
#define AT_CMD_CREG_NO  3
#define AT_CMD_CREG           "AT+CREG?\r\n"//check net reg info
#define AT_CMD_CSQ_NO   4
#define AT_CMD_CSQ            "AT+CSQ\r\n"//signal quality
#define AT_CMD_CCID_NO  5
#define AT_CMD_CCID           "AT+CCID\r\n"//sim ID code,check sim card is in the slot
#define AT_CMD_CGATT_NO 6
#define AT_CMD_CGATT          "AT+CGATT=1\r\n"//attach network
#define AT_CMD_CGACT_NO 7
#define AT_CMD_CGACT          "AT+CGACT=1,1\r\n"//activate network
#define AT_CMD_CIPSTART_NO 8
#define AT_CMD_CIPSTART       "\"TCP\",\"121.41.54.232\",4696\r\n"//Sever IP
#define AT_CMD_CIPSEND_NO 9
#define AT_CMD_CIPSEND        "AT+CIPSEND="//send GPRS message
#define AT_CMD_CIPCLOSE_NO 10
#define AT_CMD_CIPCLOSE       "AT+CIPCLOSE\r\n"//close tcpip connect
#define AT_CMD_IMEI_NO  11
#define AT_CMD_IMEI           "AT+CGSN\r\n"//read IMEI code
#define AT_CMD_CIPDATA_NO  12
#define AT_CMD_DATA           " "//blank
#define AT_CMD_CGDCONT_NO  13
#define AT_CMD_CGDCONT        "AT+CGDCONT=1,\"IP\",\"CMNET\"\r\n"//set PDP para

#define AT_CMD_COMMON_ERROR   "+CME ERROR:"

#define RCC_GPIO_GSM    RCC_APB2Periph_GPIOC
#define GPIO_GSM        GPIOC
#define GSM_PWR         GPIO_Pin_4
#define GSM_LPM         GPIO_Pin_6
#define GSM_RST         GPIO_Pin_7

//enable on 1
#define GSM_PWR_ON      GPIO_SetBits(GPIO_GSM, GSM_PWR)
#define GSM_PWR_OFF     GPIO_ResetBits(GPIO_GSM, GSM_PWR)
//enable on 0
#define GSM_LPM_ON      GPIO_ResetBits(GPIO_GSM, GSM_LPM)
#define GSM_LPM_OFF     GPIO_SetBits(GPIO_GSM, GSM_LPM)
//enable on 1
#define GSM_RST_ON      GPIO_SetBits(GPIO_GSM, GSM_RST)
#define GSM_RST_OFF     GPIO_ResetBits(GPIO_GSM, GSM_RST)

#define GPRS_CONSTATE_UART 0
#define GPRS_CONSTATE_ATE0 1
#define GPRS_CONSTATE_CCID 2
#define GPRS_CONSTATE_CREG 3
#define GPRS_CONSTATE_CGATT 4
#define GPRS_CONSTATE_CGDCONT 5
#define GPRS_CONSTATE_CGACT 6
#define GPRS_CONSTATE_IPSTART 7
#define GPRS_CONSTATE_STATUS 8
#define GPRS_CONSTATE_CLOSE 9
#define GPRS_CONSTATE_SUCC 10

#define GPRS_SBUF_EMPTY  0
#define GPRS_RBUF_EMPTY  0
#define GPRS_SEND_WAIT  12
#define GPRS_SEND_READY  13
#define GPRS_RECEIVE_READY 14
#define GPRS_RECEIVE_ING 15

#define CMD_TABLE_LENGTH  13

typedef enum
{    
  AT_CMD_SUCCESS=1,  
  AT_CMD_FAILED=100,
  AT_CMD_TIMEOUE=101,
  AT_CMD_UNKNOWN=255
}AT_CMD_RESULT;

typedef struct
{
  unsigned char cmdNo;
  char cmd[30];
  char success[15];
  char failed[15]; 
  unsigned char result;
}AT_CMD_PARAM;

extern void GPIO_GSM_Configuration(void);
extern void GPIO_GSM_Init(void);
extern void A6_MODULE_ON(void);
extern void A6_MODULE_OFF(void);
extern void Reset_GSM_MODULE(void);

extern void SendAtCmd(char * cmd, unsigned char atCmdNo);
extern void SetAtSendLength(uint16 udatLength);
extern void SendAtData(uint8 *str, uint16 len);
extern void ReceiveAtData(uint8 rbufIdx);
extern void ReConnectGPRS(void);
extern uint8 DataLayerGSM(void);
extern void WaitConnectGPRS(void);
extern void DataLayerTCPS(void);

#endif
