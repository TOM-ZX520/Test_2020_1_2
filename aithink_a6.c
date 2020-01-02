/*******************************************************************************
* File Name      : aithink_a6.c
* Description    : STM32实现GPRS与服务器数据传输(安信可A6模块)
* Author         : River
* Date / Version : 20161213 / V1.0
* CopyRight      : River
* Details        : http://blog.csdn.net/cc214042/article/details/53152329
*******************************************************************************/
#include "stm32f10x_it.h"
#include "aithink_a6.h"
#include "systick.h"
#include "uart.h"

#ifdef _HARD_VERSION_NET_SIM_

#define GPRS_RBUF_COUNT 16

uint8 g_gprsConState = GPRS_CONSTATE_UART;//GPRS Connection state 
uint8 g_sendAtCmdNo=0;//send AT command number
uint8 g_gprsSState = GPRS_SBUF_EMPTY;//GPRS send state
uint8 g_gprsRState = GPRS_RBUF_EMPTY;//GPRS receive state
uint8 g_gprs_Stimeout = 0;//send data timeout
uint8 g_gprs_comtimes = 0;//connect state timeout
uint8 g_gprsRBUF[MAX_GPRS_BUF_SIZE];
uint8 g_gprsRBufState[GPRS_RBUF_COUNT];//接收状态
uint16 g_gprsRBufAddr[GPRS_RBUF_COUNT];//接收起始地址

uint16 g_gprsCnt = 0;//RBUF 当前计数值
uint16 g_gprspCnt = 0;//RBUF 已处理的位置
uint16 g_gprsmCnt = 0;//
uint16 g_gprsLength = 0;
char g_simCCID[20];
char g_simIMEI[15];

AT_CMD_PARAM at_cmd_table[CMD_TABLE_LENGTH]={
{AT_CMD_AT_NO,        AT_CMD_AT,        "OK",         "+CME", AT_CMD_UNKNOWN},
{AT_CMD_ATE0_NO,      AT_CMD_ATE0,      "OK",         "+CME", AT_CMD_UNKNOWN},
{AT_CMD_CREG_NO,      AT_CMD_CREG,      "+CREG:",     "+CME", AT_CMD_UNKNOWN},
{AT_CMD_CSQ_NO,       AT_CMD_CSQ,       "+CSQ:",      "+CME", AT_CMD_UNKNOWN},
{AT_CMD_CCID_NO,      AT_CMD_CCID,      "+SCID:",     "+CME", AT_CMD_UNKNOWN},
{AT_CMD_CGATT_NO,     AT_CMD_CGATT,     "OK",         "+CME", AT_CMD_UNKNOWN},
{AT_CMD_CGACT_NO,     AT_CMD_CGACT,     "OK",         "+CME", AT_CMD_UNKNOWN},
{AT_CMD_CIPSTART_NO,  AT_CMD_CIPSTART,  "CONNECT OK", "+CME", AT_CMD_UNKNOWN},
{AT_CMD_CIPSEND_NO,   AT_CMD_CIPSEND,   ">",          "+CME", AT_CMD_UNKNOWN},
{AT_CMD_CIPCLOSE_NO,  AT_CMD_CIPCLOSE,  "OK",         "+CME", AT_CMD_UNKNOWN},
{AT_CMD_IMEI_NO,      AT_CMD_IMEI,      "+EGMR:",     "+CME", AT_CMD_UNKNOWN},
{AT_CMD_CIPDATA_NO,   AT_CMD_DATA,      "OK",         "+CME", AT_CMD_UNKNOWN},
{AT_CMD_CGDCONT_NO,   AT_CMD_CGDCONT,   "OK",         "+CME", AT_CMD_UNKNOWN},
};

extern uint8 server_stage;

extern char uart4_buf[MAX_GPRS_BUF_SIZE];
extern uint16 uart4_cnt;
extern uint16 uart4_preCnt;
extern void UART4_Puts(char * str);

extern uint8 glob_sbuf[MAX_SMALL_BUF_NUM][MAX_SMALL_BUF_SIZE];
extern uint8 glob_sbuf_state[MAX_SMALL_BUF_NUM];
extern uint8 glob_lbuf_state[MAX_LARGE_BUF_NUM];
extern BUFINFO glob_buf_info[MAX_SMALL_BUF_NUM];

void GPIO_GSM_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  
  RCC_APB2PeriphClockCmd(RCC_GPIO_GSM, ENABLE);
  //Port PC4/PC6/PC7 output
  GPIO_InitStructure.GPIO_Pin   = GSM_PWR | GSM_LPM | GSM_RST;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
  GPIO_Init(GPIO_GSM, &GPIO_InitStructure);
}

void GPIO_GSM_Init(void)
{
  GPIO_GSM_Configuration();//io config
  GSM_PWR_ON;//power on
  GSM_LPM_OFF;//not in low power mode 
  GSM_RST_OFF;//not enable reset
}

void A6_MODULE_ON(void)
{
  GSM_RST_OFF;//not enable reset
  GSM_PWR_ON;//power on
}

void A6_MODULE_OFF(void)
{
  GSM_PWR_OFF;//power off
  GSM_RST_ON;//enable reset   
}

void Reset_GSM_MODULE(void)
{
  uint8 i;
  A6_MODULE_OFF();
  delay_ms(1000);
  A6_MODULE_ON();
  delay_ms(1000);
  
  server_stage = 0; 
  g_sendAtCmdNo = 0;//send AT command number
  g_gprs_Stimeout=0;
  g_gprsSState = GPRS_SBUF_EMPTY;//GPRS send state
  g_gprsRState = GPRS_RBUF_EMPTY;//GPRS receive state
  
  g_gprs_comtimes = 0;
  g_gprsConState = GPRS_CONSTATE_UART;//reconnect net
  for(i=0; i<CMD_TABLE_LENGTH; i++)  
    at_cmd_table[i].result=AT_CMD_UNKNOWN;
}

unsigned char mem_cmp(char* src, char* dst,unsigned char len)
{
  unsigned char i=0,ret=0;
  
  while(i<len)
  {
    if(src[i] ==0 || dst[i]==0 ||  src[i]!= dst[i])
    {
      ret=1;
      break;
    } 
    i++;
  }
  return ret;
}

unsigned char mem_cmp_at(char* src, char* dst, uint16 ustart, unsigned char len)
{
  unsigned char i=0,ret=0;
  
  while(i<len)
  {
    if(ustart + i>=MAX_GPRS_BUF_SIZE)
      dst[i] = uart4_buf[ustart + i - MAX_GPRS_BUF_SIZE];
    if(src[i] ==0 || dst[i]==0 ||  src[i]!= dst[i])
    {
      ret=1;
      break;
    } 
    i++;
  }
  return ret;
}

unsigned char str_len(char* string)
{
   u8 i=0;

   while(string[i]!=0) 	i++;

   return i;
}

void SendAtCmd(char * cmd, unsigned char atCmdNo)
{
  g_sendAtCmdNo = atCmdNo;//command number
  at_cmd_table[g_sendAtCmdNo-1].result = AT_CMD_UNKNOWN;//set AT command state
  UART4_Puts(cmd);//send AT command data   
}

//"AT+CIPSEND=20\r\n"//send GPRS message
uint16 at_model_table[5]={10000,1000,100,10,1};
void SetAtSendLength(uint16 udatLength)
{
  uint8 i,j,ulenChar[5];
  
  for(i = 0; i<5; i++)//计算每一位的数值
  {
    ulenChar[i] = (udatLength / at_model_table[i])%10;
  }
  for(i = 0; i<5; i++)//找到第一个不是0的位
  {
    if(ulenChar[i]!=0)
      break;      
  }
  for(j=0; j<(5-i); j++)//数据搬移
  {
    ulenChar[j] = ulenChar[i+j]+0x30;//数字传换成字符
  }
  
  g_sendAtCmdNo = AT_CMD_CIPSEND_NO;//command number
  at_cmd_table[g_sendAtCmdNo-1].result = AT_CMD_UNKNOWN;//set AT command state
  UART4_Puts("AT+CIPSEND=");
  for(i=0; i<j; i++)
    UART4_Putc(ulenChar[i]);
  UART4_Puts("\r\n");//send AT command data  
}

void SendAtData(uint8 *str, uint16 len)
{
  uint8 i;
  g_sendAtCmdNo = AT_CMD_CIPDATA_NO;//command number
  at_cmd_table[g_sendAtCmdNo-1].result = AT_CMD_UNKNOWN;//set AT command state
  for(i=0; i<len; i++)
  {
    USART_SendData(UART4, *str++);
    while(USART_GetFlagStatus(UART4, USART_FLAG_TXE) == RESET);
  }
}

void ReConnectGPRS(void)
{
  server_stage = 0;
  g_gprsConState = GPRS_CONSTATE_CLOSE;//reconnect net 
  g_sendAtCmdNo = 0;//send AT command number
  g_gprs_Stimeout=0;
  g_gprsSState = GPRS_SBUF_EMPTY;//GPRS send state
  g_gprsRState = GPRS_RBUF_EMPTY;//GPRS receive state
  at_cmd_table[AT_CMD_CIPCLOSE_NO-1].result=AT_CMD_UNKNOWN;
  at_cmd_table[AT_CMD_CIPSTART_NO-1].result=AT_CMD_UNKNOWN;
}

//interrupt data analysis
uint8 DataLayerGSM(void)
{
  uint16 i;
  char *pSuccStr;
  char *pRStr;
  uint8 succStrLen = 0;
  uint16 udataStart = 0;
  uint16 udataLen = 0;
     
  if(g_sendAtCmdNo>0)//check is AT connect command
  {
    //get command detection characters 
    pSuccStr = at_cmd_table[g_sendAtCmdNo-1].success;
    //size of pSuccStr
    succStrLen = str_len(pSuccStr);
  } 
  while(uart4_buf[uart4_preCnt]==0)
  {    
    uart4_preCnt++;//filter blank character
    if(uart4_preCnt>MAX_GPRS_BUF_SIZE)
      uart4_preCnt=0;
  }
//------------------------------------------  
  pRStr = &uart4_buf[uart4_preCnt];//get valid data point
  udataStart = uart4_preCnt;//get valid data start
  if(uart4_cnt>=uart4_preCnt)//get valid data length
    udataLen = uart4_cnt - uart4_preCnt;
  else
    udataLen = MAX_GPRS_BUF_SIZE + uart4_cnt - uart4_preCnt;
//------------------------------------------
  uart4_preCnt = uart4_cnt;//next data start addr value
  
  //set send command length
  if(g_sendAtCmdNo==AT_CMD_CIPSEND_NO)
  {
    if(mem_cmp_at(pSuccStr,pRStr,udataStart,succStrLen)==0)//OK
    {
      at_cmd_table[g_sendAtCmdNo-1].result=AT_CMD_SUCCESS; //send command success
      g_sendAtCmdNo=0;//clear AT command number
    }
    else
      at_cmd_table[g_sendAtCmdNo-1].result=AT_CMD_FAILED;
  }
  //check receive data from server
  if(mem_cmp_at("+CIPRCV:",pRStr,udataStart,8)==0)//OK
  {
    //find enpty state buf
    for(i=0; i<GPRS_RBUF_COUNT; i++)
    {
      if(g_gprsRBufState[i]==GPRS_RBUF_EMPTY)
        break;
    }
    //use last buf
    if(i>=(GPRS_RBUF_COUNT-1))
      i = GPRS_RBUF_COUNT-1;
    g_gprsRBufAddr[i] = udataStart;//uart3_preCnt//save data start addr
    g_gprsRBufState[i] = GPRS_RECEIVE_READY;//change buf state
    return 0;
  }
  //check data frame is current 
  if(!(uart4_buf[uart4_cnt-2]=='\r'&&uart4_buf[uart4_cnt-1]=='\n'))
    return 0;    
  //check is not a blank line
  if(udataLen==2)
    return 0;
  //check receive command is tcpip close 
  if(mem_cmp_at("+TCPCLOSED:",pRStr,udataStart,11)==0)//OK
  {
    ReConnectGPRS();
    return 0;
  }
  //check receive command is SIM not inserted 
  if(mem_cmp_at("SIM not inserted",pRStr,udataStart,16)==0)//OK
  {
    ReConnectGPRS();
    return 0;
  }
  //AT command data analysis
  switch(g_sendAtCmdNo)
  {
    case AT_CMD_AT_NO://AT
    case AT_CMD_ATE0_NO://ATE0  
    case AT_CMD_CREG_NO:
    case AT_CMD_CSQ_NO:
    case AT_CMD_CGATT_NO:
    case AT_CMD_CGDCONT_NO: 
    case AT_CMD_CGACT_NO:
    case AT_CMD_CIPSTART_NO://connect net
    case AT_CMD_CIPCLOSE_NO://close net      
      if(mem_cmp_at(pSuccStr,pRStr,udataStart,succStrLen)==0)//OK
      {
        at_cmd_table[g_sendAtCmdNo-1].result=AT_CMD_SUCCESS; //send command success
        g_sendAtCmdNo=0;//clear AT command number
        g_gprs_comtimes = 0;
      }
      else if(mem_cmp_at("+CME",pRStr,udataStart,4)==0)//OK
      {
        at_cmd_table[g_sendAtCmdNo-1].result=AT_CMD_FAILED;    
      }
      break;      
    case AT_CMD_CCID_NO://get CCID code
      if(mem_cmp_at(pSuccStr,pRStr,udataStart,succStrLen)==0)//OK
      {
        at_cmd_table[g_sendAtCmdNo-1].result=AT_CMD_SUCCESS; //send command success
        g_sendAtCmdNo=0;//clear AT command number
        for(i=0; i<20; i++)
        {
          if((udataStart+19+i)>= MAX_GPRS_BUF_SIZE)
            g_simCCID[i] = *(pRStr + 19 + i - MAX_GPRS_BUF_SIZE);
          else  
            g_simCCID[i] = *(pRStr + 19 + i);//save CCID code 
        }
      }
      else
      {
        at_cmd_table[g_sendAtCmdNo-1].result=AT_CMD_FAILED;
      }
      break;
    case AT_CMD_IMEI_NO://get IMEI CODE
      if(udataLen==17)
      {
        at_cmd_table[g_sendAtCmdNo-1].result=AT_CMD_SUCCESS; //send command success
        g_sendAtCmdNo=0;//clear AT command number
        for(i=0; i<15; i++)
        {
          if((udataStart+i)>= MAX_GPRS_BUF_SIZE)
            g_simIMEI[i] = *(pRStr + i - MAX_GPRS_BUF_SIZE);
          else  
            g_simIMEI[i] = *(pRStr + i);//save IMEI code
        }
      }
      else
        at_cmd_table[g_sendAtCmdNo-1].result=AT_CMD_FAILED;
      break;
    case AT_CMD_CIPDATA_NO:
      if(mem_cmp_at(pSuccStr,pRStr,udataStart,succStrLen)==0)//OK
      {
        g_gprsSState = GPRS_SBUF_EMPTY;
        g_gprs_Stimeout=0;
        g_sendAtCmdNo=0;//clear AT command number
      }
      else
      {
      }
      break;
    default:
    break;
  }  
  return 0;  
}

void ReceiveAtData(uint8 rbufIdx)
{
  uint16 i,j;
  char urlentemp;
  uint8 urlenTable[5];
  uint16 urlenStart;
  uint16 urlen_gprs;
  uint16 urlen_data;
  
  urlenStart = g_gprsRBufAddr[rbufIdx];//get data start addr
  for(i=8; i<MAX_GPRS_BUF_SIZE; i++)//find ending char of length
  {
    if((urlenStart + i)>=MAX_GPRS_BUF_SIZE)
      urlentemp = uart4_buf[urlenStart + i - MAX_GPRS_BUF_SIZE];
    else
      urlentemp = uart4_buf[urlenStart + i];
    if(urlentemp == ',')
      break;
  } 
  j=i;//get ',' location
//---get gprs data length
  //low bit first
  for(i=0; i<(j-8); i++)//get length char
  {
    if((urlenStart + j-1 - i)>=MAX_GPRS_BUF_SIZE)
      urlenTable[i] = uart4_buf[urlenStart + j-1 - i - MAX_GPRS_BUF_SIZE];
    else
      urlenTable[i] = uart4_buf[urlenStart + j-1 - i];
  }
  urlen_gprs = 0;
  for(i=0; i<(j-8); i++)//calculate length value
  {
    urlen_gprs = urlen_gprs + (urlenTable[i]-'0')*at_model_table[4-i];
  }
  
  if(g_gprsRState == GPRS_RECEIVE_ING)
  {
    g_gprsRBufState[rbufIdx] = GPRS_RBUF_EMPTY;
    if((g_gprsmCnt + urlen_gprs)>=g_gprsLength)//complete
    {
      g_gprsRState = GPRS_RECEIVE_READY;//
      g_gprsmCnt = 0;
      g_gprsLength = 0;
    }
    else//data not complete
    {
      g_gprsRState = GPRS_RECEIVE_ING;//
      g_gprsmCnt = g_gprsmCnt + urlen_gprs;
    }
    for(i=0; i<urlen_gprs; i++)
    {
      if((urlenStart + j+1 + i)>=MAX_GPRS_BUF_SIZE)        
        g_gprsRBUF[g_gprsCnt] = uart4_buf[urlenStart + j+1 + i - MAX_GPRS_BUF_SIZE];
      else
        g_gprsRBUF[g_gprsCnt] = uart4_buf[urlenStart + j+1 + i];
      g_gprsCnt++;
      if(g_gprsCnt>=MAX_GPRS_BUF_SIZE)
        g_gprsCnt = 0;
    }   
  }
  else
  {
    urlen_data = 0;
    if((urlenStart + j + 1)>=MAX_GPRS_BUF_SIZE)
      urlen_data = uart4_buf[urlenStart + j + 1 - MAX_GPRS_BUF_SIZE];
    else
      urlen_data = uart4_buf[urlenStart + j + 1];
    if((urlenStart + j + 2)>=MAX_GPRS_BUF_SIZE)
      urlen_data = (urlen_data<<8) + uart4_buf[urlenStart + j + 2 - MAX_GPRS_BUF_SIZE];
    else
      urlen_data = (urlen_data<<8) + uart4_buf[urlenStart + j + 2];
    
    if(urlen_data > urlen_gprs)//data not complete
    {
      g_gprsRState = GPRS_RECEIVE_ING;//
      g_gprsmCnt = urlen_gprs;//save count value
      g_gprsLength = urlen_data;
      g_gprsRBufState[rbufIdx] = GPRS_RBUF_EMPTY;//clear buf state
      for(i=0; i<urlen_gprs; i++)//save buf data
      {
        if((urlenStart + j+1 + i)>=MAX_GPRS_BUF_SIZE)        
          g_gprsRBUF[g_gprsCnt] = uart4_buf[urlenStart + j+1 + i - MAX_GPRS_BUF_SIZE];
        else
          g_gprsRBUF[g_gprsCnt] = uart4_buf[urlenStart + j+1 + i];
        g_gprsCnt++;
        if(g_gprsCnt>=MAX_GPRS_BUF_SIZE)
          g_gprsCnt = 0;
      }
    }
    else //if(urlen_gprs==urlen_data)
    {
      g_gprsRState = GPRS_RECEIVE_READY;//
      g_gprsRBufState[rbufIdx] = GPRS_RBUF_EMPTY;
      for(i=0; i<urlen_data; i++)
      {
        if((urlenStart + j+1 + i)>=MAX_GPRS_BUF_SIZE)        
          g_gprsRBUF[g_gprsCnt] = uart4_buf[urlenStart + j+1 + i - MAX_GPRS_BUF_SIZE];
        else
          g_gprsRBUF[g_gprsCnt] = uart4_buf[urlenStart + j+1 + i];
        g_gprsCnt++;
        if(g_gprsCnt>=MAX_GPRS_BUF_SIZE)
          g_gprsCnt = 0;
      }
    }
  }
}

//Get GPRS data length
uint16 CombineGPRS_RLen(void)
{
  uint16 urlen;
  urlen = 0;  
  urlen = g_gprsRBUF[g_gprspCnt];//High byte
  if(g_gprspCnt+1>=MAX_GPRS_BUF_SIZE)//Low Byte
    urlen = (urlen<<8) + g_gprsRBUF[g_gprspCnt+1-MAX_GPRS_BUF_SIZE];
  else
    urlen = (urlen<<8) + g_gprsRBUF[g_gprspCnt+1];
  
  return urlen;
}

void SaveGPRSData(uint8 idx, uint16 urlen)
{
  uint16 left, appd;
  if(g_gprspCnt + urlen >= MAX_GPRS_BUF_SIZE)
  {
    left = MAX_GPRS_BUF_SIZE - g_gprspCnt;  // buffer tail data length
    appd = g_gprspCnt + urlen - MAX_GPRS_BUF_SIZE;  // buffer head data length
    CopyBufData(glob_sbuf[idx], &g_gprsRBUF[g_gprspCnt], left);
    CopyBufData(&glob_sbuf[idx][left], g_gprsRBUF, appd);
    g_gprspCnt = appd;
  }
  else
  {
    CopyBufData(glob_sbuf[idx], &g_gprsRBUF[g_gprspCnt], urlen);
    g_gprspCnt += urlen;
  } 
  glob_buf_info[idx].buf = glob_sbuf[idx];
  glob_buf_info[idx].channel = BUF_CHANNEL_TCPC;
  glob_buf_info[idx].len = urlen;
  glob_sbuf_state[idx] = SBUF_RECV_READY;
}

void SetGPRS_RState(uint16 urlen)
{
  uint16 left;
  if(g_gprsCnt >= g_gprspCnt)
    left = g_gprsCnt - g_gprspCnt;
  else
    left = g_gprsCnt + MAX_GPRS_BUF_SIZE - g_gprspCnt;
  if(left == 0)
  {
    g_gprsRState = GPRS_RBUF_EMPTY;
  }
  else if(left >= urlen)
  {
    g_gprsRState = GPRS_RECEIVE_READY;//full
  }
  else
  {
    g_gprsRState = GPRS_RECEIVE_ING;
  }
}

void SendGPRStoServer(uint8 idx, uint8 uclearbuf)
{
  uint8 i;
  if(g_gprsSState == GPRS_SBUF_EMPTY)
  {
    g_gprsSState = GPRS_SEND_WAIT;
    g_gprs_Stimeout=0;
    SetAtSendLength(glob_buf_info[idx].len);
    //add timeout handling
    i = 0;
    while((at_cmd_table[AT_CMD_CIPSEND_NO-1].result!=AT_CMD_SUCCESS)&&(i<200))
    {
      delay_ms(1);
      i++;//55
    }
    SendAtData(glob_buf_info[idx].buf, glob_buf_info[idx].len);
    if(uclearbuf)
      glob_sbuf_state[idx] = SBUF_EMPTY;
  }	
}

void WaitConnectGPRS(void)
{
  uint8 i;
  uint8 idx;
  uint16 urlen;
  //save GPRS data to gprsRBUF
  for(i=0; i<GPRS_RBUF_COUNT; i++)
  {
    if(g_gprsRBufState[i]==GPRS_RECEIVE_READY)
      ReceiveAtData(i);        
  }
  //save GPRS data to glob_sbuf
  if(g_gprsRState==GPRS_RECEIVE_READY)
  {
    idx = FindEmptySBuf();//Find empty buf
    if(idx >= MAX_SMALL_BUF_NUM)
    { // if buffer full, drop the last msg, do not block uart2 receive
      idx = MAX_SMALL_BUF_NUM - 1;    
    }    
    //read the received data
    urlen = CombineGPRS_RLen();
    if(urlen > 255)
    { // use large buf to receive
      idx = FindSBufState(SBUF_LARGE_RESERVE);
      SaveGPRSData(idx, urlen);							
    }
    else
    {      
      SaveGPRSData(idx, urlen);   
    }
    urlen = CombineGPRS_RLen();
    SetGPRS_RState(urlen);    
  }
  //send GPRS data
  idx = FindSReadySBuf();
  if((idx < MAX_SMALL_BUF_NUM) && (glob_buf_info[idx].channel == BUF_CHANNEL_TCPC))
  {
    //USART1_Puts(&idx,1);
    SendGPRStoServer(idx,1);
  }
  //updata new version code
  idx = FindSBufStateAndPort(SBUF_SEND_LARGE_RECV, BUF_CHANNEL_TCPC);
  if((idx < MAX_SMALL_BUF_NUM) && (glob_buf_info[idx].channel == BUF_CHANNEL_TCPC))
  {
    glob_sbuf_state[idx] = SBUF_EMPTY;				
    if(SetLargeBufferReserve() < MAX_SMALL_BUF_NUM)
    { //if large buffer ready, send this msg
			SendGPRStoServer(idx,0);
    }
    else
    { // else, delay msg sending
      glob_sbuf_state[idx] = SBUF_SEND_LARGE_RECV;									
    }
  }
}

void DataLayerTCPS(void)
{
  uint8 i;
  if(g_gprsConState != GPRS_CONSTATE_SUCC)
    return;
  for(i = 0; i < MAX_TCP_SERVER; i++)
  {
    WaitConnectGPRS(); 
  }
}

//Register GPRS mode connect to server
void RegisterCenterAtServer(void)
{
  if(at_cmd_table[AT_CMD_CIPSTART_NO-1].result==AT_CMD_SUCCESS && server_stage == 0)//if net is connect
  {
    g_gprsConState = GPRS_CONSTATE_SUCC;//connceted
    if(ReportCenterHSVersionToServer() == 1)
			server_stage = 1;   
    return;
  }
  else if(server_stage == 1)
	{
	  if(GetRealTimeFromServer() == 1)
		{
			server_stage = 2;
		}
    return;
	}
  //timeout
  if(at_cmd_table[AT_CMD_CIPSTART_NO-1].result!=AT_CMD_SUCCESS)
  {
    g_gprs_comtimes ++;
    if(g_gprs_comtimes>5)
    {
      Reset_GSM_MODULE();
      return;
    }
  }
  //Switch connection state
  switch(g_gprsConState)
  {
    //GSM UART connceted
    case GPRS_CONSTATE_UART:
      if(at_cmd_table[AT_CMD_AT_NO-1].result==AT_CMD_SUCCESS)
        g_gprsConState = GPRS_CONSTATE_ATE0;//Change connection state
        //g_gprsConState = GPRS_CONSTATE_CREG;//Change connection state
      else
        SendAtCmd(AT_CMD_AT,AT_CMD_AT_NO);//check GPRS module UART
      break;
    //close echo
    case GPRS_CONSTATE_ATE0:
      if(at_cmd_table[AT_CMD_ATE0_NO-1].result==AT_CMD_SUCCESS)
        g_gprsConState = GPRS_CONSTATE_CREG;//Change connection state
      else
        SendAtCmd(AT_CMD_ATE0,AT_CMD_ATE0_NO);//close echo
      break;  
    //check network
    case GPRS_CONSTATE_CREG:
      if(at_cmd_table[AT_CMD_CREG_NO-1].result==AT_CMD_SUCCESS)
        g_gprsConState = GPRS_CONSTATE_CGATT;//Change connection state
      else
        SendAtCmd(AT_CMD_CREG,AT_CMD_CREG_NO);//close echo
      break;
    //attach GPRS network  
    case GPRS_CONSTATE_CGATT:
      if(at_cmd_table[AT_CMD_CGATT_NO-1].result==AT_CMD_SUCCESS)
        g_gprsConState = GPRS_CONSTATE_CGDCONT;//Change connection state
      else
        SendAtCmd(AT_CMD_CGATT,AT_CMD_CGATT_NO);//attach GPRS network
      break;
    case GPRS_CONSTATE_CGDCONT:
      if(at_cmd_table[AT_CMD_CGDCONT_NO-1].result==AT_CMD_SUCCESS)
        g_gprsConState = GPRS_CONSTATE_CGACT;//Change connection state
      else
        SendAtCmd(AT_CMD_CGDCONT,AT_CMD_CGDCONT_NO);//attach GPRS network
      break;
    //activate GPRS network  
    case GPRS_CONSTATE_CGACT:
      if(at_cmd_table[AT_CMD_CGACT_NO-1].result==AT_CMD_SUCCESS)
        g_gprsConState = GPRS_CONSTATE_IPSTART;//Change connection state
      else
        SendAtCmd(AT_CMD_CGACT,AT_CMD_CGACT_NO);//activate GPRS network
      break;
    //conncet to server  
    case GPRS_CONSTATE_IPSTART:
      if(at_cmd_table[AT_CMD_CIPSTART_NO-1].result==AT_CMD_FAILED)
      {
        g_gprsConState = GPRS_CONSTATE_CLOSE;//close conncet
        at_cmd_table[AT_CMD_CIPCLOSE_NO-1].result=AT_CMD_UNKNOWN;
      }
      else
      {
        UART4_Puts("AT+CIPSTART=");
        SendAtCmd(AT_CMD_CIPSTART,AT_CMD_CIPSTART_NO);//check GPRS attach network
      }
      break;
    //close net connect  
    case GPRS_CONSTATE_CLOSE:
      if(at_cmd_table[AT_CMD_CIPCLOSE_NO-1].result==AT_CMD_SUCCESS)
      {      
        Reset_GSM_MODULE();
      }
      else
      {
        if(g_gprs_comtimes>5)
        {
          Reset_GSM_MODULE();
        }
        else
        {
          g_gprsConState = GPRS_CONSTATE_CLOSE;//close connceted
          SendAtCmd(AT_CMD_CIPCLOSE,AT_CMD_CIPCLOSE_NO);//close GPRS connect        
        }
      }
      break;      
    default:
      break;
  }
}

//Uart interrupt processing
void UART4_IRQHandler(void)
{
  if(USART_GetITStatus(UART4, USART_IT_RXNE) != RESET)
  {
	  USART_ITConfig(UART4, USART_IT_RXNE, DISABLE);
    if(uart4_cnt >= MAX_GPRS_BUF_SIZE)
      uart4_cnt = 0;
    uart4_buf[uart4_cnt] = USART_ReceiveData(UART4);
    uart4_cnt ++;
    if(uart4_cnt==1)
    {
      if(((uart4_buf[MAX_GPRS_BUF_SIZE-1]=='\r')&&(uart4_buf[0]=='\n')) \
        ||((g_sendAtCmdNo==AT_CMD_CIPSEND_NO)&&(uart4_buf[0]=='>')))//receive end of AT command
      {
        DataLayerGSM();      
      }
    }
    else
    {
      if(((uart4_buf[uart4_cnt-2]=='\r')&&(uart4_buf[uart4_cnt-1]=='\n')) \
        ||((g_sendAtCmdNo==AT_CMD_CIPSEND_NO)&&(uart4_buf[uart4_cnt-1]=='>')))//receive end of AT command
      {
        DataLayerGSM();      
      }
    }
  	USART_ITConfig(UART4,USART_IT_RXNE, ENABLE);	
  }
}

#endif
