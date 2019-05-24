//*****************************************************************
//
//	2018.12.05	S3 as HR sample pulse output/ S4 as RTL test input
//	2018.12.21	sample rate = ADC per 4ms , sample AN0 per 40ms
//		AN2SampValue[AN2_ARY_SAMPLE_SIZE] only sample per 5*4ms
//	2018.12.24	CheckValResonable() / OVER_TH_LIMIT=100
//		AN2 avg 5 time(AN2_AVG_5_CHK_BUF_FUN)
//	2018.12.25	HR_BPMvalue_NotGood to check Heart Rate out of range
//				set NSTROBE_LOW_EndSet = 0 error
//	2018.12.27	add "AGC_MCP4011_Gain"" send RTL
//	2018.12.28	UART msg Ending = /r /n
//	2019.01.09	Disable Debug Message(AGC_MCP4011_DUG_MSG_FUN)
//	2019.02.25	Send [inter-beat interval (IBI)] OR [R-R interval] in Hex format
//	2019.03.12	In [case AN0_UNDER_RANGE_Event:] reset AGC_MCP4011_Gain to MIDGAIN=32
//	2019.03.22	mail: Date:  Thu, 21 Mar 2019 send the one data point when you collect an IBI.
//				2019.03.27 test ok
//	2019.05.10	ack 增加 PIC version number 訊息
//*****************************************************************

#include "mcc_generated_files/mcc.h"
#include "mcc_generated_files/HM_HeartRate.h"

#define		KEY_IN_FUN	0
#define		AN2_AVG_5_CHK_BUF_FUN		0
#define		HR_NOT_GOOD_VALUE_FUN		0


#define		SEND_toRTL_CMD_ackSIZE			6
#define		SEND_toRTL_CMD_VerNum_ackSIZE	9
#define		SEND_toRTL_CMD_SIZE				5
#define		RxBuffer_SIZE					8

#define		RTL_PWR_ON				0
#define		RTL_cmd_err				0xff

#define		S3_PIN	IO_RA2_PORT	
#define		S4_PIN	IO_RC7_PORT

#define		GO_UP		1
#define		GO_DOWN		0

const uint8_t	PIC_cmd_pwrON[SEND_toRTL_CMD_SIZE] = { 'B', 'O' , 'P' , '\r' , '\n'} ;
const uint8_t	PIC_cmd_Error[SEND_toRTL_CMD_SIZE] = { 'E', 'R' , 'R' , '\r' , '\n'} ;

const uint8_t RTL_pwrON_ack[SEND_toRTL_CMD_ackSIZE] = { 'R', 'T' , 'L' , 'A', '\r' , '\n'} ;
// 2019.05.10 增加 PIC version number
//const uint8_t RTL_HWEnable_ack[SEND_toRTL_CMD_VerNum_ackSIZE] = { 'E', 'N' , 'B' ,'a','0','0','-', '\r' , '\n'} ;
uint8_t RTL_HWEnable_ack[SEND_toRTL_CMD_VerNum_ackSIZE] = { 'E', 'N' , 'B' ,'a','0','0','-', '\r' , '\n'} ;

const uint8_t RTL_HWStop_ack[SEND_toRTL_CMD_ackSIZE] = { 'S', 'T' , 'P' , 'A', '\r' , '\n'} ;


extern uint16_t	Tmr1_4ms_cnt;
extern uint16_t Tmr1_2sec_cnt;
extern uint16_t Tmr1_1sec_cnt;
extern uint8_t 	Tmr1_upDate;	
extern uint8_t	NSTROBE_PWM_cnt;
extern uint8_t 	AGC_MCP4011_Gain;  // current setting

extern uint8_t NSTROBE_LOW_EndSet;
extern uint8_t NSTROBE_Rset;


SystemControlStatus_Event	ControlStatus_Event ; 
extern ADCcurrentChan_Event	CurrentChan;

adc_result_t	AN0SampAvgValue;
adc_result_t	AN2MaxValue,AN2MinValue,AN2_ChkValue,AN2_SampAvgBuf_Value;
adc_result_t	AN2_HRminThresholdValue;
uint32_t		AN2_AryAvg_value;

uint8_t			AN2MaxPos,AN2MinPos;

adc_result_t	AN0SampValue[AN0_SAMPLE_QUEUE_SIZE];
adc_result_t	AN2SampValue[AN2_ARY_SAMPLE_SIZE];
adc_result_t	AN2SampAvgBufValue[AN2_SAMPLE_AVGBUF_SIZE];
//adc_result_t	AN2Samp_Inv4s_Value[AN2_SAMPLE_Inv4s_SIZE];


ADCcurrentChan_Event	CurrentChan;
uint8_t an0SampleCount;
extern uint16_t an2SampleCnt;


volatile uint8_t RxBuffer[RxBuffer_SIZE];
uint8_t cnt=0;

uint8_t S3_cnt=0;
uint8_t S4_cnt=0;
uint8_t AGC_MCP4011_Gain; 
uint8_t AGC_MCP4011_Dir;  
//uint8_t FindSetPoint_AN0_Delay;  
extern bit bADC_AN0_Delay ;

extern uint8_t AGC_MCP4011_GainLimitSet,AN2_GainContOutOfRange_CycleCnt;



bit bS3_PIN_Push ;
bit bS4_PIN_Push ;

bit bS3_PIN_PushMsgON ;
bit bS4_PIN_PushMsgON ;

extern bit bNDISCH_GoLow;
adc_channel_t	HM_ADC_CH;
adc_result_t	ARconvertedValue;

extern bit bAN0_ADC_ON,bSendToBT_timer_Flag,bSendToBT_StartTimer_Flag ;
extern bit bAN0_InRange,bTmr1_1sec_Flag;


bit bAR_ADC_DugOn ;
extern bit bAN0_ADC_DugOn;
bit bCMD_AUTO_FIND_SET_POINT;
bit bAN2_GainContInRange;
extern bit bAN2_GainContWaitFlag;
extern bit bAN2_LowAmplitudeFlag;
extern uint8_t AN2_GainContInRange_cnt;

uint16_t AN2_PulseIntervalAvg,AN2_READ_PulseIntervalAvg_1msCnt; // AN2_READ_PulseIntervalAvg
uint16_t AN2_tmpPulseCnt, AN2_OverThreshold_cnt;

uint16_t HR_BPMvalue_NotGood,HR_BPMvalue_1msCnt;
extern uint8_t InRangeStatus,AN2_HRBufferIndex;
extern uint16_t AN2_oldPulseCnt;

extern uint16_t Tmr0_1ms_cnt;


uint16_t AN2_HRBuffer[AN2HR_BUF_SIZE];


bit bGet_InitValToCopmpare,bGet_MinVal_InCycleTime;

//--------------------------------
// Modify resonable value 
// 1) |OLD - NEW| <= +/- 5 --> GOOD
// 2) |OLD - NEW| > +/- 5  --> 1/2*NEW + 1/2*OLD
//--------------------------------

void CheckValResonable(void)
{
	if(( AN2_tmpPulseCnt <= (AN2_oldPulseCnt - VAL_NN_INTERVAL_DIFF)) 
		|| ( AN2_tmpPulseCnt >= (AN2_oldPulseCnt + VAL_NN_INTERVAL_DIFF))){
		
		AN2_tmpPulseCnt = (AN2_tmpPulseCnt + AN2_oldPulseCnt)/2;
	#if HEART_RATE_DUG_MSG_FUN
		DugDataMsg('1','h','r', AN2_tmpPulseCnt );
	#endif 
	}	
}


//---------------------------------------------------------
// Heart Rate Calculate
//---------------------------------------------------------

void Cal_HeartRate(void)
{
	uint8_t i;

	if(AN2_AryAvg_value > AN2_ChkValue){
		
		//-------------------------------------
		// Under [AN2_AryAvg_value] IS start point
		//-------------------------------------
		
		if(bGet_InitValToCopmpare  == FALSE){
			//AN2_HRminThresholdValue = AN2_AryAvg_value - AN2_ChkValue;
			
			IO_RA2_SetHigh(); // S3 key
			
			//AN2_tmpPulseCnt = 0;
			Tmr0_1ms_cnt = 0;
			bGet_InitValToCopmpare  = TRUE;
			bGet_MinVal_InCycleTime = FALSE;
		}
		//---------------------------------
		// Under [AN2_AryAvg_value] again 
		// IS end point
		//---------------------------------

		if( bGet_MinVal_InCycleTime == TRUE){			
			IO_RA2_SetLow(); // S3 key
			AN2_tmpPulseCnt = Tmr0_1ms_cnt;
				
			AN2_oldPulseCnt = AN2_tmpPulseCnt; // 直接計算不調整
			AN2_READ_PulseIntervalAvg_1msCnt = AN2_tmpPulseCnt; // Tmr0 counter
			bSendToBT_timer_Flag = TRUE;
			bGet_InitValToCopmpare	= FALSE;
		}
		
		AN2_OverThreshold_cnt = 0;
	}
	
	//-------------------------------
	// Over [AN2_AryAvg_value] point
	//-------------------------------

	else{
		if( bGet_InitValToCopmpare	== TRUE )	AN2_OverThreshold_cnt ++; 
		//
		// keep over [AN2_AryAvg_value] 
		//
		if((AN2_OverThreshold_cnt > OVER_TH_LIMIT) 
			&& (bGet_MinVal_InCycleTime == FALSE) )	bGet_MinVal_InCycleTime = TRUE;
	}	
	
}

//------------------------------------
// What is the cmd from RTL
// -----------------------------------
uint8_t RTLCmdCheck(uint8_t RTLcm0,uint8_t RTLcm1,uint8_t RTLcm2)
{
	SystemControlStatus_Event	ControlStatus_Event ; 

	ControlStatus_Event = RTL_cmdError_Event;
	
	if((RTLcm0 == 'R') && (RTLcm1 == 'T') && (RTLcm2 == 'L')){
		for( int i=0; i < SEND_toRTL_CMD_ackSIZE ;i++)
	    			EUSART_Write( RTL_pwrON_ack[i] );
		ControlStatus_Event = RTL_PwrOncmd_Event;
		return ControlStatus_Event;
	}
	else if((RTLcm0 == 'E') && (RTLcm1 == 'N')){
		//for( int i=0; i < SEND_toRTL_CMD_ackSIZE ;i++)
		for( int i=0; i < SEND_toRTL_CMD_VerNum_ackSIZE ;i++)
	    			EUSART_Write( RTL_HWEnable_ack[i] );
		ControlStatus_Event = RTL_HWEnablecmd_Event;
		AN2_oldPulseCnt = 0;
		AN2_READ_PulseIntervalAvg_1msCnt = 0;
		bSendToBT_timer_Flag = FALSE;
		bSendToBT_StartTimer_Flag = TRUE;
		Tmr1_2sec_cnt = 0;
		return ControlStatus_Event;
	}
	else if((RTLcm0 == 'S') && (RTLcm1 == 'T')){
		for( int i=0; i < SEND_toRTL_CMD_ackSIZE ;i++)
	    			EUSART_Write( RTL_HWStop_ack[i] );
		ControlStatus_Event = RTL_HWStopcmd_Event;
		return ControlStatus_Event;
	}
	
	else	return ControlStatus_Event;
	
}



/*
                         Main application
 */
void main(void)
{
	uint8_t TX_BUF;
	volatile uint8_t rxData;
	uint8_t ControlStatus_change;
	uint8_t i;
	uint16_t tmpUINT16_val;
	char	ver_moth,ver_date10,ver_date1,ver_dash;
	
    // initialize the device
    SYSTEM_Initialize();

    // When using interrupts, you need to set the Global and Peripheral Interrupt Enable bits
    // Use the following macros to:

    // Enable the Global Interrupts
    INTERRUPT_GlobalInterruptEnable();

    // Enable the Peripheral Interrupts
    INTERRUPT_PeripheralInterruptEnable();

    // Disable the Global Interrupts
    //INTERRUPT_GlobalInterruptDisable();

    // Disable the Peripheral Interrupts
    //INTERRUPT_PeripheralInterruptDisable();

	ControlStatus_Event = PowerOn_Event;
	Tmr1_4ms_cnt=0;	
	Tmr1_1sec_cnt=0;	
	Tmr1_2sec_cnt=0;	
	Tmr1_upDate=0;	
	ControlStatus_change=0;
	NSTROBE_LOW_EndSet=1;

	S3_cnt=0;
	bS3_PIN_PushMsgON = TRUE;
	bS3_PIN_Push = FALSE;
	S4_cnt=0;
	bS4_PIN_PushMsgON = TRUE;
	bS4_PIN_Push = FALSE;
	NSTROBE_Rset = 0;
	
	an0SampleCount = 0;
	
	bAR_ADC_DugOn = FALSE;
	bAN0_ADC_DugOn = FALSE ; 
	
	bCMD_AUTO_FIND_SET_POINT = TRUE ;
	
	//AN2_tmpPulseCnt = 0;
	bSendToBT_StartTimer_Flag = FALSE;
	bTmr1_1sec_Flag = FALSE;

	IO_RA2_SetDigitalOutput(); 	//S3_PIN
	IO_RC7_SetDigitalOutput();	//S4_PIN

	ver_moth = '5';
	ver_date10 = '1';
	ver_date1='0';
	ver_dash ='0';

	DugCmdMsg('V','0',ver_moth); // version number month is 

	
	DugCmdMsg(ver_date10,ver_date1,ver_dash); // version number day is // test 0417 for hex file

	
	RTL_HWEnable_ack[3] = ver_moth;
	RTL_HWEnable_ack[4] = ver_date10;
	RTL_HWEnable_ack[5] = ver_date1;
	RTL_HWEnable_ack[6] = ver_dash;
	
    TMR1_StartTimer();	// start 50ms counter
    
    // GO_nDONE stop; ADON enabled; CHS AN10; 
    //ADCON0 = 0x29;
    ADCON0 = 0x28; //  ADON DISabled

	if( ControlStatus_Event == PowerOn_Event ){
		for( int i=0; i < SEND_toRTL_CMD_SIZE ;i++)
    		EUSART_Write( PIC_cmd_pwrON[i] );
	}

    while (1)
    {
    	//---------------------------------------
        // check RX & what is the cmd from RTL
    	//---------------------------------------
    	
		if((bTmr1_1sec_Flag == TRUE) && (EUSART_is_rx_ready() > 4)){ // check RX per 1sec
			bTmr1_1sec_Flag = FALSE;
			cnt = 0;
			do{
				rxData = EUSART_Read();
	            RxBuffer[ cnt ] = rxData ;			
				cnt++;
			}while(eusartRxCount>0);
			
			if(( RxBuffer[ 3 ] == '\n' ) && ( RxBuffer[ 4 ] == '\r' )){ // for RTL NOT test
			//if(( RxBuffer[ 3 ] == '\r' ) && ( RxBuffer[ 4 ] == '\n' )){ // for RealTerm test only
				ControlStatus_Event = RTLCmdCheck(RxBuffer[ 0 ],RxBuffer[ 1 ],RxBuffer[ 2 ]);
				ControlStatus_change = 1;
			}

			for(i = 0 ; i<RxBuffer_SIZE ; i++)	RxBuffer[i] = 0; // Clear RxBuffer[]
        }

	#if KEY_IN_FUN
		if(Tmr1_4ms_cnt%20 == 0) { 
			//----------------------------------------------
			// check S3 per 200 msec
			//----------------------------------------------
			if(S3_PIN == LOW){
				if( (bS3_PIN_PushMsgON == TRUE) && (S3_cnt == 0) ) {
					//DugCmdMsg('S','3' ,'p');
					bS3_PIN_PushMsgON = FALSE;
				}
				if(S3_cnt > 3){
					bS3_PIN_Push = TRUE;
					//DugCmdMsg('S','3','P');
				}
				S3_cnt++;
			}
			else if(S3_PIN == HIGH) {			
				S3_cnt=0;
				bS3_PIN_PushMsgON = TRUE;
			}
			if((bS3_PIN_Push == TRUE) && (S3_PIN == HIGH)) {
				NSTROBE_LOW_EndSet++;
				if( NSTROBE_LOW_EndSet > 9 ) NSTROBE_LOW_EndSet=1;
				DugCmdMsg('N','L',HexNumTable[NSTROBE_LOW_EndSet] );
				bS3_PIN_Push = FALSE;
			}
			//----------------------------------------------
			// check S4 per 2 msec
			//----------------------------------------------
			if(S4_PIN == LOW){
				if( (bS4_PIN_PushMsgON == TRUE) && (S4_cnt == 0) ) {
					//DugCmdMsg('S','4' ,'p');
					bS4_PIN_PushMsgON = FALSE;
				}
				if(S4_cnt > 3){
					bS4_PIN_Push = TRUE;
					//DugCmdMsg('S','4','P');
				}
				S4_cnt++;
			}
			else if(S4_PIN == HIGH) {			
				S4_cnt=0;
				bS4_PIN_PushMsgON = TRUE;
			}
			if((bS4_PIN_Push == TRUE) && (S4_PIN == HIGH)) {
				NSTROBE_Rset++;
				if( NSTROBE_Rset > 3 ) NSTROBE_Rset = 0;
				DugCmdMsg('R','S',HexNumTable[NSTROBE_Rset] );
				bS4_PIN_Push = FALSE;
			}
			
		}
	#endif
		//---------------------------------------
        // Control Status change from the cmd from RTL
    	//---------------------------------------

		while (ControlStatus_change)
		{	
			switch (ControlStatus_Event)
			{
			case PowerOn_Event: 
		        ControlStatus_change = 0;
        			break; 
		    case RTL_PwrOncmd_Event: 
				
				ControlStatus_change = 0;
		        	break; 
			case RTL_HWEnablecmd_Event: 
				initialHWset();			
				bAN2_LowAmplitudeFlag = TRUE;
				ControlStatus_Event = HW_running_Event;
		        ControlStatus_change = 0;
		        	break; 
			case RTL_HWStopcmd_Event: 
				StopRunHW();
				ControlStatus_Event = HW_STOP_run_Event;
		        ControlStatus_change = 0;
		        	break;
			case RTL_cmdError_Event: 	
					for( int i=0; i < SEND_toRTL_CMD_SIZE ;i++)
    					EUSART_Write( PIC_cmd_Error[i] );
				ControlStatus_change = 0;
					break;
		    
			default:
					break;
				
			}
		}

		//---------------------------------------
        // HW_running_Event (Main Event)
    	//---------------------------------------

		if( ControlStatus_Event == HW_running_Event ){
			
			if( bAN0_ADC_ON == TRUE ){
				// sample AN0 per 40ms
				if( Tmr1_4ms_cnt%10 == 1 ){
					ADC1_SelectChannel(HM_ADC_CH_AN0);
					CurrentChan = ADC_CH_AN0_Event;
					ADC1_StartConversion();
					while(!ADC1_IsConversionDone());
					AN0SampValue[an0SampleCount] = ADC1_GetConversionResult();
					ADC1_SelectChannel(HM_ADC_CH_AN2);
					CurrentChan = ADC_CH_AN2_Event;
					
					an0SampleCount++;
					if( an0SampleCount > (AN0_SAMPLE_QUEUE_SIZE-1)){
						AN0SampAvgValue = AN0SampValue[0];
						for (i = 1; i < AN0_SAMPLE_QUEUE_SIZE; ++i)	AN0SampAvgValue = AN0SampAvgValue + AN0SampValue[i];
					
						AN0SampAvgValue = AN0SampAvgValue/AN0_SAMPLE_QUEUE_SIZE;
						an0SampleCount = 0;
				
						if(( bCMD_AUTO_FIND_SET_POINT == TRUE ) && ( bADC_AN0_Delay == FALSE )) 
							FindSetPoint(AN0SampAvgValue);		
						if(bADC_AN0_Delay == TRUE) bADC_AN0_Delay = FALSE; // if PWM value change will delay
					}
				}
				// sample AN2 per 4ms
				//else{
					//AN2_tmpPulseCnt ++;
					if( CurrentChan == ADC_CH_AN2_Event ){
						
						if( bAN0_InRange == TRUE ){
							//-------------
							// ADC AN2 avg
							//-------------
							
							//IO_RC7_SetHigh(); // S4 key
							
							for (i = 0; i < AN2_SAMPLE_AVGBUF_SIZE; ++i)
							{
								ADC1_StartConversion();
								while(!ADC1_IsConversionDone());
								AN2SampAvgBufValue[i] = ADC1_GetConversionResult();	
							}
							AN2_SampAvgBuf_Value = AN2SampAvgBufValue[0];
							for (i = 1; i < AN2_SAMPLE_AVGBUF_SIZE; ++i)
							{
								AN2_SampAvgBuf_Value = AN2_SampAvgBuf_Value + AN2SampAvgBufValue[i] ;	
							}
							
							//IO_RC7_SetLow(); // S4 key
							
							AN2_SampAvgBuf_Value = AN2_SampAvgBuf_Value/AN2_SAMPLE_AVGBUF_SIZE;
							AN2_ChkValue = AN2_SampAvgBuf_Value;
							if ( an2SampleCnt%5 == 0 )
								AN2SampValue[an2SampleCnt/5] = AN2_SampAvgBuf_Value;
							
							if(an2SampleCnt == 0){ 
								AN2MinValue = AN2_ChkValue;
								AN2MaxValue = AN2_ChkValue;
							}

							// 即刻算? 不管振福
							if(( bAN2_GainContInRange == TRUE )||(InRangeStatus == IN_GET_AVG_STATUS))
								Cal_HeartRate(); 
							an2SampleCnt ++;

							//------------------------------------
							// Check AN2 range every sample 
							//------------------------------------
							//
							// 超過 上下限 依舊計算 BPM, and 2sec 後才 上下限 檢查
							//
							if(( bAN2_GainContWaitFlag == KEEP_GO ) &&
								((AN2_ChkValue > V3P3max_VALVE) || (AN2_ChkValue < V0min_VALVE)) ){

								
								if(AN2_GainContInRange_cnt < 2){
									
									AGC_MCP4011_Dir = GO_DOWN;
									MCP_setVal(AGC_MCP4011_Dir);
							#if AGC_MCP4011_DUG_MSG_FUN 
									if(AGC_MCP4011_Gain)
										DugCmdMsg('D',HexNumTable[AGC_MCP4011_Gain/10],HexNumTable[AGC_MCP4011_Gain%10] );
							#endif	
									bAN2_GainContWaitFlag = WAIT;
									bAN2_GainContInRange = FALSE;
									//AN2_HRBufferIndex = 0;
									AN2_GainContInRange_cnt = 0;
									//an2SampleCnt = 0;
								}
								else{
									AN2_GainContInRange_cnt--;
									if (AN2_GainContInRange_cnt < 0) AN2_GainContInRange_cnt = 0;
								}
							
							#if AGC_MCP4011_DUG_MSG_FUN 
								DugDataMsg('o','v','R',AN2_ChkValue) ; 
							#endif
							}
								
							if( AN2MinValue > AN2_ChkValue)		AN2MinValue = AN2_ChkValue;
							if( AN2MaxValue < AN2_ChkValue)		AN2MaxValue = AN2_ChkValue;
							
							
							
							//------------------------------------
							// Check AN2 amplitude every 2sec 
							//------------------------------------

							if( an2SampleCnt > (AN2_SAMPLE_SIZE-1)){
									
								if((AN2MaxValue - AN2MinValue)  < V1P5_VALVE ){ 
									
									AGC_MCP4011_Dir = GO_UP;
									MCP_setVal(AGC_MCP4011_Dir);
								
							#if AGC_MCP4011_DUG_MSG_FUN	
									if(AGC_MCP4011_Gain)
											DugCmdMsg('U',HexNumTable[AGC_MCP4011_Gain/10],HexNumTable[AGC_MCP4011_Gain%10] );
							#endif	
									bAN2_GainContWaitFlag = WAIT;
									AN2_GainContInRange_cnt = 0;
									bAN2_GainContInRange = FALSE;
									//AN2_HRBufferIndex = 0;
							#if AGC_MCP4011_DUG_MSG_FUN 
									DugDataMsg('L','a','R',(AN2MaxValue - AN2MinValue)) ; 
							#endif
								}
								else{
									
									if(AN2_GainContInRange_cnt < 3)	AN2_GainContInRange_cnt++;
									bAN2_GainContInRange = TRUE;
								}

								//
								AN2_AryAvg_value = AN2SampValue[0];
								for (i = 1; i < AN2_ARY_SAMPLE_SIZE; ++i) 
									AN2_AryAvg_value = AN2_AryAvg_value + AN2SampValue[i];
								AN2_AryAvg_value = AN2_AryAvg_value/AN2_ARY_SAMPLE_SIZE;
								AN2_AryAvg_value = ( AN2_AryAvg_value + AN2MinValue )/2;								
								InRangeStatus = IN_GET_AVG_STATUS;
							#if HEART_RATE_DUG_MSG_FUN
								DugDataMsg('m','i','n',AN2_AryAvg_value) ; 
							#endif
								
								an2SampleCnt = 0;
								if( bAN2_GainContWaitFlag == WAIT )	bAN2_GainContWaitFlag = KEEP_GO;
							}
							
						}
					}
				//}
				bAN0_ADC_ON = FALSE;

			}
		
			
			if( bSendToBT_timer_Flag == TRUE){
				HR_BPMvalue_1msCnt = 0;
				if(AN2_READ_PulseIntervalAvg_1msCnt != 0)
					
					// 2019.02.25 Send [inter-beat interval (IBI)]
					//HR_BPMvalue_1msCnt = VAL_1MIN_MS/(AN2_READ_PulseIntervalAvg_1msCnt); // sample rate 4ms
					HR_BPMvalue_1msCnt = AN2_READ_PulseIntervalAvg_1msCnt; // 0xFFFF 資料格式 傳送
				if(bAN0_InRange == TRUE){

					DugHRMsg(HexNumTable[bAN2_GainContInRange],HexNumTable[AGC_MCP4011_Gain/10],HexNumTable[AGC_MCP4011_Gain%10],
						HexNumTable[(HR_BPMvalue_1msCnt & 0xF00)>>8 ],HexNumTable[(HR_BPMvalue_1msCnt & 0xF0 )>>4],HexNumTable[(HR_BPMvalue_1msCnt & 0x0F)] );	
						
					//HexNumTable[3],HexNumTable[5],HexNumTable[9] );	 test OK
				}
				else{ 
					DugHRMsg('2',HexNumTable[AGC_MCP4011_Gain/10],HexNumTable[AGC_MCP4011_Gain%10],'0','0','0' );
				}
				
				bSendToBT_timer_Flag = FALSE;
			}
		}

		
		else if( ControlStatus_Event == HW_STOP_run_Event ){
	#if STOP_DUG_MSG_FUN
			if(Tmr1_4ms_cnt%100 == 0) DugCmdMsg('S','T','P') ; 
	#endif
		}
    }
}
/**
 End of File
*/
