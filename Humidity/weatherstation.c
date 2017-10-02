/**************************************************************************//**
 * @file
 * @brief Weather station demo for EFM32ZG_STK3200 and Sensors-EXP
 * @version 5.0.0
 ******************************************************************************
 * @section License
 * <b>Copyright 2015 Silicon Labs, Inc. http://www.silabs.com</b>
 *******************************************************************************
 *
 * This file is licensed under the Silabs License Agreement. See the file
 * "Silabs_License_Agreement.txt" for details. Before using this software for
 * any purpose, you must agree to the terms of that agreement.
 *
 ******************************************************************************/

#include "em_device.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_gpio.h"
#include "em_usart.h"
#include "i2cspm.h"
#include "si7013.h"
#include "si114x_algorithm.h"
#include "rtcdriver.h"
#include "em_adc.h"
#include "bspconfig.h"
#include "em_leuart.h"
#include "si7021.h"
#include "nrf24l01.h"
#include "lcd.h"
#include "em_rtc.h"
#include "em_rmu.h"

/**************************************************************************//**
 * Local defines
 *****************************************************************************/
/** Time (in ms) to keep looking for gestures if none are seen. */
#define GESTURE_TIMEOUT_MS      60000
/** Time (in ms) between periodic updates of the measurements. */
#define PERIODIC_UPDATE_MS      2000
/** Time (in ms) between scrolling updates. Lower means faster scrolling
 *  between screens. */
#define ANIMATION_UPDATE_MS     50
/** Voltage defined to indicate dead battery. */
#define LOW_BATTERY_THRESHOLD   2800

#define EFM32_HFXO_FREQ				24000000UL
#define RTC_FREQ    32768

/**************************************************************************//**
 * Local variables
 *****************************************************************************/


/** This variable is set to true when the user presses PB0. */
static volatile bool startDemo = false;
/** This variable is set to true when we are in gesture control mode. */
static volatile bool demoMode =  false;
/** This flag tracks if we need to update the display
 *  (animations or measurements). */
static volatile bool updateDisplay = true;
/** This flag tracks if we need to perform a new
 *  measurement. */
static volatile bool updateMeasurement = true;
/** Flag that is used whenever we have get an gesture process interrupt. */
static volatile bool processGestures = false;
/** GUI scrolling offset. */
static volatile int  xoffset = 0;
/** GUI scrolling offset. */
static volatile int  yoffset = 0;
/** Amount to increment xoffset every ANIMATION_UPDATE_MS */
static volatile int  xinc            = 0;
/** Amount to increment yoffset every ANIMATION_UPDATE_MS */
static volatile int  yinc = 0;
/** Millisecond tick counter */
volatile uint32_t msTicks;
/** Used to track what happens when the user presses PB0. */
static int pb0Action = 1;
/** Flag used to indicate ADC is finished */
static volatile bool adcConversionComplete = false;


/** Timer used for timing out gesturemode to save power. */
RTCDRV_TimerID_t gestureTimeoutTimerId;
/** Timer used for counting milliseconds. Used for gesture detection. */
RTCDRV_TimerID_t millisecondsTimerId;
/** Timer used for periodic update of the measurements. */
RTCDRV_TimerID_t periodicUpdateTimerId;
/** Timer used for animations (swiping) */
RTCDRV_TimerID_t animationTimerId;

/**************************************************************************//**
 * Local prototypes
 *****************************************************************************/
static void gpioSetup(void);
void PB0_handler(void);

static uint32_t MeasurePressure (void);
static void adcInit( void );
static void spiInit(void);
static void init_usart(void);
float calc_pressure(uint32_t vdata);
void esp_8266(void);
void rtcSetup(void);
void RTC_IRQHandler(void);
void esp_8266_send(char *cmd,uint8_t say);
void delay_mm(uint8_t delay);
void delay_s(uint8_t sec);
/**************************************************************************//**
 * @brief  Helper function to perform data measurements.
 *****************************************************************************/
static int performMeasurements(uint32_t *rhData, int32_t *tData)
{
  Si7013_MeasureRHAndTemp(I2C0, SI7021_ADDR, rhData, tData);
  return 0;
}

int32_t          tempData;
float volt=0.0,pressure=0.0f;
uint32_t         rhData;
uint32_t temp_rh=0,temp_temp=0;
uint8_t fv;
  bool             si7013_status, si1147_status;

uint8_t stat,rxdata;

uint8_t at[30] = {"Gokhan Arslanbayaa"};
uint8_t at1[50];
uint8_t s=0;

uint32_t fre1=0,fre2=0,fre3=0;
bool data_tamam=0,bekle=0,ok=0;

char data11[16];

uint16_t delay_cntr=0;

uint8_t tempppp=0;
uint8_t desimal=0,ondalik=0;
uint16_t sayici=0;
char gonder[20];
float humudity=0.0f,temperature=0.0;
	uint8_t aa=0;
	
char data111[50];
float temp_press=0;
uint8_t data[500];
uint8_t sayac=0;
uint8_t komut_sayac=0;
uint8_t data_kontrol =0;
bool data_gonder=0;
uint16_t reset_sayac=0;
uint16_t sayici11=0;
uint8_t temp_sayac2=0;
uint8_t zaman_sayac=0;
uint32_t reset=0;

uint8_t reset_sayac1=0;

uint8_t saat=0,dakika=0,saniye=0,gun=0;
uint16_t sayac111=0;


float int_temp=0.0f,int_hum=0.0f,int_pres=0.0f;
/**************************************************************************//**
 * @brief  Main function
 *****************************************************************************/
int main(void)
{
  I2CSPM_Init_TypeDef i2cInit = I2CSPM_INIT_DEFAULT;
  
	uint8_t vBat=0;
	
	char metin[100];
	char komut7[30];
	
	char kapat[]={"AT+CIPCLOSE=0\r\n"};
	char wifi[]={"AT+CWJAP?\r\n"};
  /* Chip errata */
  CHIP_Init();
	CMU_ClockDivSet(cmuClock_HFPER, cmuClkDiv_1);
	
	CMU_OscillatorEnable(cmuOsc_HFRCO, true, true);            // enable XTAL osc and wait for it to stabilize
  CMU_ClockSelectSet(cmuClock_HF, cmuSelect_HFRCO);          // select HF XTAL osc as system clock source (24MHz)
	CMU_HFRCOBandSet(cmuHFRCOBand_21MHz);
	
//* Misc initalizations. */
	CMU_ClockEnable(cmuClock_HFPER, true); 
	rtcSetup();
  gpioSetup();
  adcInit();
	lcd_init();
	lcd_hazirla();
	lcd_yazi_yaz("Humudity");
	lcd_git_xy(2,0);
	lcd_yazi_yaz("Controller v1.0");
	delay_s(100);
	init_usart();
	esp_8266();
	spiInit();
	Config_transmitter();
	
	
	

  /* Misc timers. */
  //RTCDRV_Init();
  //RTCDRV_AllocateTimer(&gestureTimeoutTimerId);
  //RTCDRV_AllocateTimer(&millisecondsTimerId);
  //RTCDRV_AllocateTimer(&periodicUpdateTimerId);
  //RTCDRV_AllocateTimer(&animationTimerId);


  /* Initialize I2C driver, using standard rate. */
  I2CSPM_Init(&i2cInit);
	
	//vBat=SI7021_init();
	
	vBat=Si7013_GetFirmwareRevision(I2C0,SI7021_ADDR,&fv);

  si7013_status = Si7013_Detect(I2C0, SI7021_ADDR, NULL);
	
	  fre1 = CMU_ClockFreqGet(cmuClock_CORE);
    fre2 = CMU_ClockFreqGet(cmuClock_HF);
    fre3 = CMU_ClockFreqGet(cmuClock_HFPER);
/*
		for(s=0;s<sizeof(at);s++)
		{
			LEUART_Tx(LEUART0,at[s]);
		}*/
	//stat=	NRF_Write_reg(0x00,0x18,1);
	//rxdata=NRF_Read_reg(0x00);
		
  while (1)
  {
			tempData=MeasurePressure();
			temp_press+= calc_pressure(tempData);
			performMeasurements(&rhData,&tempData);
			humudity+=rhData/1000.0f;
			temperature+=tempData/1000.0f;
			sayici++;

		
		if(sayici11>=10)
		{
			
			lcd_init();
			
			RTC_IntDisable(RTC_IEN_COMP0);
			GPIO_PinOutToggle(gpioPortA,2);
			//tempData=MeasurePressure();
			//pressure= calc_pressure(volt);
			data_tamam=0;
			sayici11=0;
			
			humudity=humudity/(float)sayici;
			temperature=temperature/(float)sayici;
			pressure=temp_press/(float)sayici;
			sprintf(data11,"RH   = %2.1f RH",humudity);
		
			lcd_git_xy(1,1);
			//sprintf(data11,"RH = %2.1u C",rhData);
			lcd_yazi_yaz(data11);
		
			sprintf(data11,"TEMP = %2.2f C",temperature);
		
			lcd_git_xy(2,1);
			//sprintf(data11,"TEMP = %2.1u C",tempData);
			lcd_yazi_yaz(data11);
			
			//sprintf(data11,"Pressure = %2.1f mBar",pressure);
			
			
			sprintf((char *)at,"%2.2f %2.1f %4.1f ",temperature,humudity,pressure);


			//strcat(komut7,"AT+CIPSEND=0,");
			//sayac=strlen(metin);
			//strcat(komut7,strlen(metin));
			
			//memcpy(at,gonder,sizeof(at));

			Nrf24l01_transmit(at,30);
			
			RTC_IntEnable(RTC_IEN_COMP0);

	
			temperature=0.0f;
			humudity=0.0f;
			temp_press=0.0f;
			
			sayici=0;
			
		}
		
			if(data_gonder==1)
			{
					delay_s(200);
				if(strstr((const char*)data,":GET /reset") != NULL)
					{
			 
							sprintf((char *)metin,"<html><body>Sistem Resetlendi</body></html>");
			
							sprintf(komut7,"AT+CIPSEND=0,%u\r\n",strlen(metin));
					
							esp_8266_send(komut7,strlen(komut7));

							delay_s(200);
							esp_8266_send(metin,strlen(metin));
							NVIC_SystemReset();
					}
					if(strstr((const char*)data,"GET /led1on") != NULL)
					{
			 
							sprintf((char *)metin,"<html><body>Led Yandi.</body></html>");
			
							sprintf(komut7,"AT+CIPSEND=0,%u\r\n",strlen(metin));
					
							esp_8266_send(komut7,strlen(komut7));

							delay_s(200);
							esp_8266_send(metin,strlen(metin));
							GPIO_PinOutSet(gpioPortA,8);
					}
					if(strstr((const char*)data,"GET /led1off") != NULL)
					{
			 
							sprintf(metin,"<html><body>Led Sondu.</body></html>");
			
							sprintf((char *)komut7,"AT+CIPSEND=0,%u\r\n",strlen(metin));
					
							esp_8266_send(komut7,strlen(komut7));

							delay_s(200);
							esp_8266_send(metin,strlen(metin));
							GPIO_PinOutClear(gpioPortA,8);
					}
					if(strstr((const char*)data,"GET /errorreset") != NULL)
					{
			 
							sprintf(metin,"<html><head>Hata Resetlendi</head></html>");
			
							sprintf(komut7,"AT+CIPSEND=0,%u\r\n",strlen(metin));
					
							esp_8266_send(komut7,strlen(komut7));

							delay_s(200);
							esp_8266_send(metin,strlen(metin));
						
							RMU_ResetCauseClear();
					}
					if(strstr((const char*)data,"GET /wifireset") != NULL)
					{
							sprintf(metin,"<html><head>WIFI Resetlendi</head></html>");
			
							sprintf(komut7,"AT+CIPSEND=0,%u\r\n",strlen(metin));
					
							esp_8266_send(komut7,strlen(komut7));

							delay_s(200);
							esp_8266_send(metin,strlen(metin));
							esp_8266();
					}
					
					if(strstr((const char*)data,"GET /result") != NULL)
					{
				
						//temp_sayac2=sayici11+10;
						int_hum=humudity/(float)sayici;
						int_temp=temperature/(float)sayici;
						int_pres=temp_press/(float)sayici;
				
						reset=RMU_ResetCauseGet();
				
						sprintf(at1,(char *)" %02ud%02uh%02um%02us Sicaklik=%2.2f °C Nem=%2.1f RH Basinc=%4.1f mBar reset=0x%04x ",gun,saat,dakika,saniye,int_temp,int_hum,int_pres,reset);
			 
						sprintf(metin,"<html><head><br><textarea cols=\"100\" rows=\"1\">%s</textarea></br></head></html>",at1);
			
						sprintf(komut7,"AT+CIPSEND=0,%u\r\n",strlen(metin));

						memset(data,'\0',sizeof(data));
						ok=0;
						esp_8266_send(komut7,strlen(komut7));


						//memset(data,'\0',sizeof(data));

						//while(temp_sayac2!=sayici11);
						delay_s(200);
						esp_8266_send(metin,strlen(metin));

						//delay_s(200);
						//sprintf(metin,"GET / HTTP/1.1\r\n Host: 192.168.2.150\r\n Connection: close\r\n\r\n");
						//esp_8266_send(metin,strlen(metin));
						//data_kontrol=0;
						//while(!data_kontrol);
						//data_kontrol=0;
						//esp_8266_send(kapat,strlen(kapat));
					}
					
					memset(data,'\0',sizeof(data));
					data_gonder=0;
					sayac111=0;
					esp_8266_send(kapat,strlen(kapat));
				}
			
		
				EMU_EnterEM2(true);
  }
}

/**************************************************************************//**
 * @brief This function is called whenever we want to measure the supply v.
 *        It is reponsible for starting the ADC and reading the result.
 *****************************************************************************/
static uint32_t MeasurePressure (void)
{
  uint32_t vData;
  /* Sample ADC */
  adcConversionComplete = false;
  ADC_Start(ADC0, adcStartSingle);
  while (!adcConversionComplete) EMU_EnterEM1();
  vData = ADC_DataSingleGet( ADC0 );
  return vData;
}

/**************************************************************************//**
 * @brief ADC Interrupt handler (ADC0)
 *****************************************************************************/
void ADC0_IRQHandler( void ) {

   uint32_t flags;

   /* Clear interrupt flags */
   flags = ADC_IntGet( ADC0 );
   ADC_IntClear( ADC0, flags );

   adcConversionComplete = true;

}

/**************************************************************************//**
 * @brief ADC Initialization
 *****************************************************************************/
static void adcInit( void ) {

   ADC_Init_TypeDef       init       = ADC_INIT_DEFAULT;
   ADC_InitSingle_TypeDef initSingle = ADC_INITSINGLE_DEFAULT;

   /* Enable ADC clock */
   CMU_ClockEnable( cmuClock_ADC0, true );

   /* Initiate ADC peripheral */
   ADC_Init(ADC0, &init);

   /* Setup single conversions for internal VDD/3 */
   initSingle.acqTime = adcAcqTime16;
   initSingle.input   = adcSingleInpCh6;
	 initSingle.reference=adcRef2V5;
   ADC_InitSingle( ADC0, &initSingle );

   /* Manually set some calibration values */
   ADC0->CAL = (0x7C << _ADC_CAL_SINGLEOFFSET_SHIFT) | (0x1F << _ADC_CAL_SINGLEGAIN_SHIFT);

   /* Enable interrupt on completed conversion */
   ADC_IntEnable(ADC0, ADC_IEN_SINGLE);
   NVIC_ClearPendingIRQ( ADC0_IRQn );
   NVIC_EnableIRQ( ADC0_IRQn );
}


/**************************************************************************//**
 * @brief This function is called whenever a new gesture needs to be processed.
 *        It is reponsible for setting up the animations.
 *****************************************************************************/


/**************************************************************************//**
 * @brief Enable gesture mode.
 *****************************************************************************/

/**************************************************************************//**
 * @brief Disable gesture mode.
 * @param id
 *   Timer ID that triggered this event. Not used, only there for
 *   compatability with RTC driver.
 *****************************************************************************/


/**************************************************************************//**
* @brief Setup GPIO interrupt for pushbuttons.
*****************************************************************************/
static void gpioSetup(void)
{
  /* Enable GPIO clock */
  CMU_ClockEnable(cmuClock_GPIO, true);

  /* Configure PB0 as input and enable interrupt  */
  //GPIO_PinModeSet(BSP_GPIO_PB0_PORT, BSP_GPIO_PB0_PIN, gpioModeInputPull, 1);
  //GPIO_IntConfig(BSP_GPIO_PB0_PORT, BSP_GPIO_PB0_PIN, false, true, true);

  /* Configure PB1 as input and enable interrupt */
  //GPIO_PinModeSet(BSP_GPIO_PB1_PORT, BSP_GPIO_PB1_PIN, gpioModeInputPull, 1);
  //GPIO_IntConfig(BSP_GPIO_PB1_PORT, BSP_GPIO_PB1_PIN, false, true, true);

  //NVIC_ClearPendingIRQ(GPIO_EVEN_IRQn);
  //NVIC_EnableIRQ(GPIO_EVEN_IRQn);

  //NVIC_ClearPendingIRQ(GPIO_ODD_IRQn);
  //NVIC_EnableIRQ(GPIO_ODD_IRQn);

  /* Configure PD4 as pushpull. (5v enable signal) */
  GPIO_PinModeSet(gpioPortD, 4, gpioModePushPull, 0);
  GPIO_PinOutClear(gpioPortD, 4);
	GPIO_PinModeSet(gpioPortA, 2, gpioModePushPull, 1);
  GPIO_PinOutClear(gpioPortA, 2);
	GPIO_PinModeSet(gpioPortA, 8, gpioModePushPull, 1);
  GPIO_PinOutClear(gpioPortA, 8);
	GPIO_PinModeSet(gpioPortB, 11, gpioModePushPull, 1);
  GPIO_PinOutClear(gpioPortB, 11);

  /* Configure PD5 as input and enable interrupt - proximity interrupt. */
  /* Interrupt is active low */
  GPIO_PinModeSet(gpioPortD, 5, gpioModeInputPull, 1);
  GPIO_IntConfig(gpioPortD, 5, false, true, true);

  /*Enable 5V supply to add-on board. */
  GPIO_PinOutClear(gpioPortA, 2);
	GPIO_PinOutClear(gpioPortA, 8);
}

/**************************************************************************//**
* @brief Unified GPIO Interrupt handler (pushbuttons)
*        PB0 Switches units within a measurement display
*        PB1 Starts the demo (quit splashscreen)
*****************************************************************************/
void GPIO_Unified_IRQ(void)
{
  /* Get and clear all pending GPIO interrupts */
  uint32_t interruptMask = GPIO_IntGet();
  GPIO_IntClear(interruptMask);

  /* Act on interrupts */
  if (interruptMask & (1 << BSP_GPIO_PB0_PIN))
  {
    /* PB0: Switch units within display*/
    PB0_handler();
  }

  if (interruptMask & (1 << BSP_GPIO_PB1_PIN))
  {
    /* PB1: Start the demo */
    startDemo = true;
  }

  if (interruptMask & (1 << 5))
  {
    /* Interrupt from Si1147 on PD5 */
    processGestures = true;
  }
}

/**************************************************************************//**
* @brief GPIO Interrupt handler for even pins
*****************************************************************************/
void GPIO_EVEN_IRQHandler(void)
{
  GPIO_Unified_IRQ();
}

/**************************************************************************//**
* @brief GPIO Interrupt handler for odd pins
*****************************************************************************/
void GPIO_ODD_IRQHandler(void)
{
  GPIO_Unified_IRQ();
}

/**************************************************************************//**
 * @brief PB0 Interrupt handler
 *****************************************************************************/
void PB0_handler(void)
{
  if (xoffset == 0)
  {
    pb0Action = 1;
  }
  if (xoffset == 32)
  {
    pb0Action = -1;
  }
  updateDisplay = true;
  /* This timer runs the animations. E.g if an animation is
   * active this will retrigger a redraw. */
  //RTCDRV_StartTimer(animationTimerId, rtcdrvTimerTypePeriodic,ANIMATION_UPDATE_MS, animationUpdateCallback, NULL);
  xinc          = pb0Action;
  startDemo     = true;
}

/**************************************************************************//**
 * @brief   The actual callback for Memory LCD toggling
 * @param[in] id
 *   The id of the RTC timer (not used)
 *****************************************************************************/


/**************************************************************************//**
 * @brief   Register a callback function at the given frequency.
 *
 * @param[in] pFunction  Pointer to function that should be called at the
 *                       given frequency.
 * @param[in] argument   Argument to be given to the function.
 * @param[in] frequency  Frequency at which to call function at.
 *
 * @return  0 for successful or
 *         -1 if the requested frequency does not match the RTC frequency.
 *****************************************************************************/


/**************************************************************************//**
 * @brief Callback used to count between measurement updates
 *****************************************************************************/



static void spiInit(void)
{
	
		CMU_ClockEnable(cmuClock_USART1, true);

		USART_TypeDef *spi_type = USART1;

		USART_InitSync_TypeDef spi_setup = USART_INITSYNC_DEFAULT;
	
		spi_setup.baudrate=100000;
		spi_setup.msbf =true;

		USART_InitSync(spi_type, &spi_setup);

		GPIO_PinModeSet(gpioPortC, 0, 	gpioModePushPull, 	0);	 //mosi
		GPIO_PinModeSet(gpioPortC, 1, 	gpioModeInputPull, 	1);	 //miso
		GPIO_PinModeSet(gpioPortB, 7, 	gpioModePushPull, 	0);	 //clk
		GPIO_PinModeSet(gpioPortC, 2, 	gpioModePushPull, 	1);  //cs
		GPIO_PinModeSet(gpioPortC, 3, 	gpioModePushPull, 	0);  //ce
		GPIO_PinModeSet(gpioPortC, 4, 	gpioModeInputPull, 	0);  //irq

		spi_type->ROUTE = USART_ROUTE_LOCATION_LOC0 | USART_ROUTE_TXPEN | USART_ROUTE_RXPEN | USART_ROUTE_CLKPEN;
		spi_type->CMD = USART_CMD_MASTEREN | USART_CMD_TXEN | USART_CMD_RXEN;



		GPIO_PinOutSet(gpioPortC,2);
		GPIO_PinOutClear(gpioPortC,3);
		
		USART_IntClear(spi_type, USART_IF_RXDATAV);
		USART_IntEnable(USART1,USART_IF_RXDATAV);
		NVIC_ClearPendingIRQ(USART1_RX_IRQn);
		NVIC_DisableIRQ(USART1_RX_IRQn);

}//

bool data_geldi=0;

void USART1_RX_IRQHandler (void)
{
	USART_TypeDef *spi = USART1;
  uint8_t       rxdata;
	
	
	USART_IntClear(USART1,USART_IF_RXDATAV);
	data_geldi=1;
	desimal=spi->RXDATA;
}

static void init_usart(void)
{
	
		CMU_ClockEnable(cmuClock_HFPER, true);
		CMU_ClockEnable(cmuClock_GPIO, true);
	
		GPIO_PinModeSet(gpioPortD,4,gpioModePushPull,1);
		GPIO_PinModeSet(gpioPortD,5,gpioModeInput,0);
	
		LEUART_TypeDef      *leuart = LEUART0;
		LEUART_Init_TypeDef init    = LEUART_INIT_DEFAULT;
	
		CMU_ClockSelectSet(cmuClock_LFB, cmuSelect_LFRCO);
		CMU_ClockSelectSet(cmuClock_CORELE,cmuSelect_LFRCO);
	  CMU_ClockEnable(cmuClock_LFB, true);
		CMU_ClockEnable(cmuClock_CORELE, true);
	
		//CMU_ClockEnable(cmuClock_CORELE, true);
		//CMU_ClockSelectSet(cmuClock_LFB, cmuSelect_LFXO);
		CMU_ClockEnable(cmuClock_LEUART0, true);
		//CMU_ClockDivSet(cmuClock_LEUART0, cmuClkDiv_2);
	
		//init.enable=leuartDisable;
		//init.baudrate=9600;
		//init.refFreq=CMU_ClockFreqGet(cmuClock_LEUART0);
		//init.refFreq=cmuSelect_CORELEDIV2;
		LEUART_Init(LEUART0,&init);
	
		leuart->ROUTE = LEUART_ROUTE_RXPEN | LEUART_ROUTE_TXPEN | LEUART_ROUTE_LOCATION_LOC0;		
		LEUART_IntClear(LEUART0,LEUART_IF_RXDATAV);
		NVIC_ClearPendingIRQ(LEUART0_IRQn);
		
		LEUART_IntEnable(LEUART0,LEUART_IF_RXDATAV);
		NVIC_EnableIRQ(LEUART0_IRQn);
		LEUART_Enable(LEUART0, leuartEnable);
		
}


void LEUART0_IRQHandler(void)
{
	
	
	LEUART_IntClear(LEUART0,LEUART_IF_RXDATAV);
	data[sayac111]= LEUART_RxDataGet(LEUART0);
	if(data[sayac111]==10&& data[sayac111-1]==13&& data[sayac111-2]==75&& data[sayac111-3]==79)
	{
		data_kontrol=1;	
		sayac111=0;
		//memset(data,'\0',sizeof(data));
	}
	if(data[sayac111]=='N'&& data[sayac111-1]=='N'&& data[sayac111-2]=='O'&& data[sayac111-3]=='C')
	{
		data_gonder=1;
		//sayac111=0;
	}
	if(data[sayac111]==0x20 && data[sayac111-1]=='>')
	{
		ok=1;
		//memset(data,'\0',sizeof(data));
		//sayac111=0;
	}
	
	sayac111++;
}

float calc_pressure(uint32_t vdata)
{
	float pres=0.0f;
	
	volt = (2.5f/4096.0f)*(tempData*2);
	
	//volt= volt - 0.204f;
	
	pres=((volt/5.0f)+0.095f)/0.009f;

	return pres*10.0;
}

void esp_8266(void)
{
	
	char komut1[]={"AT\r\n"};
	char komut2[]={"AT+CWMODE?\r\n"};
	char komut3[]={"AT+CWMODE=1\r\n"};
	char komut4[]={"AT+CWJAP=\"GArslanbay\",\"OGE2269195\"\r\n"};
	char komut5[]={"AT+CIPMUX=1\r\n"};
	char komut6[]={"AT+CIPSERVER=1,80\r\n"};
	char komut7[]={"AT+CIPSTA=\"192.168.2.150\"\r\n"};
	
	char komut9[]={"ATE0\r\n"};
	char komut10[]={"AT+CIPSTO=10\r\n"};
	
	
	
	char komut8[]={"AT+UART=9600,8,1,0,0\r\n"};
	//char komut7[20];
	//sprintf(komut7,"AT+CIPSEND=0,%2d\r\n",sizeof(gonder));
	
	uint8_t dene=0;
	


	
		memset(data,'\0',sizeof(data));
		sayac111=0;
		esp_8266_send(komut9,sizeof(komut9));
		delay_s(200);
		if (strstr((char *)data,"OK") != NULL)
		{
			sayac111=0;
			memset(data,'\0',sizeof(data));
			//return;
		}
		

	
tekrar:	
	
		memset(data,'\0',sizeof(data));
		sayac111=0;
		esp_8266_send(komut1,sizeof(komut1));
		delay_s(200);
		if (strstr((char *)data,"OK") != NULL)
		{
			sayac111=0;
			memset(data,'\0',sizeof(data));
			lcd_temizle();
			lcd_git_xy(1,1);
			lcd_yazi_yaz("Wifi Modul  :");
			lcd_git_xy(2,1);
			lcd_yazi_yaz("Bulundu..");
			//delay_s(200);
			//return;
		}
		else
		{
			lcd_komut(0x01);
			lcd_yazi_yaz("Wifi Modul  :");
			lcd_git_xy(2,1);
			lcd_yazi_yaz("Bulunmadi..");
			delay_s(200);
			if(++dene<=3)
			{
				lcd_komut(0x01);
				lcd_yazi_yaz("Wifi Modul  :");
				lcd_git_xy(2,1);
				lcd_yazi_yaz("Tekrar Deniyor..");
				delay_s(200);
				goto tekrar;
			}
			else
			{
				lcd_komut(0x01);
				lcd_yazi_yaz("Wifi Modul  :");
				lcd_git_xy(2,1);
				lcd_yazi_yaz("Modul Yok..");
				delay_s(200);
				goto modul_yok;
			}
		}
		
		//end if
		
	
		data_kontrol=0;
		esp_8266_send(komut9,sizeof(komut9));
		delay_s(200);
		if (strstr((char *)data,"OK") != NULL)
		{
			sayac111=0;
			memset(data,'\0',sizeof(data));
			//return;
		}
	//while(!data_kontrol);
	//data_kontrol=0;
		
		
		esp_8266_send(komut3,sizeof(komut3));
		delay_s(200);
		if (strstr((char *)data,"OK") != NULL)
		{
			sayac111=0;
			memset(data,'\0',sizeof(data));
			lcd_komut(0x01);
			lcd_yazi_yaz("Wifi Modul  :");
			lcd_git_xy(2,1);
			lcd_yazi_yaz("ist. Ayarlandi.");
			delay_s(200);
			//return;
		}
	//while(!data_kontrol);
	//data_kontrol=0;
	//lcd_komut(0x01);
		
		esp_8266_send(komut7,sizeof(komut7));
		delay_s(200);
		if (strstr((char *)data,"OK") != NULL)
		{
			sayac111=0;
			memset(data,'\0',sizeof(data));
			lcd_temizle();
			lcd_yazi_yaz((char *)("Wifi Modul  :"));
			lcd_git_xy(2,1);
			lcd_yazi_yaz("IP:192.168.2.150");
			delay_s(200);
			//return;
		}

	
	//memset(data,'\0',sizeof(data));
		esp_8266_send(komut4,sizeof(komut4));
		delay_s(200);
		delay_s(200);
		delay_s(200);
		delay_s(200);
		delay_s(200);
		delay_s(200);
		if (strstr((char *)data,"OK") != NULL)
		{
			sayac111=0;
			memset(data,'\0',sizeof(data));
			lcd_komut(0x01);
			lcd_yazi_yaz("Wifi Modul  :");
			lcd_git_xy(2,1);
			lcd_yazi_yaz("Baglandi.");
			delay_s(200);
			//return;
		}
	//while(!data_kontrol);
	//data_kontrol=0;
	
	
	//delay_s(200);
	
	//memset(data,'\0',sizeof(data));

	//while(!data_kontrol);
	//data_kontrol=0;

	
	//memset(data,'\0',sizeof(data));
		esp_8266_send(komut5,sizeof(komut5));
		delay_s(200);
		if (strstr((char *)data,"OK") != NULL)
		{
			sayac111=0;
			memset(data,'\0',sizeof(data));
			//return;
		}
	
		esp_8266_send(komut6,sizeof(komut6));
		delay_s(200);
		if (strstr((char *)data,"OK") != NULL)
		{
			sayac111=0;
			memset(data,'\0',sizeof(data));
			//return;
		}	
		
		esp_8266_send(komut10,sizeof(komut10));
		delay_s(200);
		if (strstr((char *)data,"OK") != NULL)
		{
			sayac111=0;
			memset(data,'\0',sizeof(data));
			//return;
		}	
		
		lcd_temizle();
	
	modul_yok:
}

void esp_8266_send(char *cmd,uint8_t say)
{

	for(s=0;s<say;s++)
	{
		LEUART_Tx(LEUART0,*cmd);
		cmd++;
	}

}

void RTC_IRQHandler(void)
{
  /* Clear interrupt source */
  RTC_IntClear(RTC_IFC_COMP0);
	bekle=1;
	sayici11++;
	delay_cntr++;
	
	if(++zaman_sayac==10)
	{
		zaman_sayac=0;
		if(++saniye==60)
		{
			saniye=0;
			if(++dakika==60)
			{
				dakika=0;
				if(++saat==24)
				{
					saat=0;
					gun++;
				}
				
			}
		}
	}

}
void rtcSetup(void)
{
  RTC_Init_TypeDef rtcInit = RTC_INIT_DEFAULT;

  /* Enable LE domain registers */
	
	CMU_OscillatorEnable(cmuOsc_LFRCO, true, true);
	
  CMU_ClockEnable(cmuClock_CORELE, true);

  /* Enable LFXO as LFACLK in CMU. This will also start LFXO */
  CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFRCO);

  /* Set a clock divisor of 32 to reduce power conumption. */
  CMU_ClockDivSet(cmuClock_RTC, cmuClkDiv_1);

  /* Enable RTC clock */
  CMU_ClockEnable(cmuClock_RTC, true);

  /* Initialize RTC */
  rtcInit.enable   = false;  /* Do not start RTC after initialization is complete. */
  rtcInit.debugRun = false;  /* Halt RTC when debugging. */
  rtcInit.comp0Top = true;   /* Wrap around on COMP0 match. */
  

  /* Interrupt every minute */
  RTC_CompareSet(0,(RTC_FREQ / 10)- 2 );

  /* Enable interrupt */
  NVIC_EnableIRQ(RTC_IRQn);
  RTC_IntEnable(RTC_IEN_COMP0);
	
	RTC_Init(&rtcInit);

  /* Start Counter */
  RTC_Enable(true);
}

void delay_mm(uint8_t delay)
{
	uint8_t temp_delay;
	
	for(temp_delay=0;temp_delay < delay ; temp_delay++)
	{
		__NOP;
	}

}

void delay_s(uint8_t sec)
{

	delay_cntr=0;
	while(delay_cntr<sec/10);

}
uint8_t get_compare(char *msj, char *cmp_msj)
{
	

}