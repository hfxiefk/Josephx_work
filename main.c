/* Copyright (c) 2014 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

/** @file
 *
 * @defgroup ble_sdk_uart_over_ble_main main.c
 * @{
 * @ingroup  ble_sdk_app_nus_eval
 * @brief    UART over BLE application main file.
 *
 * This file contains the source code for a sample application that uses the Nordic UART service.
 * This application uses the @ref srvlib_conn_params module.
 */

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "app_timer.h"
#include "nrf.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "softdevice_handler.h"
#include "app_timer.h"
#include "app_button.h"
#include "ble_nus.h"
#include "app_uart.h"
#include "app_util_platform.h"
#include "bsp.h"
#include "bsp_btn_ble.h"
#include "nrf_delay.h"
#include "pstorage.h"
#include "rn_8209.h"
#include "yhcom.h"

//		EMU_RN8209    emu_regs;
char	StartNetCheck,//1Min启动检测通讯链路标识
			RxBuf[50],//AT指令接收寄存器
			at_cnt;//寄存器当前数据序号

uint8_t rxdTemp = 0;
uint8_t txdTemp = 0;

#define LED1_PIN	24
#define LED2_PIN	21
#define RST_PIN		29
#define EN_PIN		28
#define RX_PIN		1
#define TX_PIN		0
// #define RX_PIN 	11
// #define TX_PIN 	9
#define CTS_PIN		22
#define RTS_PIN		23

uint8_t AT_BUFFER_AT[] = "AT\r\n";
uint8_t AT_BUFFER_CWMODE[] = "AT+CWMODE=3\r\n";
uint8_t AT_BUFFER_CWJAP[] = "AT+CWJAP=\"Tenda_5609A0\",\"yf830812\"\r\n";
uint8_t AT_BUFFER_CIPSTART[] = "AT+CIPSTART=\"TCP\",\"192.168.0.104\",3000\r\n";
//uint8_t AT_BUFFER_CWJAP[] = "AT+CWJAP=\"JIN_plus\",\"294h-pugb-pdtp\"\r\n";
//uint8_t AT_BUFFER_CWJAP[] = "AT+CWJAP=\"YUNHAN\",\"5578900000\"\r\n";
//uint8_t AT_BUFFER_CIPSTART[] = "AT+CIPSTART=\"TCP\",\"192.168.1.10\",1000\r\n";
//uint8_t AT_BUFFER_CIPSTART[] = "AT+CIPSTART=\"TCP\",\"112.27.206.97\",1000\r\n";
// uint8_t AT_BUFFER_CIPSTART[] = "AT+CIPSTART=\"UDP\",\"192.168.1.107\",8080,8080,0\r\n";
uint8_t AT_BUFFER_CIPMODE[] = "AT+CIPMODE=1\r\n";
uint8_t AT_BUFFER_CIPSEND[] = "AT+CIPSEND\r\n";
uint8_t AT_BUFFER_RANDOM[] = "ID=XXX:efafewafaefwae\r\n";
uint8_t	yunhan[] = "YH_Welcome_YOU!";

#define MY_ONE_SECTIMER_INTERVAL      APP_TIMER_TICKS(1000, APP_TIMER_PRESCALER) 

/**用户申请TIMER的时间间隔(ticks)，单位ms. */

#define IS_SRVC_CHANGED_CHARACT_PRESENT 0                                           /**< Include the service_changed characteristic. If not enabled, the server's database cannot be changed for the lifetime of the device. */

#define CENTRAL_LINK_COUNT              0                                           /**< Number of central links used by the application. When changing this number remember to adjust the RAM settings*/
#define PERIPHERAL_LINK_COUNT           1                                           /**< Number of peripheral links used by the application. When changing this number remember to adjust the RAM settings*/

// #define DEVICE_NAME                     "Nordic_UART"                               /**< Name of device. Will be included in the advertising data. */
#define DEVICE_NAME                     "YH_BLE+WIFI"                               /**< Name of device. Will be included in the advertising data. */
#define NUS_SERVICE_UUID_TYPE           BLE_UUID_TYPE_VENDOR_BEGIN                  /**< UUID type for the Nordic UART Service (vendor specific). */

#define APP_ADV_INTERVAL                64                                          /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */
//MODIFY BY JIE AT 161015
//#define APP_ADV_TIMEOUT_IN_SECONDS      180   
#define APP_ADV_TIMEOUT_IN_SECONDS      1   

#define APP_TIMER_PRESCALER             0                                           /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_OP_QUEUE_SIZE         4                                           /**< Size of timer operation queues. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(20, UNIT_1_25_MS)             /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(75, UNIT_1_25_MS)             /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER)  /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000, APP_TIMER_PRESCALER) /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

// #define UART_TX_BUF_SIZE                256                                         /**< UART TX buffer size. */
// #define UART_RX_BUF_SIZE                256                                         /**< UART RX buffer size. */

static ble_nus_t                        m_nus;                                   /**< Structure to identify the Nordic UART Service. */
static uint16_t                         m_conn_handle = BLE_CONN_HANDLE_INVALID;    /**< Handle of the current connection. */

static ble_uuid_t                       m_adv_uuids[] = {{BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE}};  /**< Universally unique service identifier. */

uint8_t teststring[] ="afekhgurlq\r\n";
uint8_t recvstring[100];
uint8_t flag = 0;
APP_TIMER_DEF(myOneSecTimerId);                                                  /**注册用户申请TIMER的ID号 */
//10_04计量开始
struct
{
  uint8_t		ChkErrCnt;
  uint32_t 	Pw[2];   		    //pa,pb   
  uint32_t 	I[2];          // Ia=UI[0] Inal U 
	uint32_t 	U; 
  uint16_t 	Frequency;   		//????,??:                            	
  uint32_t		Pulse;		    	//????
  uint16_t     Pstart;
  //---????---	
  uint32_t		Pulse_Eg;	    	//????
  uint32_t 	PDirect;				//????
  uint32_t 	ChkSum1;				//??EMU???????
  // ??????
  uint16_t		RatintU;				// ????
	uint16_t		RatingI;				// ????
  uint32_t		TempU;					// ???????
  uint32_t		TempI;					// ???????
  uint32_t		TempPw;					// ???????
} EMU_YH;
//SPI
sDl645FirmParaFile_TypeDef	Dl645FirmPara;

//10_04计量结束

void Uart_Init(void)
{
	NRF_UART0 -> ENABLE = (UART_ENABLE_ENABLE_Enabled << UART_ENABLE_ENABLE_Pos);
	NRF_UART0 -> BAUDRATE = (UART_BAUDRATE_BAUDRATE_Baud115200 << UART_BAUDRATE_BAUDRATE_Pos);
	// NRF_UART0 -> CONFIG = (UART_CONFIG_HWFC_Enabled << UART_CONFIG_HWFC_Pos);
	
	nrf_gpio_cfg_output(TX_PIN);							//TXD
	nrf_gpio_cfg_input(RX_PIN,NRF_GPIO_PIN_NOPULL);			//RXD
	// nrf_gpio_cfg_output(0x0A);							//CTS
	// nrf_gpio_cfg_input(0x08,NRF_GPIO_PIN_NOPULL);		//RTS
	
	NRF_UART0 -> PSELTXD = TX_PIN;
	NRF_UART0 -> PSELRXD = RX_PIN;
	// NRF_UART0 -> PSELCTS = 0x0A;
	// NRF_UART0 -> PSELRTS = 0x08;
	NRF_UART0 -> TASKS_STARTTX = 1;
	NRF_UART0 -> TASKS_STARTRX = 1;
	NRF_UART0 -> EVENTS_RXDRDY = 0;
	
	NRF_UART0 -> INTENSET = UART_INTENSET_RXDRDY_Msk;
	NVIC_SetPriority(UART0_IRQn,4);
	NVIC_EnableIRQ(UART0_IRQn);
}

/**
 * @brief Function for application main entry.
 */

/*void UART0_SEND_CHAR(uint8_t temp)
{
	NRF_UART0 -> TXD = temp;
	while( NRF_UART0 -> EVENTS_TXDRDY != 1 )
		;
	NRF_UART0 -> EVENTS_TXDRDY = 0;		
}
 
void UART0_SEND_STRING(uint8_t *ptr)
{
	while (*ptr)
	{
		UART0_SEND_CHAR(*ptr++);
	}	
}
*/ 

//主函数前申明用户定时器中断程序处理入口
static void my_OneSec_timeout_handler(void * p_context)
{
	nrf_gpio_pin_toggle(LED1_PIN);
	nrf_gpio_pin_toggle(LED2_PIN);
	sys_time.sec ++;
	if(sys_time.sec>=60)
	{
			sys_time.sec =0;
			sys_time.min ++;
			StartNetCheck = 0x01;
			if(sys_time.min >=60)
			{
				sys_time.min =0;
				sys_time.hour ++;
				if(sys_time.hour >=24)
				{
					sys_time.hour = 0;
					sys_time.day ++;
					if(sys_time.mon == 2)
					{
							if((sys_time.year%4) != 0)
							{	
								if(sys_time.day >=29)
								{
									sys_time.day = 1 ;
									sys_time.mon ++;
								}
							}
							else{
								if(sys_time.day >=30)
								{
									sys_time.day = 1 ;
									sys_time.mon ++;
								}
							}
					}
					else if((sys_time.mon == 4)||(sys_time.mon == 6)||(sys_time.mon == 9)||(sys_time.mon == 11))
					{
						if(sys_time.day >=31)
						{
							sys_time.day = 1 ;
							sys_time.mon ++;
						}
					}
					else {
						if(sys_time.day >=32)
						{
							sys_time.day = 1 ;
							sys_time.mon ++;
						}
					}
				}
				if(sys_time.mon >= 13)
				{
					sys_time.mon = 1;
					sys_time.year++;
				}
			}
	}
	
	
	
}

//
void IP_Init(void)
{
	UART0_SEND_STRING(AT_BUFFER_CIPSTART);
	nrf_delay_ms(1500);
	
	UART0_SEND_STRING(AT_BUFFER_CIPMODE);
	nrf_delay_ms(1500);
	
	UART0_SEND_STRING(AT_BUFFER_CIPSEND);
	nrf_delay_ms(1500);
}
//
void Timer_Init(void)
{
	uint32_t err_code;

    // Initialize.
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);
    err_code = app_timer_create(&myOneSecTimerId,
                                APP_TIMER_MODE_REPEATED,
                                my_OneSec_timeout_handler);
    APP_ERROR_CHECK(err_code);
		//
	  err_code = app_timer_start(myOneSecTimerId, MY_ONE_SECTIMER_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);
}
//
void WIFI_Init(void)
{
    // WIFI模块管脚配置
    nrf_gpio_cfg_output(RST_PIN);
    nrf_gpio_pin_set(RST_PIN);

    nrf_gpio_cfg_output(EN_PIN);
    nrf_gpio_pin_set(EN_PIN);
	
		nrf_gpio_cfg_output(RTS_PIN);
    nrf_gpio_pin_clear(RTS_PIN);
	
	
	//CTS管脚配置，可以不做任何操作
	// NRF_GPIO-> PIN_CNF[CTS_PIN] = 
		// (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos);	
		
	//WiFi模块操作
	//对WIFI模块进行一次复位操作，非必须。WIFI上电后会回传较多字符，需等待较长时间！！！！
   nrf_gpio_pin_set(RST_PIN);
	nrf_delay_ms(500);	
    nrf_gpio_pin_clear(RST_PIN);	
	nrf_delay_ms(1000);	
    nrf_gpio_pin_set(RST_PIN);

	nrf_delay_ms(10000);	
	
	UART0_SEND_STRING(AT_BUFFER_AT);
	nrf_delay_ms(1000);
	
	UART0_SEND_STRING(AT_BUFFER_AT);
	nrf_delay_ms(1000);
	
	UART0_SEND_STRING(AT_BUFFER_CWMODE);
	nrf_delay_ms(1000);
	//	
	UART0_SEND_STRING(AT_BUFFER_CWJAP);
	nrf_delay_ms(10000);//此处需等待至少5s时间！！！！
	//	
	IP_Init();
} 
//


char* str1;
char CIPSTATUS_Flag=0;//CIPSTATUS_Rxd_Flag=0;

void check_NetStatus(void )
{
	UART0_SEND_STRING("+++\r\n");
	nrf_delay_ms(1200);	
	UART0_SEND_STRING("AT+CIPSTATUS\r\n");
	CIPSTATUS_Flag=1;
	at_cnt = 0;
}

void NetReLinking(void )
{
	//if((CIPSTATUS_Flag==1)&&(CIPSTATUS_Rxd_Flag))
	if((CIPSTATUS_Flag==1)&&(at_cnt>7))
	{
		str1=strstr(RxBuf,"STATUS:");
		if(str1)
		{
			//CIPSTATUS_Flag=0;
			//CIPSTATUS_Rxd_Flag=0;
			//str1=strstr(RxBuf,"STATUS:");
			CIPSTATUS_Flag=*(str1+7)-'0';//STATUS:<stat>  stat=2 IP，stat=3已连接,stat=4断开连接，stat=5未连接到wifi。
			memset(RxBuf,0,sizeof(RxBuf));
			at_cnt = 0;
			if(CIPSTATUS_Flag==4)
			{
				IP_Init();//重新登录：IP地址
				//telent;
				fnYHcom_Init();//Í¨Ñ¶³õÊ¼»¯
				fnYHcom_Telent();//ÖÕ¶ËµÇÂ¼Í¨Ñ¶Ö¡
				ComBox.EFlag = COMPK_LINK;
			}
			if(CIPSTATUS_Flag==5)
			{
				WIFI_Init();//重新登录：从wifi开始
				//telent;
				fnYHcom_Init();//Í¨Ñ¶³õÊ¼»¯
				fnYHcom_Telent();//ÖÕ¶ËµÇÂ¼Í¨Ñ¶Ö¡
				ComBox.EFlag = COMPK_LINK;
			}
			CIPSTATUS_Flag=0;
		}
		else{
			if(at_cnt>=50)
			{;}
		}
	}
}


/**@brief Function for assert macro callback.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyse 
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num    Line number of the failing ASSERT call.
 * @param[in] p_file_name File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief Function for the GAP initialization.
 *
 * @details This function will set up all the necessary GAP (Generic Access Profile) parameters of 
 *          the device. It also sets the permissions and appearance.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);
    
    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *) DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the data from the Nordic UART Service.
 *
 * @details This function will process the data received from the Nordic UART BLE Service and send
 *          it to the UART module.
 *
 * @param[in] p_nus    Nordic UART Service structure.
 * @param[in] p_data   Data to be send to UART module.
 * @param[in] length   Length of the data.
 */
/**@snippet [Handling the data received over BLE] */
static void nus_data_handler(ble_nus_t * p_nus, uint8_t * p_data, uint16_t length)
{
    // for (uint32_t i = 0; i < length; i++)
    // {
        // while(app_uart_put(p_data[i]) != NRF_SUCCESS);
    // }
    // while(app_uart_put('\n') != NRF_SUCCESS);
	
	strcpy((char *)recvstring,(const char *)p_data);
	recvstring[length] = 0;
	flag = 1;
	
}
/**@snippet [Handling the data received over BLE] */


/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    uint32_t       err_code;
    ble_nus_init_t nus_init;
    
    memset(&nus_init, 0, sizeof(nus_init));

    nus_init.data_handler = nus_data_handler;
    
    err_code = ble_nus_init(&m_nus, &nus_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling an event from the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module
 *          which are passed to the application.
 *
 * @note All this function does is to disconnect. This could have been done by simply setting
 *       the disconnect_on_fail config parameter, but instead we use the event handler
 *       mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;
    
    if(p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling errors from the Connection Parameters module.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;
    
    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;
    
    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
    uint32_t err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

    // Prepare wakeup buttons.
    err_code = bsp_btn_ble_sleep_mode_prepare();
    APP_ERROR_CHECK(err_code);

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    uint32_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            APP_ERROR_CHECK(err_code);
            break;
        case BLE_ADV_EVT_IDLE:
            sleep_mode_enter();
            break;
        default:
            break;
    }
}


/**@brief Function for the application's SoftDevice event handler.
 *
 * @param[in] p_ble_evt SoftDevice event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t                         err_code;
    
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            break;
            
        case BLE_GAP_EVT_DISCONNECTED:
            err_code = bsp_indication_set(BSP_INDICATE_IDLE);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            // No system attributes have been stored.
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for dispatching a SoftDevice event to all modules with a SoftDevice 
 *        event handler.
 *
 * @details This function is called from the SoftDevice event interrupt handler after a 
 *          SoftDevice event has been received.
 *
 * @param[in] p_ble_evt  SoftDevice event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    ble_conn_params_on_ble_evt(p_ble_evt);
    ble_nus_on_ble_evt(&m_nus, p_ble_evt);
    on_ble_evt(p_ble_evt);
    ble_advertising_on_ble_evt(p_ble_evt);
    bsp_btn_ble_on_ble_evt(p_ble_evt);
    
}


/**@brief Function for the SoftDevice initialization.
 *
 * @details This function initializes the SoftDevice and the BLE event interrupt.
 */
//10_03增加
static void sys_evt_dispatch(uint32_t sys_evt)
{
pstorage_sys_event_handler (sys_evt);
}
//10_03增加
static void ble_stack_init(void)
{
    uint32_t err_code;
    
    nrf_clock_lf_cfg_t clock_lf_cfg = NRF_CLOCK_LFCLKSRC;
    
    // Initialize SoftDevice.
    SOFTDEVICE_HANDLER_INIT(&clock_lf_cfg, NULL);
    
    ble_enable_params_t ble_enable_params;
    err_code = softdevice_enable_get_default_config(CENTRAL_LINK_COUNT,
                                                    PERIPHERAL_LINK_COUNT,
                                                    &ble_enable_params);
    APP_ERROR_CHECK(err_code);
        
    //Check the ram settings against the used number of links
    CHECK_RAM_START_ADDR(CENTRAL_LINK_COUNT,PERIPHERAL_LINK_COUNT);
    // Enable BLE stack.
    err_code = softdevice_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);
    
    // Subscribe for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);
		// 10_03：Register with the SoftDevice handler module for System events.
		err_code = softdevice_sys_evt_handler_set(sys_evt_dispatch);
		APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated by button press.
 */
void bsp_event_handler(bsp_event_t event)
{
    uint32_t err_code;
    switch (event)
    {
        case BSP_EVENT_SLEEP:
            sleep_mode_enter();
            break;

        case BSP_EVENT_DISCONNECT:
            err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;

        case BSP_EVENT_WHITELIST_OFF:
            err_code = ble_advertising_restart_without_whitelist();
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;

        default:
            break;
    }
}


/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
    uint32_t      err_code;
    ble_advdata_t advdata;
    ble_advdata_t scanrsp;

    // Build advertising data struct to pass into @ref ble_advertising_init.
    memset(&advdata, 0, sizeof(advdata));
    advdata.name_type          = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance = false;
		//MODIFY BY JIE AT 161015
    //advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;
		advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;

    memset(&scanrsp, 0, sizeof(scanrsp));
    scanrsp.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    scanrsp.uuids_complete.p_uuids  = m_adv_uuids;

    ble_adv_modes_config_t options = {0};
    options.ble_adv_fast_enabled  = BLE_ADV_FAST_ENABLED;
    options.ble_adv_fast_interval = APP_ADV_INTERVAL;
    options.ble_adv_fast_timeout  = APP_ADV_TIMEOUT_IN_SECONDS;

    err_code = ble_advertising_init(&advdata, &scanrsp, &options, on_adv_evt, NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for placing the application in low power state while waiting for events.
 */
static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}
//10-02
//添加flash相关的代码和变量。

pstorage_handle_t   g_block_id;            //定义全局变量，flash存储的地址索引
uint8_t                   my_flag = 0;        //定义标志标量。Main函数中会根据标志来       //读flash
uint8_t                   my_buff[8]={0};    //全局数组用来存放手机发过来的数据，          //然后写到flash中

static void my_cb(pstorage_handle_t  *handle, uint8_t op_code, uint32_t result, uint8_t  * p_data, uint32_t data_len)  
	{  
  switch(op_code)  
		{        
      case PSTORAGE_UPDATE_OP_CODE:  
         if (result == NRF_SUCCESS)  
          {  
              my_flag = 1; //当flash update完成后置位标志。 Main函数中便可以读flash数据了  
          }  
          else  
           {  
              // Update operation failed.  
          }  
          break;  
		}  
	}  
//
static void on_write(ble_nus_t * p_nus, ble_evt_t * p_ble_evt)  
{  
    ble_gatts_evt_write_t * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;  
  
    if ( (p_evt_write->handle == p_nus->rx_handles.cccd_handle)&& (p_evt_write->len == 2))  
    {  
         // 
    }  
    else if ( (p_evt_write->handle == p_nus->tx_handles.value_handle)&& (p_nus->data_handler != NULL))  
    {    //屏蔽掉之前的处理函数，添加flash 更新数据操作。  
        //p_nus->data_handler(p_nus, p_evt_write->data, p_evt_write->len);  
         pstorage_handle_t      dest_block_id;  
         uint8_t len = p_evt_write->len>8?8:p_evt_write->len;  
         memcpy(my_buff, p_evt_write->data, len);  
         pstorage_block_identifier_get(&g_block_id, 0, &dest_block_id);   //因为只注册了一个flash块，所以其实这个函数可以不调用的。  
         pstorage_update(&dest_block_id, my_buff, 8, 0);  
     }  
    else  
    {  
        // Do Nothing. This event is not relevant for this service.  
    }  
}  
	
	
//10-02
//10_03增加：自定义变量
pstorage_handle_t m_storage_test;
uint32_t flash_test[2]={8,10};
uint32_t flash_test2[2];
static void YhTest_flash(void)
{
pstorage_module_param_t param;
uint32_t err_code = pstorage_init(); //flash init
APP_ERROR_CHECK(err_code);
param.block_size = 1024; //存储的大小
param.block_count = 1; //分页
param.cb = my_cb; //回调函数
pstorage_register(&param, &m_storage_test); //注册
pstorage_clear(&m_storage_test, 1024); //clear
//保存数据
pstorage_store(&m_storage_test,(uint8_t *)&flash_test[0],8,0);
//保存数据不是立刻可以存储，而是通过协议栈队列实现，因此读取数据最后确认操作完成后操作
nrf_delay_ms(50);
pstorage_load((uint8_t *)&flash_test2[0],&m_storage_test,8,0); //读取
nrf_delay_ms(2);	
}
//10_03
//10_04:SPI 
void SPI_Init(void)
{
	nrf_gpio_cfg_output(MOSI_PIN);
	nrf_gpio_cfg_input(MISO_PIN,NRF_GPIO_PIN_NOPULL);	
	nrf_gpio_cfg_output(SCLK_PIN);
	nrf_gpio_cfg_output(CS_PIN);
	nrf_gpio_pin_set(CS_PIN);

	NRF_SPI0 -> PSELMOSI = MOSI_PIN;
	NRF_SPI0 -> PSELMISO = MISO_PIN;
	NRF_SPI0 -> PSELSCK = SCLK_PIN;
	NRF_SPI0 -> FREQUENCY = (SPI_FREQUENCY_FREQUENCY_M1 << SPI_FREQUENCY_FREQUENCY_Pos);
	NRF_SPI0 -> CONFIG =	(SPI_CONFIG_ORDER_MsbFirst << SPI_CONFIG_ORDER_Pos)	|
							(SPI_CONFIG_CPHA_Trailing << SPI_CONFIG_CPHA_Pos)	|
							(SPI_CONFIG_CPOL_ActiveHigh << SPI_CONFIG_CPOL_Pos);
							
	NRF_SPI0 -> ENABLE = (SPI_ENABLE_ENABLE_Enabled << SPI_ENABLE_ENABLE_Pos);

}
 /*
 输入:电气参数的存储地址,RN计量芯片寄存器地址，参数长度
 输出:电气参数值
 */
void SPI_RnIC_Read( uint8_t fn,uint8_t * RxdTemp,uint8_t num)
{
		uint8_t SPI0_RxdTemp,i;
	
		nrf_gpio_pin_clear(CS_PIN);
		nrf_delay_us(1);
	//发送寄存器地址
		NRF_SPI0 -> TXD = fn;
		while( NRF_SPI0 -> EVENTS_READY != 1)
			;
		SPI0_RxdTemp = NRF_SPI0 -> RXD;
		NRF_SPI0 -> EVENTS_READY = 0;
		for(i=0;i<num;i++)
		{
		NRF_SPI0 -> TXD = 0x00;
		while( NRF_SPI0 -> EVENTS_READY != 1)
			;
		RxdTemp[i-1] = NRF_SPI0 -> RXD;
		//UART0_SEND_CHAR(SPI0_RxdTemp);
		NRF_SPI0 -> EVENTS_READY = 0;
		}
}

//
void SPI_Rn_DeviceID(uint8_t fn)
{
		uint8_t SPI0_RxdTemp[4];
	
		nrf_gpio_pin_clear(CS_PIN);
		nrf_delay_us(1);
		
		NRF_SPI0 -> TXD = fn;
		while( NRF_SPI0 -> EVENTS_READY != 1)
			;
		SPI0_RxdTemp[0] = NRF_SPI0 -> RXD;
		NRF_SPI0 -> EVENTS_READY = 0;
		
		NRF_SPI0 -> TXD = 0x00;
		while( NRF_SPI0 -> EVENTS_READY != 1)
			;
		SPI0_RxdTemp[1] = NRF_SPI0 -> RXD;
		UART0_SEND_CHAR(SPI0_RxdTemp[1]);
		NRF_SPI0 -> EVENTS_READY = 0;
		
		NRF_SPI0 -> TXD = 0x00;
		while( NRF_SPI0 -> EVENTS_READY != 1)
			;
		SPI0_RxdTemp[2] = NRF_SPI0 -> RXD;
		UART0_SEND_CHAR(SPI0_RxdTemp[2]);
		NRF_SPI0 -> EVENTS_READY = 0;
		
		NRF_SPI0 -> TXD = 0x00;
		while( NRF_SPI0 -> EVENTS_READY != 1)
			;
		SPI0_RxdTemp[3] = NRF_SPI0 -> RXD;
		UART0_SEND_CHAR(SPI0_RxdTemp[3]);
		NRF_SPI0 -> EVENTS_READY = 0;

		nrf_delay_us(1);
		nrf_gpio_pin_set(CS_PIN);
}
void SPI_RnWri(uint8_t fn,uint8_t* TxdTemp,uint8_t num)
{
		uint8_t SPI0_RxdTemp[4],i;
	
		nrf_gpio_pin_clear(CS_PIN);
		nrf_delay_us(1);
		
		NRF_SPI0 -> TXD = fn;
		while( NRF_SPI0 -> EVENTS_READY != 1)
			;
		SPI0_RxdTemp[0] = NRF_SPI0 -> RXD;
		NRF_SPI0 -> EVENTS_READY = 0;	
		for(i=0;i<num;i++)
		{
		NRF_SPI0 -> TXD = *TxdTemp++;
		while( NRF_SPI0 -> EVENTS_READY != 1)
			;
		NRF_SPI0 -> EVENTS_READY = 0;
	  }
		nrf_delay_us(1);
		nrf_gpio_pin_set(CS_PIN);
}
//

//10_04:SPIend
/**@brief Application main function.
 */
int main(void)
{
    uint32_t err_code;
    // Initialize.
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);
	  
		//
    ble_stack_init();
    gap_params_init();
    services_init();
    advertising_init();
    conn_params_init();
    //flash 注册
    err_code = ble_advertising_start(BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(err_code);
	//
	Timer_Init();
    
	//配置两个指示灯，用作软件探针
	nrf_gpio_cfg_output(LED1_PIN);
    nrf_gpio_pin_set(LED1_PIN);

    nrf_gpio_cfg_output(LED2_PIN);
    nrf_gpio_pin_clear(LED2_PIN);
//10_02
	pstorage_module_param_t module_param;  
	module_param.block_count = 1;  // 申请了一个块  
	module_param.block_size = 16;  //块大小为16(最小要求是16，但是后面的处理我们都只处理手机发过来的前8字节)  
	module_param.cb =  my_cb;  
	pstorage_init();               //初始化  
	pstorage_register(&module_param, &g_block_id);//注册申请 

//10_02
	SPI_Init();
	Uart_Init();
	WIFI_Init();//wifi模块初始化需要发送AT指令，必须先初始化串口后再初始化wifi模块！！！！！！！
	
	
	nrf_gpio_pin_toggle(LED1_PIN);
	nrf_gpio_pin_toggle(LED2_PIN);
	nrf_delay_ms(500);
	
	fnYHcom_Init();
	//sec_reg=0;
	//min_reg=0;
	sys_time.sec = 30;
	sys_time.min = 15;
	sys_time.hour = 9;
	sys_time.day = 4;
	sys_time.mon = 11;
	sys_time.year = 17;
	emu_regs.SYSCON = 220;
	emu_regs.EMUCON = 5;
	at_cnt = 0;
	
//10_03
	YhTest_flash();
//10_03

    // Enter main loop.
    //for (;;)
		while(1)
    {
		//10_04
			 nrf_delay_ms(1000);
			 SPI_RnIC_Read( ADDR_URMS,(uint8_t *)(&EMU_YH.U),4);
			 SPI_RnIC_Read( ADDR_IARMS,(uint8_t *)(&EMU_YH.I[0]),4);
			 SPI_RnIC_Read( ADDR_IBRMS,(uint8_t *)(&EMU_YH.I[1]),4);
			//10_04

	
		if(flag)
		{		 
			
			//10-02
			 if ( my_flag == 1 )
				 {         
 					 ble_nus_string_send(&m_nus,"read flash: ", 12);
           my_flag = 0;
           pstorage_handle_t    dest_block_id;
           uint8_t buff[8];
       //因为只注册了一个flash块，所以其实这个函数可以不调用的。           pstorage_block_identifier_get(&g_block_id,0,&dest_block_id);
           pstorage_load(buff, &dest_block_id, 8, 0);
           for(uint8_t i = 0; i< 8; i++){
              ble_nus_string_send(&m_nus, &i, 1);
           }
				 }
			//10-02
							
			nrf_delay_ms(10);
			nrf_gpio_pin_toggle(LED1_PIN);
			nrf_gpio_pin_toggle(LED2_PIN);
			err_code = ble_nus_string_send(&m_nus, "Yh_WelcomeYou!", 14);
			err_code = ble_nus_string_send(&m_nus, recvstring, strlen((const char *)recvstring));
			if (err_code != NRF_ERROR_INVALID_STATE)
			{
				APP_ERROR_CHECK(err_code);
			}
			
			UART0_SEND_STRING(recvstring);
			
			memset(recvstring,0,strlen((const char *)recvstring));
			flag = 0;

		}

		fnYHcomPk_Exec();
	
/*		
		if(StartNetCheck == 0x01)
		{
			StartNetCheck = 0x0;
			check_NetStatus();
		}
		NetReLinking();
*/		
		// nrf_delay_ms(500);
        // power_manage();
    }
}

void UART0_IRQHandler(void)
{
	while( NRF_UART0 -> EVENTS_RXDRDY != 1)
		;
	NRF_UART0 -> EVENTS_RXDRDY = 0;	
	rxdTemp = NRF_UART0 -> RXD;
	//UART0_RxdTemp = NRF_UART0 -> RXD;
	if(CIPSTATUS_Flag != 1)
	{
		ComBox.RxBuf[ComBox.inbyte_cnt] = rxdTemp;//UART0_RxdTemp;
		ComBox.inbyte_cnt++;
		if(ComBox.inbyte_cnt>= MAX_YHCOMPACK_SIZE)
			ComBox.inbyte_cnt =0;
	}
	else{
		RxBuf[at_cnt] = rxdTemp;
		at_cnt++;
		if(at_cnt>= 50)
			at_cnt =0;
	}
	
}

/** 
 * @}
 */
