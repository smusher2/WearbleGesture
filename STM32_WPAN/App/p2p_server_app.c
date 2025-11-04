/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : App/p2p_server_app.c
  * Description        : P2P Server Application
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2019-2021 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "app_common.h"
#include "dbg_trace.h"
#include "ble.h"
#include "p2p_server_app.h"
#include "stm32_seq.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
static uint16_t hello_service_handle;
static uint16_t hello_char_handle;
extern uint8_t hello_timer_id;
static uint8_t sensorData = 0;

uint8_t hello_timer_id;  // Global timer ID for Hello messages
uint8_t ble_connected = 0;



/* USER CODE BEGIN P2P_PROTOTYPES */
static void Send_Hello_Data(void);
/* USER CODE END P2P_PROTOTYPES */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
 //typedef struct{
   // uint8_t             Device_Led_Selection;
    //uint8_t             Led1;
 //}P2P_LedCharValue_t;

 //typedef struct{
 //   uint8_t             Device_Button_Selection;
 //   uint8_t             ButtonStatus;
// }P2P_ButtonCharValue_t;

typedef struct
{
  uint8_t               Notification_Status; /* used to check if P2P Server is enabled to Notify */
  //P2P_LedCharValue_t    LedControl;
 // P2P_ButtonCharValue_t ButtonControl;
  uint16_t              ConnectionHandle;
} P2P_Server_App_Context_t;
/* USER CODE END PTD */

/* Private defines ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macros -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
/**
 * START of Section BLE_APP_CONTEXT
 */

static P2P_Server_App_Context_t P2P_Server_App_Context;

/**
 * END of Section BLE_APP_CONTEXT
 *//* Custom Hello characteristic handles */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
static void P2PS_Send_Notification(void);
static void P2PS_APP_LED_BUTTON_context_Init(void);
/* USER CODE END PFP */

/* Functions Definition ------------------------------------------------------*/
void P2PS_STM_App_Notification(P2PS_STM_App_Notification_evt_t *pNotification)
{
/* USER CODE BEGIN P2PS_STM_App_Notification_1 */
aPwmLedGsData_TypeDef aPwmLedGsData;
/* USER CODE END P2PS_STM_App_Notification_1 */
  switch(pNotification->P2P_Evt_Opcode)
  {
/* USER CODE BEGIN P2PS_STM_App_Notification_P2P_Evt_Opcode */

#if(BLE_CFG_OTA_REBOOT_CHAR != 0)
    case P2PS_STM_BOOT_REQUEST_EVT:
      APP_DBG_MSG("-- P2P APPLICATION SERVER : BOOT REQUESTED\n");
      APP_DBG_MSG(" \n\r");

      *(uint32_t*)SRAM1_BASE = *(uint32_t*)pNotification->DataTransfered.pPayload;
      NVIC_SystemReset();
      break;
#endif
/* USER CODE END P2PS_STM_App_Notification_P2P_Evt_Opcode */

    case P2PS_STM__NOTIFY_ENABLED_EVT:
/* USER CODE BEGIN P2PS_STM__NOTIFY_ENABLED_EVT */
      P2P_Server_App_Context.Notification_Status = 1;
      APP_DBG_MSG("-- P2P APPLICATION SERVER : NOTIFICATION ENABLED\n");
      APP_DBG_MSG(" \n\r");
/* USER CODE END P2PS_STM__NOTIFY_ENABLED_EVT */
      break;

    case P2PS_STM_NOTIFY_DISABLED_EVT:
/* USER CODE BEGIN P2PS_STM_NOTIFY_DISABLED_EVT */
      P2P_Server_App_Context.Notification_Status = 0;
      APP_DBG_MSG("-- P2P APPLICATION SERVER : NOTIFICATION DISABLED\n");
      APP_DBG_MSG(" \n\r");
/* USER CODE END P2PS_STM_NOTIFY_DISABLED_EVT */
      break;

    case P2PS_STM_WRITE_EVT:
/* USER CODE BEGIN P2PS_STM_WRITE_EVT */
      if(pNotification->DataTransfered.pPayload[0] == 0x00){ /* ALL Deviceselected - may be necessary as LB Routeur informs all connection */
        if(pNotification->DataTransfered.pPayload[1] == 0x01)
        {
          aPwmLedGsData[PWM_LED_RED]   = PWM_LED_GSDATA_OFF;
          aPwmLedGsData[PWM_LED_GREEN] = PWM_LED_GSDATA_OFF;
          aPwmLedGsData[PWM_LED_BLUE]  = PWM_LED_GSDATA_7_0;
          LED_On(aPwmLedGsData);
          APP_DBG_MSG("-- P2P APPLICATION SERVER  : LED1 ON\n");
          APP_DBG_MSG(" \n\r");
          //P2P_Server_App_Context.LedControl.Led1=0x01; /* LED1 ON */
        }
        if(pNotification->DataTransfered.pPayload[1] == 0x00)
        {
          LED_Off();
          APP_DBG_MSG("-- P2P APPLICATION SERVER  : LED1 OFF\n");
          APP_DBG_MSG(" \n\r");
        //  P2P_Server_App_Context.LedControl.Led1=0x00; /* LED1 OFF */
        }
      }
#if(P2P_SERVER1 != 0)
      if(pNotification->DataTransfered.pPayload[0] == 0x01){ /* end device 1 selected - may be necessary as LB Routeur informs all connection */
        if(pNotification->DataTransfered.pPayload[1] == 0x01)
        {
          aPwmLedGsData[PWM_LED_RED]   = PWM_LED_GSDATA_OFF;
          aPwmLedGsData[PWM_LED_GREEN] = PWM_LED_GSDATA_OFF;
          aPwmLedGsData[PWM_LED_BLUE]  = PWM_LED_GSDATA_7_0;
          LED_On(aPwmLedGsData);
          APP_DBG_MSG("-- P2P APPLICATION SERVER 1 : LED1 ON\n");
          APP_DBG_MSG(" \n\r");
         // P2P_Server_App_Context.LedControl.Led1=0x01; /* LED1 ON */
        }
        if(pNotification->DataTransfered.pPayload[1] == 0x00)
        {
          LED_Off();
          APP_DBG_MSG("-- P2P APPLICATION SERVER 1 : LED1 OFF\n");
          APP_DBG_MSG(" \n\r");
         // P2P_Server_App_Context.LedControl.Led1=0x00; /* LED1 OFF */
        }
      }
#endif
#if(P2P_SERVER2 != 0)
      if(pNotification->DataTransfered.pPayload[0] == 0x02){ /* end device 2 selected */
        if(pNotification->DataTransfered.pPayload[1] == 0x01)
        {
          aPwmLedGsData[PWM_LED_RED]   = PWM_LED_GSDATA_OFF;
          aPwmLedGsData[PWM_LED_GREEN] = PWM_LED_GSDATA_OFF;
          aPwmLedGsData[PWM_LED_BLUE]  = PWM_LED_GSDATA_7_0;
          LED_On(aPwmLedGsData);
          APP_DBG_MSG("-- P2P APPLICATION SERVER 2 : LED1 ON\n");
          APP_DBG_MSG(" \n\r");
          P2P_Server_App_Context.LedControl.Led1=0x01; /* LED1 ON */
        }
        if(pNotification->DataTransfered.pPayload[1] == 0x00)
        {
          LED_Off();
          APP_DBG_MSG("-- P2P APPLICATION SERVER 2 : LED1 OFF\n");
          APP_DBG_MSG(" \n\r");
          P2P_Server_App_Context.LedControl.Led1=0x00; /* LED1 OFF */
        }
      }
#endif
#if(P2P_SERVER3 != 0)
      if(pNotification->DataTransfered.pPayload[0] == 0x03){ /* end device 3 selected - may be necessary as LB Routeur informs all connection */
        if(pNotification->DataTransfered.pPayload[1] == 0x01)
        {
          aPwmLedGsData[PWM_LED_RED]   = PWM_LED_GSDATA_OFF;
          aPwmLedGsData[PWM_LED_GREEN] = PWM_LED_GSDATA_OFF;
          aPwmLedGsData[PWM_LED_BLUE]  = PWM_LED_GSDATA_7_0;
          LED_On(aPwmLedGsData);
          APP_DBG_MSG("-- P2P APPLICATION SERVER 3 : LED1 ON\n");
          APP_DBG_MSG(" \n\r");
          P2P_Server_App_Context.LedControl.Led1=0x01; /* LED1 ON */
        }
        if(pNotification->DataTransfered.pPayload[1] == 0x00)
        {
          LED_Off();
          APP_DBG_MSG("-- P2P APPLICATION SERVER 3 : LED1 OFF\n");
          APP_DBG_MSG(" \n\r");
          P2P_Server_App_Context.LedControl.Led1=0x00; /* LED1 OFF */
        }
      }
#endif
#if(P2P_SERVER4 != 0)
      if(pNotification->DataTransfered.pPayload[0] == 0x04){ /* end device 4 selected */
        if(pNotification->DataTransfered.pPayload[1] == 0x01)
        {
          aPwmLedGsData[PWM_LED_RED]   = PWM_LED_GSDATA_OFF;
          aPwmLedGsData[PWM_LED_GREEN] = PWM_LED_GSDATA_OFF;
          aPwmLedGsData[PWM_LED_BLUE]  = PWM_LED_GSDATA_7_0;
          LED_On(aPwmLedGsData);
          APP_DBG_MSG("-- P2P APPLICATION SERVER 2 : LED1 ON\n");
          APP_DBG_MSG(" \n\r");
          P2P_Server_App_Context.LedControl.Led1=0x01; /* LED1 ON */
        }
        if(pNotification->DataTransfered.pPayload[1] == 0x00)
        {
          LED_Off();
          APP_DBG_MSG("-- P2P APPLICATION SERVER 2 : LED1 OFF\n");
          APP_DBG_MSG(" \n\r");
          P2P_Server_App_Context.LedControl.Led1=0x00; /* LED1 OFF */
        }
      }
#endif
#if(P2P_SERVER5 != 0)
      if(pNotification->DataTransfered.pPayload[0] == 0x05){ /* end device 5 selected - may be necessary as LB Routeur informs all connection */
        if(pNotification->DataTransfered.pPayload[1] == 0x01)
        {
          aPwmLedGsData[PWM_LED_RED]   = PWM_LED_GSDATA_OFF;
          aPwmLedGsData[PWM_LED_GREEN] = PWM_LED_GSDATA_OFF;
          aPwmLedGsData[PWM_LED_BLUE]  = PWM_LED_GSDATA_7_0;
          LED_On(aPwmLedGsData);
          APP_DBG_MSG("-- P2P APPLICATION SERVER 5 : LED1 ON\n");
          APP_DBG_MSG(" \n\r");
          P2P_Server_App_Context.LedControl.Led1=0x01; /* LED1 ON */
        }
        if(pNotification->DataTransfered.pPayload[1] == 0x00)
        {
          LED_Off();
          APP_DBG_MSG("-- P2P APPLICATION SERVER 5 : LED1 OFF\n");
          APP_DBG_MSG(" \n\r");
          P2P_Server_App_Context.LedControl.Led1=0x00; /* LED1 OFF */
        }
      }
#endif
#if(P2P_SERVER6 != 0)
      if(pNotification->DataTransfered.pPayload[0] == 0x06){ /* end device 6 selected */
        if(pNotification->DataTransfered.pPayload[1] == 0x01)
        {
          aPwmLedGsData[PWM_LED_RED]   = PWM_LED_GSDATA_OFF;
          aPwmLedGsData[PWM_LED_GREEN] = PWM_LED_GSDATA_OFF;
          aPwmLedGsData[PWM_LED_BLUE]  = PWM_LED_GSDATA_7_0;
          LED_On(aPwmLedGsData);
          APP_DBG_MSG("-- P2P APPLICATION SERVER 6 : LED1 ON\n");
          APP_DBG_MSG(" \n\r");
          P2P_Server_App_Context.LedControl.Led1=0x01; /* LED1 ON */
        }
        if(pNotification->DataTransfered.pPayload[1] == 0x00)
        {
          LED_Off();
          APP_DBG_MSG("-- P2P APPLICATION SERVER 6 : LED1 OFF\n");
          APP_DBG_MSG(" \n\r");
          P2P_Server_App_Context.LedControl.Led1=0x00; /* LED1 OFF */
        }
      }
#endif
/* USER CODE END P2PS_STM_WRITE_EVT */
      break;

    default:
/* USER CODE BEGIN P2PS_STM_App_Notification_default */

/* USER CODE END P2PS_STM_App_Notification_default */
      break;
  }
/* USER CODE BEGIN P2PS_STM_App_Notification_2 */

/* USER CODE END P2PS_STM_App_Notification_2 */
  return;
}

void P2PS_APP_Notification(P2PS_APP_ConnHandle_Not_evt_t *pNotification)
{
/* USER CODE BEGIN P2PS_APP_Notification_1 */
	switch(pNotification->P2P_Evt_Opcode)
	  {
/* USER CODE END P2PS_APP_Notification_1 */
	case PEER_CONN_HANDLE_EVT:
	            // BLE connected
	            HAL_GPIO_WritePin(Connected_LED_GPIO_Port, Connected_LED_Pin, GPIO_PIN_RESET); // ON
	            HAL_GPIO_WritePin(Power_LED_GPIO_Port, Power_LED_Pin, GPIO_PIN_SET);           // OFF
	            APP_DBG_MSG("✅ BLE Connected\r\n");
	            HW_TS_Start(hello_timer_id, (1000 * 1000 / CFG_TS_TICK_VAL));
	            break;

	        case PEER_DISCON_HANDLE_EVT:
	            // BLE disconnected
	            HAL_GPIO_WritePin(Connected_LED_GPIO_Port, Connected_LED_Pin, GPIO_PIN_SET);   // OFF
	            HAL_GPIO_WritePin(Power_LED_GPIO_Port, Power_LED_Pin, GPIO_PIN_RESET);         // ON
	            APP_DBG_MSG("❌ BLE Disconnected\r\n");
	            HW_TS_Stop(hello_timer_id);
	            break;

    default:
        break;
  }
/* USER CODE BEGIN P2PS_APP_Notification_2 */

/* USER CODE END P2PS_APP_Notification_2 */
  return;
}

void P2PS_APP_Init(void)
{
/* USER CODE BEGIN P2PS_APP_Init */
 // UTIL_SEQ_RegTask( 1<< CFG_TASK_SW1_BUTTON_PUSHED_ID, UTIL_SEQ_RFU, P2PS_Send_Notification );

  /**
   * Initialize LedButton Service
   */
	HW_TS_Create(CFG_TIM_PROC_ID_ISR, &hello_timer_id, hw_ts_Repeated, Send_Hello_Data);
 // P2P_Server_App_Context.Notification_Status=0;
 // P2PS_APP_LED_BUTTON_context_Init();
	UTIL_SEQ_RegTask( 1<< CFG_TASK_SW1_BUTTON_PUSHED_ID, UTIL_SEQ_RFU, P2PS_Send_Notification );

	  /**
	   * Initialize LedButton Service
	   */
	  P2P_Server_App_Context.Notification_Status = 0;
	  P2PS_APP_LED_BUTTON_context_Init();
	  /* USER CODE BEGIN Custom Hello Service */
	  tBleStatus ret;
	  uint8_t hello_service_uuid[16] = {0xAA,0xBB,0xCC,0xDD,0xEE,0xFF,0x00,0x11,
	                                    0x22,0x33,0x44,0x55,0x66,0x77,0x88,0x99};
	  uint8_t hello_char_uuid[16]    = {0xAA,0xBB,0xCC,0xDD,0xEE,0xFF,0x00,0x11,
	                                    0x22,0x33,0x44,0x55,0x66,0x77,0x88,0x9A};

	  Service_UUID_t service_uuid;
	  Char_UUID_t char_uuid;

	  memcpy(service_uuid.Service_UUID_128, hello_service_uuid, 16);
	  memcpy(char_uuid.Char_UUID_128, hello_char_uuid, 16);


	  ret = aci_gatt_add_service(UUID_TYPE_128, &service_uuid, PRIMARY_SERVICE, 7, &hello_service_handle);
	  if (ret != BLE_STATUS_SUCCESS)
	  {
	      APP_DBG_MSG("Failed to add Hello service: 0x%02X\r\n", ret);
	  }

	  ret = aci_gatt_add_char(hello_service_handle,
	                          UUID_TYPE_128,
	                          &char_uuid,
	                          5,
	                          CHAR_PROP_NOTIFY,
	                          ATTR_PERMISSION_NONE,
	                          0,
	                          10, 0,
	                          &hello_char_handle);
	  APP_DBG_MSG("Service add ret=0x%02X handle=%d\r\n", ret, hello_service_handle);
	  APP_DBG_MSG("Char add ret=0x%02X handle=%d\r\n", ret, hello_char_handle);


	  if (ret != BLE_STATUS_SUCCESS)
	  {
	      APP_DBG_MSG("Failed to add Hello characteristic: 0x%02X\r\n", ret);
	  }

	  APP_DBG_MSG("✅ Custom Hello Service initialized\r\n");
	  /* USER CODE END Custom Hello Service */
	  // ✅ Enable notifications so data can be sent
	  static uint8_t hello_timer_id;

	  HW_TS_Create(CFG_TIM_PROC_ID_ISR, &hello_timer_id, hw_ts_Repeated, Send_Hello_Data);
	 // HW_TS_Start(hello_timer_id, (1000 * 1000 / CFG_TS_TICK_VAL));  // 1-second period
	  /* ---- ADD ABOVE ---- */
	  HAL_GPIO_WritePin(Power_LED_GPIO_Port, Power_LED_Pin, GPIO_PIN_RESET);     // Power ON
	  HAL_GPIO_WritePin(Connected_LED_GPIO_Port, Connected_LED_Pin, GPIO_PIN_SET); // Disconnected
	  HAL_GPIO_WritePin(Transmitting_LED_GPIO_Port, Transmitting_LED_Pin, GPIO_PIN_SET); // Not transmitting
/* USER CODE END P2PS_APP_Init */
  return;
}

/* USER CODE BEGIN FD */
void P2PS_APP_LED_BUTTON_context_Init(void){

  LED_Off();

  #if(P2P_SERVER1 != 0)
 // P2P_Server_App_Context.LedControl.Device_Led_Selection=0x01; /* Device1 */
//  P2P_Server_App_Context.LedControl.Led1=0x00; /* led OFF */
 // P2P_Server_App_Context.ButtonControl.Device_Button_Selection=0x01;/* Device1 */
 // P2P_Server_App_Context.ButtonControl.ButtonStatus=0x00;
#endif
#if(P2P_SERVER2 != 0)
  P2P_Server_App_Context.LedControl.Device_Led_Selection=0x02; /* Device2 */
  P2P_Server_App_Context.LedControl.Led1=0x00; /* led OFF */
  P2P_Server_App_Context.ButtonControl.Device_Button_Selection=0x02;/* Device2 */
  P2P_Server_App_Context.ButtonControl.ButtonStatus=0x00;
#endif
#if(P2P_SERVER3 != 0)
  P2P_Server_App_Context.LedControl.Device_Led_Selection=0x03; /* Device3 */
  P2P_Server_App_Context.LedControl.Led1=0x00; /* led OFF */
  P2P_Server_App_Context.ButtonControl.Device_Button_Selection=0x03; /* Device3 */
  P2P_Server_App_Context.ButtonControl.ButtonStatus=0x00;
#endif
#if(P2P_SERVER4 != 0)
  P2P_Server_App_Context.LedControl.Device_Led_Selection=0x04; /* Device4 */
  P2P_Server_App_Context.LedControl.Led1=0x00; /* led OFF */
  P2P_Server_App_Context.ButtonControl.Device_Button_Selection=0x04; /* Device4 */
  P2P_Server_App_Context.ButtonControl.ButtonStatus=0x00;
#endif
 #if(P2P_SERVER5 != 0)
  P2P_Server_App_Context.LedControl.Device_Led_Selection=0x05; /* Device5 */
  P2P_Server_App_Context.LedControl.Led1=0x00; /* led OFF */
  P2P_Server_App_Context.ButtonControl.Device_Button_Selection=0x05; /* Device5 */
  P2P_Server_App_Context.ButtonControl.ButtonStatus=0x00;
#endif
#if(P2P_SERVER6 != 0)
  P2P_Server_App_Context.LedControl.Device_Led_Selection=0x06; /* device6 */
  P2P_Server_App_Context.LedControl.Led1=0x00; /* led OFF */
  P2P_Server_App_Context.ButtonControl.Device_Button_Selection=0x06; /* Device6 */
  P2P_Server_App_Context.ButtonControl.ButtonStatus=0x00;
#endif
}

void P2PS_APP_SW1_Button_Action(void)
{
  UTIL_SEQ_SetTask( 1<<CFG_TASK_SW1_BUTTON_PUSHED_ID, CFG_SCH_PRIO_0);

  return;
}
/* USER CODE END FD */

/*************************************************************
 *
 * LOCAL FUNCTIONS
 *
 *************************************************************/

void P2PS_Send_Notification(void)
{
 
 // if(P2P_Server_App_Context.ButtonControl.ButtonStatus == 0x00){
  //  P2P_Server_App_Context.ButtonControl.ButtonStatus=0x01;
  //  UTIL_LCD_DisplayStringAt(0, LINE(2), (uint8_t *)"BUTTON-1 STATUS: 1", LEFT_MODE);
  //  BSP_LCD_Refresh(0);
 // } else {
 //   P2P_Server_App_Context.ButtonControl.ButtonStatus=0x00;
    //UTIL_LCD_DisplayStringAt(0, LINE(2), (uint8_t *)"BUTTON-1 STATUS: 0", LEFT_MODE);
   // BSP_LCD_Refresh(0);
  //}
  
  /* if(P2P_Server_App_Context.Notification_Status){
    APP_DBG_MSG("-- P2P APPLICATION SERVER  : INFORM CLIENT BUTTON 1 PUSHED \n ");
    APP_DBG_MSG(" \n\r");
    P2PS_STM_App_Update_Char(P2P_NOTIFY_CHAR_UUID, (uint8_t *)&P2P_Server_App_Context.ButtonControl);
   } else {
    APP_DBG_MSG("-- P2P APPLICATION SERVER : CAN'T INFORM CLIENT -  NOTIFICATION DISABLED\n "); 
   }

  return;*/
}


/* USER CODE BEGIN FD_LOCAL_FUNCTIONS*/
static void Send_Hello_Data(void)
{
	//const uint8_t msg[] = {'H', 'e', 'l', 'l', 'o'};
	uint8_t msg = 0;
	tBleStatus ret;

	    uint32_t start_time = HAL_GetTick();  // Record timestamp before sending

	    // Turn ON transmit LED (active-low)
	    HAL_GPIO_WritePin(Transmitting_LED_GPIO_Port, Transmitting_LED_Pin, GPIO_PIN_RESET);

	    // Send data
	    ret = aci_gatt_update_char_value(hello_service_handle,
	                                     hello_char_handle,
	                                     0,
	                                    1,
	                                   &msg);

	    // Turn OFF transmit LED (active-low)
	    HAL_GPIO_WritePin(Transmitting_LED_GPIO_Port, Transmitting_LED_Pin, GPIO_PIN_SET);

	    uint32_t end_time = HAL_GetTick();  // Record after send
	    uint32_t latency = end_time - start_time;  // Calculate in ms
	    if (ret == BLE_STATUS_SUCCESS)
	        {
	            APP_DBG_MSG("📤 Sent Hello (%lu ms)\r\n", latency);
	        }
	        else
	        {
	            APP_DBG_MSG("❌ Failed to send Hello (ret=0x%02X)\r\n", ret);
	        }
	    // Print latency only to terminal
	    //APP_DBG_MSG("BLE Tx Latency: %lu ms\r\n", latency);

	}
void BLE_SendInstantValue(uint8_t *data, uint8_t len)
{
    if (P2P_Server_App_Context.Notification_Status)  // only send if BLE connected
    {
        aci_gatt_update_char_value(hello_service_handle,
                                   hello_char_handle,
                                   0,
                                   len,
                                   data);
        APP_DBG_MSG("📤 Sent data: %c\r\n", *data);
    }
}
uint8_t BLE_IsReady(void)
{
    return P2P_Server_App_Context.Notification_Status;
}

/* USER CODE END FD_LOCAL_FUNCTIONS*/
