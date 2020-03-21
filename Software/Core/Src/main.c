
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "eeprom.h"

/* Private typedef -----------------------------------------------------------*/
CAN_TxHeaderTypeDef CAN2TxMessage; //CAN2 Tx message header

CAN_RxHeaderTypeDef CAN2RxMessage; //CAN2 Rx message header

uint32_t            CAN2TxMailbox; //CAN2 Tx Mailbox ID

CAN_TxHeaderTypeDef CAN1TxMessage; //CAN1 Tx message header

CAN_RxHeaderTypeDef CAN1RxMessage; //CAN1 Rx message header

uint32_t            CAN1TxMailbox; //CAN1 Tx Mailbox ID
/* Private define ------------------------------------------------------------*/
typedef enum{
	CAN_1000KBPS=1,
	CAN_500KBPS=2,
	CAN_250KBPS=3,
	CAN_150KBPS=4,
	CAN_100KBPS=5,
	CAN_50KBPS=6
}CAN_SpeedTypeDef;
#ifdef __GNUC__
  /* With GCC, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */


/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void ToggleLed1(void);
void BlinkLed2(void);
bool Can_Init(CAN_HandleTypeDef *myhcan);
bool Change_Can_Speed(CAN_HandleTypeDef *myhcan,char canspeed_char);
void PrintMenu(void);
void PrintCanSpeed(void);
void PrintConfig(void);
void EEPROM_Block_Read(uint16_t *BlockList);
void EEPROM_Block_Write(uint16_t *BlockList);
bool Serial_Available(UART_HandleTypeDef *huart);
void MenuDecode(char decodechar);
bool cancheckblock(void);
void clearbuffer(uint8_t *buffer);
bool candecode(uint8_t *buffer);
void DeleteBlockList(uint16_t BlockedCanID);
void AddtoBlockList(uint16_t BlockedCanID);

/* Private variables ---------------------------------------------------------*/
uint16_t VirtAddVarTab[20]={20777,20778,20779,20780,20781,20782,20783,20784,20785,20786,20787,20788,20789,20790};
uint16_t BlockList[10];
uint8_t Can1RxData[8];
char Rxchar;
char canspeedchar='2';
bool start=false;
bool debug=true;
uint8_t CAN1RxData[8];

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  printf("System initializated.\r\n");

  EE_Init();
  EEPROM_Block_Read(BlockList);
  if(Can_Init(&hcan1)){
	  printf("Using CAN-1 - initialization completed.\r\n");
  }
  else{
	  printf("CAN-1 initialization (sync) ERROR\r\n");
  }
  if(Can_Init(&hcan2)){
	  printf("Using CAN-2 - initialization completed.\r\n");
  }
  else{
	  printf("CAN-2 initialization (sync) ERROR\r\n");
  }
  HAL_TIM_Base_Start_IT(&htim3);
  PrintMenu();
  /* Infinite loop */
  while (1)
  {
	if(Serial_Available(&huart1)){
		HAL_UART_Receive(&huart1, (uint8_t*)&Rxchar, 1, 1);
		MenuDecode(Rxchar);
	}
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.Prediv1Source = RCC_PREDIV1_SOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL2.PLL2State = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the Systick interrupt time 
  */
  __HAL_RCC_PLLI2S_ENABLE();
}

bool Can_Init(CAN_HandleTypeDef *myhcan){
	CAN_FilterTypeDef myFilterConfig;
	myFilterConfig.FilterBank = 0;
	myFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	myFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	myFilterConfig.FilterIdHigh = 0x000;
	myFilterConfig.FilterIdLow = 0x0000;
	myFilterConfig.FilterMaskIdHigh = 0x0000; //0xFFE0
	myFilterConfig.FilterMaskIdLow = 0x0000;
	myFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	myFilterConfig.FilterActivation = ENABLE;
	myFilterConfig.SlaveStartFilterBank = 14;
	if(HAL_CAN_ConfigFilter(myhcan, &myFilterConfig) != HAL_OK)
	{
	  return false;
	}
	HAL_Delay(2);
	if (HAL_CAN_Start(myhcan) != HAL_OK)
	{
	    return false;
	    Error_Handler();
	}
	CAN2TxMessage.RTR = CAN_RTR_DATA;
	CAN2TxMessage.IDE = CAN_ID_STD;
	CAN2TxMessage.TransmitGlobalTime = DISABLE;

	return true;
}
bool Change_Can_Speed(CAN_HandleTypeDef *myhcan,char canspeed_char){
	HAL_CAN_Stop(myhcan);
	switch(canspeed_char){
		case '1':
			myhcan->Init.Prescaler = 4;
			myhcan->Init.TimeSeg1 = CAN_BS1_7TQ;
			myhcan->Init.TimeSeg2 = CAN_BS2_1TQ;
			if (HAL_CAN_Init(myhcan) != HAL_OK)
			{
				return false;
			}
			printf("CAN Speed Changed to 1000 KBPS\r\n");
			break;
		case '2':
			myhcan->Init.Prescaler = 9;
			myhcan->Init.TimeSeg1 = CAN_BS1_6TQ;
			myhcan->Init.TimeSeg2 = CAN_BS2_1TQ;
			if (HAL_CAN_Init(myhcan) != HAL_OK)
			{
				return false;
			}
			printf("CAN Speed Changed to 500 KBPS\r\n");
			break;
		case '3':
			myhcan->Init.Prescaler = 9;
			myhcan->Init.TimeSeg1 = CAN_BS1_13TQ;
			myhcan->Init.TimeSeg2 = CAN_BS2_2TQ;
			if (HAL_CAN_Init(myhcan) != HAL_OK)
			{
				return false;
			}
			printf("CAN Speed Changed to 250 KBPS\r\n");
			break;
		case '4':
			myhcan->Init.Prescaler = 15;
			myhcan->Init.TimeSeg1 = CAN_BS1_13TQ;
			myhcan->Init.TimeSeg2 = CAN_BS2_2TQ;
			if (HAL_CAN_Init(myhcan) != HAL_OK)
			{
				return false;
			}
			printf("CAN Speed Changed to 150 KBPS\r\n");
			break;
		case '5':
			myhcan->Init.Prescaler = 45;
			myhcan->Init.TimeSeg1 = CAN_BS1_6TQ;
			myhcan->Init.TimeSeg2 = CAN_BS2_1TQ;
			if (HAL_CAN_Init(myhcan) != HAL_OK)
			{
				return false;
			}
			printf("CAN Speed Changed to 100 KBPS\r\n");
			break;
		case '6':
			myhcan->Init.Prescaler = 45;
			myhcan->Init.TimeSeg1 = CAN_BS1_13TQ;
			myhcan->Init.TimeSeg2 = CAN_BS2_2TQ;
			if (HAL_CAN_Init(myhcan) != HAL_OK)
			{
				return false;
			}
			printf("CAN Speed Changed to 50 KBPS\r\n");
			break;
		default:
			printf("Invalid Input\r\n");
	}
	if(HAL_CAN_Start(myhcan)!=HAL_OK){
		return false;
	}
	return true;
}
void PrintMenu(void){
	printf("\r\nPlease press the button for the action you want to do\r\n");
	printf("1-Change CAN-1 Speed\r\n");
	printf("2-Change CAN-2 Speed\r\n");
	printf("3-Block CAN ID's\r\n");
	printf("4-Unblock CAN ID's\r\n");
	printf("5-Modify CAN Messages\r\n");
	printf("6-Send Custom Messages via CAN-1\r\n");
	printf("7-Send Custom Messages via CAN-2\r\n");
	printf("c-Show Current Configuration.\r\n");
	printf("d-Enable/Disable Debug\r\n");
	printf("s-Start/Stop The Man in the Middle\r\n");
	printf("m-Show Menu\r\n");
}
void PrintCanSpeed(void){
	printf("Please Select The Can Speed\r\n");
	printf("1-1000 KBPS\r\n");
	printf("2-500  KBPS(default)\r\n");
	printf("3-250  KBPS\r\n");
	printf("4-150  KBPS\r\n");
	printf("5-100  KBPS\r\n");
	printf("6-50   KPBS\r\n");
}
void PrintConfig(void){
	int counter=1;
	printf("Block List\r\n");
	for(int i=0;i<10;i++){
		if(BlockList[i]!=0){
			printf("%d-%03x\r\n",counter,BlockList[i]);
			counter++;
		}
	}
}
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the EVAL_COM1 and Loop until the end of transmission */
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);

  return ch;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance==TIM3){
		ToggleLed1();
	}
}
void ToggleLed1(void){
	HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
}
void EEPROM_Block_Read(uint16_t *BlockList){
	HAL_FLASH_Unlock();
	EE_ReadVariable(VirtAddVarTab[0], &BlockList[0]);
	EE_ReadVariable(VirtAddVarTab[1], &BlockList[1]);
	EE_ReadVariable(VirtAddVarTab[2], &BlockList[2]);
	EE_ReadVariable(VirtAddVarTab[3], &BlockList[3]);
	EE_ReadVariable(VirtAddVarTab[4], &BlockList[4]);
	EE_ReadVariable(VirtAddVarTab[5], &BlockList[5]);
	EE_ReadVariable(VirtAddVarTab[6], &BlockList[6]);
	EE_ReadVariable(VirtAddVarTab[7], &BlockList[7]);
	EE_ReadVariable(VirtAddVarTab[8], &BlockList[8]);
	EE_ReadVariable(VirtAddVarTab[9], &BlockList[9]);
	HAL_FLASH_Lock();
}
void EEPROM_Block_Write(uint16_t *BlockList){
	HAL_FLASH_Unlock();
	EE_WriteVariable(VirtAddVarTab[0], BlockList[0]);
	EE_WriteVariable(VirtAddVarTab[1], BlockList[1]);
	EE_WriteVariable(VirtAddVarTab[2], BlockList[2]);
	EE_WriteVariable(VirtAddVarTab[3], BlockList[3]);
	EE_WriteVariable(VirtAddVarTab[4], BlockList[4]);
	EE_WriteVariable(VirtAddVarTab[5], BlockList[5]);
	EE_WriteVariable(VirtAddVarTab[6], BlockList[6]);
	EE_WriteVariable(VirtAddVarTab[7], BlockList[7]);
	EE_WriteVariable(VirtAddVarTab[8], BlockList[8]);
	EE_WriteVariable(VirtAddVarTab[9], BlockList[9]);
	HAL_FLASH_Lock();
}
bool Serial_Available(UART_HandleTypeDef *huart)
{
    uint32_t RX_State;

    RX_State = huart->Instance->SR & UART_FLAG_RXNE;

    //something is in the buffer
    if (RX_State > 0)
    {
        return true;
    }
    //nothing is there
    return false;
}
void MenuDecode(char decodechar){
	char CANIDRx[5];
	char datalenght;
	uint8_t datalenghtint;
	unsigned int BlockedCanID;
	unsigned int TxData[8];
	char CANRxcharData[4];
	switch(decodechar){
		case '1':
			PrintCanSpeed();
			HAL_UART_Receive(&huart1, (uint8_t*)&canspeedchar, 1, HAL_MAX_DELAY);
			Change_Can_Speed(&hcan1, canspeedchar);
			break;
		case '2':
			PrintCanSpeed();
			HAL_UART_Receive(&huart1, (uint8_t*)&canspeedchar, 1, HAL_MAX_DELAY);
			Change_Can_Speed(&hcan2, canspeedchar);
			break;
		case '3':
			printf("Please Input CAN ID in 0xXXX format.\r\n");
			HAL_UART_Receive(&huart1, (uint8_t*)CANIDRx, 5, HAL_MAX_DELAY);
			sscanf(CANIDRx,"%x",&BlockedCanID);
			AddtoBlockList(BlockedCanID);
			break;
		case '4':
			printf("Please Input CAN ID in 0xXXX format.\r\n");
			HAL_UART_Receive(&huart1, (uint8_t*)CANIDRx, 5, HAL_MAX_DELAY);
			sscanf(CANIDRx,"%x",&BlockedCanID);
			DeleteBlockList(BlockedCanID);
			break;
		case '5':
			printf("This Option is Disabled for now.\r\n");
			break;
		case '6':
			printf("Please Input DataLenght.(1-8)\r\n");
			HAL_UART_Receive(&huart1, (uint8_t*)&datalenght, 1, HAL_MAX_DELAY);
			datalenghtint = (uint8_t)(datalenght-'0');
			if(datalenghtint>8)
				printf("Invalid data lenght\r\n");
			printf("Input the Data in 0xXX format");
			for(int i=0;i<datalenghtint;i++){
				HAL_UART_Receive(&huart1, (uint8_t*)CANRxcharData, 4, HAL_MAX_DELAY);
				sscanf(CANRxcharData,"%x",&TxData[i]);
			}
			CAN1TxMessage.DLC=datalenghtint;
			CAN1TxMessage.StdId=0x001;
			if(HAL_CAN_AddTxMessage(&hcan1, &CAN1TxMessage, (uint8_t*)TxData, &CAN1TxMailbox)==HAL_OK){
				if(debug)
					printf("Message Sent via CAN-1\r\n");
			}else{
				printf("Error while sending message\r\n");
			}
			break;
		case '7':
			printf("Please Input DataLenght.(1-8)\r\n");
			HAL_UART_Receive(&huart1, (uint8_t*)&datalenght, 1, HAL_MAX_DELAY);
			datalenghtint = (uint8_t)(datalenght-'0');
			if(datalenghtint>8)
				printf("Invalid data lenght\r\n");
			printf("Input the Data in 0xXX format");
			for(int i=0;i<datalenghtint;i++){
				HAL_UART_Receive(&huart1, (uint8_t*)CANRxcharData, 4, HAL_MAX_DELAY);
				sscanf(CANRxcharData,"%x",&TxData[i]);
			}
			CAN2TxMessage.DLC=datalenghtint;
			CAN2TxMessage.StdId=0x001;
			if(HAL_CAN_AddTxMessage(&hcan2, &CAN2TxMessage, (uint8_t*)TxData, &CAN2TxMailbox)==HAL_OK){
				if(debug)
					printf("Message Sent via CAN-2\r\n");
			}else{
				printf("Error while sending message\r\n");
			}
			break;
		case 'c':
			PrintConfig();
			break;
		case 'd':
			if(debug){
				debug=false;
				printf("Debug Disabled\r\n");
			}
			else{
				debug=true;
				printf("Debug Enabled\r\n");
			}
			break;
		case 's':
			if(start){
				start=false;
				if(debug)
					printf("MITM Stopped\r\n");
			}
			else{
				start=true;
				if(debug)
					printf("MITM Starteded\r\n");
			}
			break;
		case 'm':
			PrintMenu();
			break;
		default:
			printf("Invalid Input\r\n");
			break;
	}
}
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){
	HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
	clearbuffer(CAN1RxData);
	if(HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &CAN1RxMessage, CAN1RxData)==HAL_OK){
		if(debug){
			printf("Incoming CAN Message\r\n");
			printf("Message arrived from 0x%03lx\r\n",CAN1RxMessage.StdId);
			for(int i=0;i<CAN1RxMessage.DLC;i++){
				printf("0x%x ",CAN1RxData[i]);
			}
		}
		if(start){
			if(!cancheckblock()){
				if(debug){
					printf("\r\nMessage Blocked\r\n");
				}
			}
			else{
				if(candecode(CAN1RxData)){
					if(debug){
						printf("Message Modified\r\n");
					}
				}
				if(debug){
					printf("Sending the Message via CAN-2\r\n");
				}
				CAN2TxMessage.DLC=CAN1RxMessage.DLC;
				CAN2TxMessage.StdId=CAN1RxMessage.StdId;
				HAL_CAN_AddTxMessage(&hcan2, &CAN2TxMessage, CAN1RxData, &CAN2TxMailbox);
			}
		}
	}
	HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
}
bool cancheckblock(void){
	for(int i=0;i<10;i++){
		if(CAN1RxMessage.StdId==BlockList[i]){
			return false;
		}
	}
	return true;
}
void clearbuffer(uint8_t *buffer){
	for(int i=0;i<8;i++){
		buffer[i]=0;
	}
}
bool candecode(uint8_t *buffer){
	return false;
}
void AddtoBlockList(uint16_t BlockedCanID){
	for(int i=0;i<10;i++){
		if(BlockList[i]==BlockedCanID){
			if(debug)
				printf("0x%03x Already Blocked\r\n",BlockedCanID);
			return;
		}
	}
	for(int i=0;i<10;i++){
		if(BlockList[i]==0){
			BlockList[i]=BlockedCanID;
			if(debug)
				printf("0x%03x Added to Block List\r\n",BlockedCanID);
			EEPROM_Block_Write(BlockList);
			return;
		}
	}
	if(debug)
		printf("Block List is Full.\r\n");
}
void DeleteBlockList(uint16_t BlockedCanID){
	for(int i=0;i<10;i++){
		if(BlockList[i]==BlockedCanID){
			BlockList[i]=0;
			if(debug)
				printf("0x%03x Unblocked.\r\n",BlockedCanID);
			EEPROM_Block_Write(BlockList);
			return;
		}
	}
	if(debug)
		printf("0x%03x was not in Block List.\r\n",BlockedCanID);
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* User can add his own implementation to report the HAL error return state */
	printf("There was Hard Error.\r\n");
}


