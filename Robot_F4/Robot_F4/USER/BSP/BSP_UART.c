#include "BSP_UART.h"
#include "DYPA02.h"
#include "VL53L0.h"
#include "ano_dt.h"
#include "YuTu.h"

/**
	* @brief  串口接收中断回调（普通中断模式）
  * @param  huart USART设备
	* @retval none
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

}

/**
	* @brief  串口接收中断回调（空闲中断模式）
  * @param  huart USART设备
  * @param  Size 	数据长度
	* @retval none
  */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	HAL_UART_AbortReceive(huart);
	DYPA02_IRQ(huart, Size);
	VL53L0_UART_IRQ(huart, Size);
	YuTu_RX_IRQ(huart, Size);
	ANO_RX_IRQ(huart, Size);
//	HAL_GPIO_TogglePin(SYS_LED_GPIO_Port, SYS_LED_Pin);
}

/**
  * @brief  串口发送中断回调
  * @param  huart USART设备
	* @retval none
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{

}
