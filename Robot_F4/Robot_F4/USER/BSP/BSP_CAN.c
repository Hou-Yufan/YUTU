#include "BSP_CAN.h"
#include "Displayer.h"
#include "RC.h"
#include "CAN_HCSR04.h"
#include "CAN_ADC.h"

CAN_DATA_Config_s can_data[My_DATA_LEN] = {
	{0x00, BROAD_ID_RK, 100, 0, &hcan2, 0},
	{0x01, BROAD_ID_RK, 100, 0, &hcan2, 0},
	{0x02, BROAD_ID_RK, 100, 0, &hcan2, 0},
	{0x03, BROAD_ID_RK, 100, 0, &hcan2, 0},
	{0x04, BROAD_ID_RK, 100, 0, &hcan2, 0},
	{0x05, BROAD_ID_RK, 100, 0, &hcan2, 0},
	{0x06, BROAD_ID_RK, 100, 0, &hcan2, 0},
	{0x07, BROAD_ID_RK, 10, 0, &hcan2, 0},
	{0x08, BROAD_ID_RK, 5, 0, &hcan2, 0},
	{0x09, BROAD_ID_RK, 1, 0, &hcan2, 0},
	{0x0a, BROAD_ID_RK, 1, 0, &hcan2, 0},
	{0x0b, BROAD_ID_DISPLAY, 0, 0, &hcan2, 0},
	{0x0c, BROAD_ID_DISPLAY, 10, 0, &hcan2, Disp_Send_AP_Data}
};

/**
  * @brief          初始化CAN总线过滤器
  * @retval         none
  */
void CAN_Filter_Init(void)
{
	CAN_FilterTypeDef can_filter_st;

	can_filter_st.FilterBank = 0;//过滤器0
	can_filter_st.FilterActivation= ENABLE;
	can_filter_st.FilterFIFOAssignment=CAN_FILTER_FIFO0;//选择邮箱FIFO 0
	
	can_filter_st.FilterIdHigh=0x00;
	can_filter_st.FilterIdLow=0x00;
	can_filter_st.FilterMaskIdHigh=0x00;
	can_filter_st.FilterMaskIdLow=0x00;
	
	can_filter_st.FilterMode=CAN_FILTERMODE_IDMASK;//掩码模式
	can_filter_st.FilterScale=CAN_FILTERSCALE_32BIT;	//32位获取字节

	can_filter_st.FilterBank = 14;				//分配筛选器
	can_filter_st.SlaveStartFilterBank=14	;//分配筛选器
	HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);//csnd
	HAL_CAN_Start(&hcan2);
	//open FIFO 0 message pending interrupt
	HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}

/**
  * @brief          自动上报CAN数据
  * @retval         none
  */
void CAN_Auto_Send_Task(void)
{
	uint16_t i;
	uint32_t tnow;
	for(i=0; i<My_DATA_LEN; ++i){
		//获取系统当前时间，单位MS
		tnow = HAL_GetTick();
		if(can_data[i].Freq/1000 <= tnow - can_data[i].last_time){
			if(can_data[i].send_func != NULL && can_data[i].can_dev != NULL)
					can_data[i].send_func(can_data[i].can_dev, My_CAN_ID, can_data[i].RX_ADDR);
			can_data[i].last_time = tnow;
		}
	}
}


/**
  * @brief          CAN接收中断回调函数
  * @param[in]      hcan: 产生中断的CAN
  * @retval         none
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)//接收回调函数
{
	CAN_RxHeaderTypeDef		rx_message;//定义接收结构体
	uint8_t rx_data[8];
	CAN_ExtId id;
	//回调CAN RX
	if ( HAL_OK == HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_message, rx_data))//获取接收完成标志
	{
		id.ID = rx_message.ExtId;
//		LOG("CAN IRQ id = %d, type = %d, TX = %d, RX = %d\r\n", id.ID, id.ExtId.Type, id.ExtId.TX_ADDR, id.ExtId.RX_ADDR);
		if(id.ExtId.RX_ADDR == My_CAN_ID || id.ExtId.RX_ADDR == BROAD_ID_RK)		//发送给或者3588的数据
		{
			if(id.ExtId.Type & CAN_TYPE_OTA_MARK)			//OTA命令
			{
				
			}
			else if(id.ExtId.Type & CAN_TYPE_CMD_MARK)	//数据控制命令
			{
				switch(id.ExtId.Type){
					case CAN_CMD_SET_FREQ:{									//设置数据上报频率
						if(rx_data[0] < My_DATA_LEN){
							can_data[rx_data[0]].Freq = (uint16_t)rx_data[1]<<8 | rx_data[2];
							if(can_data[rx_data[0]].Freq > 1000)
								can_data[rx_data[0]].Freq = 1000;
						}
						break;
					}
					case CAN_CMD_SLEEP:{											//设置板子电源状态
						break;
					}
					case CAN_CMD_GET_DATA:{
						if(rx_data[0] < My_DATA_LEN){
							if(can_data[rx_data[0]].send_func != NULL)
								can_data[rx_data[0]].send_func(hcan, My_CAN_ID, id.ExtId.TX_ADDR);
						}
						break;
					}
					case CAN_CMD_SET_ADDR:{
						if(rx_data[0] < My_DATA_LEN){
							can_data[rx_data[0]].RX_ADDR = rx_data[1];
						}
						break;
					}
					case CAN_CMD_EX:{
						break;
					}
					default:
						break;
				}
			}
			else	//正常数据
			{
				switch(id.ExtId.TX_ADDR){
					
					case BROAD_ID_RK:{
						switch(id.ExtId.Type){
							case 0:{
								RC_CAN_RX(rx_data);
								LOG("BROAD_ID_RK 0\r\n");
								break;
							}
							default:
								break;
						}
						break;
					}
					
					case BROAD_ID_ADC:{
						switch(id.ExtId.Type){
							case 0:{
								CAN_ADCHub_Data0_RX(rx_data);
								LOG("BROAD_ID_ADC 0\r\n");
								break;
							}
							case 1:{
								CAN_ADCHub_Data1_RX(rx_data);
								LOG("BROAD_ID_ADC 1\r\n");
								break;
							}
							default:
								break;
						}
						break;
					}
					
					case BROAD_ID_DISPLAY:{
						switch(id.ExtId.Type){
							case 0:{
								Disp_RX_Motor_Data(rx_data);
								LOG("BROAD_ID_DISPLAY 0\r\n");
								break;
							}
							case 1:{
								Disp_RX_Sensor_Data(rx_data);
								LOG("BROAD_ID_DISPLAY 1\r\n");
								break;
							}
							case 2:{
								Disp_RX_Regs_Data(rx_data);
								LOG("BROAD_ID_DISPLAY 2\r\n");
								break;
							}
							default:
								break;
						}
						break;
					}
					
					case BROAD_ID_HCSR:{
						CAN_HCSR04_RX(id.ExtId.Type, rx_data);
						LOG("BROAD_ID_HCSR 0\r\n");
						break;
					}
					
					default:
						break;
				}
			}
		}
	}
	//需要编写中断标志位清零
	__HAL_CAN_CLEAR_FLAG(hcan,CAN_IER_FMPIE0);
}

