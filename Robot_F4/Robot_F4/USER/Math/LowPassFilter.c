/*
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-11-30     LSY       从老沈的姿态解析代码中提取出来
 * 2022-08-23			LSY				修复注释错误
 */


#include "LowPassFilter.h"
#include "math.h"

#define M_PI_F 3.14159265358979f                    //圆周率
	
/**
 * @brief 二阶低通滤波器初始化
 *
 * @param lpfData       滤波器地址
 *				sample_freq		计算(采样)频率
 *				cutoff_freq		截至频率
 *
 * @return none
 */
void lpf2pSetCutoffFreq(lpf2pData* lpfData, float sample_freq, float cutoff_freq)
{
	float fr    = sample_freq/cutoff_freq;
	float ohm   = tanf(M_PI_F/fr);
	float c     = 1.0f+2.0f*cosf(M_PI_F/4.0f)*ohm+ohm*ohm;

	lpfData->b0 = ohm*ohm/c;
	lpfData->b1 = 2.0f*lpfData->b0;
	lpfData->b2 = lpfData->b0;
	lpfData->a1 = 2.0f*(ohm*ohm-1.0f)/c;
	lpfData->a2 = (1.0f-2.0f*cosf(M_PI_F/4.0f)*ohm+ohm*ohm)/c;
	lpfData->delay_element_1 = 0.0f;
	lpfData->delay_element_2 = 0.0f;
}

/**
 * @brief 二阶低通滤波
 *
 * @param lpfData       滤波器地址
 *				sample				原始数据（样品）
 *
 * @return output				滤波后的数据
 */
float lpf2pApply(lpf2pData* lpfData, float sample)
{
	float delay_element_0 = sample - lpfData->delay_element_1 * lpfData->a1
                            - lpfData->delay_element_2 * lpfData->a2;
	if (!isfinite(delay_element_0)) 
	{
        //不要让错误的值通过过滤器过滤掉
		delay_element_0 = sample;
	}
	float output = delay_element_0 * lpfData->b0 + lpfData->delay_element_1 * lpfData->b1 + lpfData->delay_element_2 * lpfData->b2;

	lpfData->delay_element_2 = lpfData->delay_element_1;
	lpfData->delay_element_1 = delay_element_0;
    
	return output;
}

