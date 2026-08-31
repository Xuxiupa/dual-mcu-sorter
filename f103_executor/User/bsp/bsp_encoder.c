#include "bsp_encoder.h"
#include "main.h"

extern TIM_HandleTypeDef htim2;

// 硬件参数，改成你实际电机参数
#define ENCODER_LINE        11     //编码器线数
#define REDUCTION_RATIO     30     //减速比
#define SAMPLE_PERIOD_S     0.1f   //采样周期 100ms

void BSP_ENCODER_Init(void)
{
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    __HAL_TIM_SET_COUNTER(&htim2,0);
}

int16_t BSP_ENCODER_GetDelta(void)
{
    int16_t cnt = (int16_t)__HAL_TIM_GET_COUNTER(&htim2);
    __HAL_TIM_SET_COUNTER(&htim2,0);
    return -cnt;   //符号修正，和你之前实验保持一致
}

//delta：100ms采样间隔内脉冲变化
// 每分钟转速 = delta × 60 ÷ (PPR ×4 ×0.1)
float BSP_ENCODER_GetRpm(int16_t delta)
{
    return (float)delta * 60.0f / (ENCODER_PPR * 4.0f * 0.1f);
}
