# STM32F042

## 破解倒挡
波特率：500K  
CANID: 标准帧 数据帧 0x2E7

|data 2 3|事件|
|---|---|
30 04 |R松刹车
30 06 |R踩刹车
30 00 |D松刹车
30 02 |D刹车
30 00 |N松刹车
30 02 |N刹车
00 00 |P松刹车
00 02 |P刹车

```c
static uint8_t control = 0;
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *handcan)
{
  uint8_t Data[8] = {0};
  HAL_StatusTypeDef HAL_RetVal;
  CAN_RxHeaderTypeDef RxMeg;
  if (handcan == &hcan)
  {
    HAL_RetVal = HAL_CAN_GetRxMessage(handcan, CAN_RX_FIFO0, &RxMeg, Data);
    if (HAL_OK == HAL_RetVal)
    {
      if (RxMeg.StdId == 0x2e7)
      {
        if (Data[2] == 3 && ((Data[3] & 0xFC) == 4))
        {
          control = 1;
        }
        else
        {
          control = 0;
        }
      }
    }
  }
}



HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,control);

```

