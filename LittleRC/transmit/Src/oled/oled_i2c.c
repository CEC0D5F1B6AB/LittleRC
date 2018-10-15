
#include "OLED_I2C.h"

#include "stm32f0xx_hal.h"

extern I2C_HandleTypeDef hi2c1;

/**
  * @brief  WriteCmd，向OLED写入命令
  * @param  I2C_Command：命令代码
  * @retval 无
  */
void WriteCmd(unsigned char cmd) //写命令
{
    HAL_I2C_Mem_Write(&hi2c1, OLED_ADDRESS, 0x00, 1, &cmd, 1, 0xff);
}

void OLED_WriteDat(unsigned char data) //写数据
{
    HAL_I2C_Mem_Write(&hi2c1, OLED_ADDRESS, 0x40, 1, &data, 1, 0xff);
}

/**
  * @brief  OLED_Init，初始化OLED
  * @param  无
  * @retval 无
  */
void OLED_Init(void)
{
    //Delay_ms(100); //这里的延时很重要
    HAL_Delay(100);

    WriteCmd(0xAE); //display off
    WriteCmd(0x20); //Set Memory Addressing Mode
    WriteCmd(0x10); //00,Horizontal Addressing Mode;01,Vertical Addressing Mode;10,Page Addressing Mode (RESET);11,Invalid
    WriteCmd(0xb0); //Set Page Start Address for Page Addressing Mode,0-7
    WriteCmd(0xc8); //Set COM Output Scan Direction
    WriteCmd(0x00); //---set low column address
    WriteCmd(0x10); //---set high column address
    WriteCmd(0x40); //--set start line address
    WriteCmd(0x81); //--set contrast control register
    WriteCmd(0xff); //亮度调节 0x00~0xff
    WriteCmd(0xa1); //--set segment re-map 0 to 127
    WriteCmd(0xa6); //--set normal display
    WriteCmd(0xa8); //--set multiplex ratio(1 to 64)
    WriteCmd(0x3F); //
    WriteCmd(0xa4); //0xa4,Output follows RAM content;0xa5,Output ignores RAM content
    WriteCmd(0xd3); //-set display offset
    WriteCmd(0x00); //-not offset
    WriteCmd(0xd5); //--set display clock divide ratio/oscillator frequency
    WriteCmd(0xf0); //--set divide ratio
    WriteCmd(0xd9); //--set pre-charge period
    WriteCmd(0x22); //
    WriteCmd(0xda); //--set com pins hardware configuration
    WriteCmd(0x12);
    WriteCmd(0xdb); //--set vcomh
    WriteCmd(0x20); //0x20,0.77xVcc

		OLED_Cls();
		OLED_On();
}

/**
  * @brief  OLED_SetPos，设置光标
  * @param  x,光标x位置
	*					y，光标y位置
  * @retval 无
  */
void OLED_SetPos(unsigned char x, unsigned char y) //设置起始点坐标
{
    WriteCmd(0xb0 + y);
    WriteCmd(((x & 0xf0) >> 4) | 0x10);
    WriteCmd((x & 0x0f) | 0x01);
}

/**
  * @brief  OLED_Fill，填充整个屏幕
  * @param  fill_Data:要填充的数据
	* @retval 无
  */
void OLED_Fill(unsigned char fill_Data) //全屏填充
{
    unsigned char m, n;
    for (m = 0; m < 8; m++) {
        WriteCmd(0xb0 + m); //page0-page1
        WriteCmd(0x00); //low column start address
        WriteCmd(0x10); //high column start address
        for (n = 0; n < 128; n++) {
            OLED_WriteDat(fill_Data);
        }
    }
}

/**
  * @brief  OLED_CLS，清屏
  * @param  无
	* @retval 无
  */
void OLED_Cls(void) //清屏
{
    OLED_Fill(0x00);
}

/**
  * @brief  OLED_ON，将OLED从休眠中唤醒
  * @param  无
	* @retval 无
  */
void OLED_On(void)
{
    WriteCmd(0X8D); //设置电荷泵
    WriteCmd(0X14); //开启电荷泵
    WriteCmd(0XAF); //OLED唤醒
}

/**
  * @brief  OLED_OFF，让OLED休眠 -- 休眠模式下,OLED功耗不到10uA
  * @param  无
	* @retval 无
  */
void OLED_Off(void)
{
    WriteCmd(0X8D); //设置电荷泵
    WriteCmd(0X10); //关闭电荷泵
    WriteCmd(0XAE); //OLED休眠
}
