#ifndef __OLED_I2C_H
#define	__OLED_I2C_H


#define OLED_ADDRESS	0x78 //通过调整0R电阻,屏可以0x78和0x7A两个地址 -- 默认0x78


void OLED_Init(void);
void OLED_SetPos(unsigned char x, unsigned char y);
void OLED_Fill(unsigned char fill_Data);
void OLED_Cls(void);
void OLED_On(void);
void OLED_Off(void);
void OLED_WriteDat(unsigned char I2C_Data);//写数据


#endif
