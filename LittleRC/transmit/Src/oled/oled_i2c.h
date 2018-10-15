#ifndef __OLED_I2C_H
#define	__OLED_I2C_H


#define OLED_ADDRESS	0x78 //ͨ������0R����,������0x78��0x7A������ַ -- Ĭ��0x78


void OLED_Init(void);
void OLED_SetPos(unsigned char x, unsigned char y);
void OLED_Fill(unsigned char fill_Data);
void OLED_Cls(void);
void OLED_On(void);
void OLED_Off(void);
void OLED_WriteDat(unsigned char I2C_Data);//д����


#endif
