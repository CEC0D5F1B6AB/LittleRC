#ifndef _LCD_GRAPH_H_
#define _LCD_GRAPH_H_

#include <stdint.h>
#include <stdbool.h>


typedef struct {
	uint8_t x;
	uint8_t y;
} point_t;

typedef struct  {
	uint8_t *	font;
	uint8_t		x_size;
	uint8_t		y_size;
	uint8_t		offset;
	uint8_t		numchars;
	uint8_t		inverted;
}font_t;


#define LCD_CENTER 0xFF

#define LCD_X   128              //液晶屏横坐标宽度
#define LCD_Y   64              //液晶屏纵坐标高度
#define LCD_ROW LCD_X           //液晶屏列宽度
#define LCD_COLUMN  (LCD_Y / 8)    //液晶屏行高度
#define LCD_MID_X	(LCD_X/2-1)
#define LCD_MID_Y	(LCD_Y/2-1)

#define LCD_BLACK   0
#define LCD_WHITE   1

typedef struct {
	uint8_t x;
	uint8_t y;
	uint8_t width;
	uint8_t length;
	uint8_t dir;
	int	minVal;
	int	maxVal;
	int	value;
}processBar_t;

#define PB_VERTICAL 0
#define PB_HORIZON  1

//LCD_bXXXX() 带有"b" 代表操作在buffer里进行

/*-----设置缓冲区指针坐标-------*/
void LCD_bSetXY(uint8_t x, uint8_t y);
/*----将整个缓冲区刷新到5110上-----*/
void LCD_RefreshAll(void);
/*-------将缓冲区指定区域刷新到5110对应区域-------*/
void LCD_Refresh(uint8_t x, uint8_t y, uint8_t width, uint8_t height);
/*------清空整个缓冲区-------*/
void LCD_bClearAll(void);
/*------清空整个缓冲区并刷新-------*/
void LCD_ClearAll(void);
/*-------清空缓冲区指定区域-------*/
void LCD_bClear(uint8_t x, uint8_t y, uint8_t width, uint8_t height);

void LCD_SetFont(uint8_t *font);
uint8_t LCD_GetFontHeight(void);
uint8_t LCD_GetFontWidth(void);
void LCD_InvertFont(bool inv);

/*-------写一个ASCII字符(6*8)到缓冲区(不需要坐标，紧跟上个字符)--------*/
void LCD_bPutChar(char c);
/*-------写一个ASCII字符(6*8)到缓冲区(不需要坐标，紧跟上个字符)--------*/
void LCD_bPutChar(char c);
/*--------通过缓冲区写一个ASCII字符(6*8)到5110(不需要坐标，紧跟上个字符)--------*/
void LCD_PutChar(char c);
/*----写一个ASCII字符(6*8)到缓冲区-----*/
void LCD_bPutCharXY(uint8_t x, uint8_t y, char c);
/*----通过缓冲区写一个ASCII字符(6*8)到5110-----*/
void LCD_PutCharXY(uint8_t x, uint8_t y, char c);
/*------写一个字符串(高度8)到缓冲区(不需要坐标，紧跟上个字符)-----*/
uint8_t LCD_bPutStr(char *str);
/*----通过缓冲区写一个字符串(高度8)到5110(不需要坐标，紧跟上个字符)----*/
uint8_t LCD_PutStr(char *str);
/*-----写一个字符串(高度8)到缓冲区------*/
uint8_t LCD_bPutStrXY(uint8_t x, uint8_t y, char *str);
/*------通过缓冲区写一个字符串(高度8)到5110------*/
uint8_t LCD_PutStrXY(uint8_t x, uint8_t y, char *str);


//*********    以下为画点，线，折线，矩形，圆，bmp图片函数      *********//
/*------画一个点到缓冲区-------*/
void LCD_bDrawPoint(uint8_t x, uint8_t y, uint8_t color);
/*------通过缓冲区画一个点到5110------*/
void LCD_DrawPoint(uint8_t x, uint8_t y, uint8_t color);
/*------画一条线到缓冲区--------*/
void LCD_bDrawLine(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint8_t color);
/*-----通过缓冲区画一条线到5110-------*/
void LCD_DrawLine(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint8_t color);
/*--画折线到缓冲区(p为折线各个节点，line_num为折线的线段数目)--*/
void LCD_bDrawPolyLine(point_t *p, uint8_t line_num, uint8_t color);
/*--通过缓冲区画折线到5110(p为折线各个节点，line_num为折线的线段数目)--*/
void LCD_DrawPolyLine(point_t *p, uint8_t line_num, uint8_t color);
/*---------画矩形到缓冲区--------*/
void LCD_bDrawRect(uint8_t x, uint8_t y, uint8_t width, uint8_t height, uint8_t color);
/*--------通过缓冲区画矩形到5110--------*/
void LCD_DrawRect(uint8_t x, uint8_t y, uint8_t width, uint8_t height, uint8_t color);
/*---------画实心矩形到缓冲区--------*/
void LCD_bDrawFillRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t color);
/*--------通过缓冲区画实心矩形到5110--------*/
void LCD_DrawFillRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t color);
/*--------画圆到缓冲区--------*/
void LCD_bDrawCircle(uint8_t x0, uint8_t y0, uint8_t r, uint8_t color);
/*--------通过缓冲区画圆到5110--------*/
void LCD_DrawCircle(uint8_t x0, uint8_t y0, uint8_t r, uint8_t color);
/*--------画实心圆到缓冲区--------*/
void LCD_bDrawFillCircle(uint8_t x0, uint8_t y0, uint8_t r, uint8_t color);
/*--------通过缓冲区画实心圆到5110--------*/
void LCD_DrawFillCircle(uint8_t x0, uint8_t y0, uint8_t r, uint8_t color);
/*----画一个图片到缓冲区-----*/
void LCD_bDrawBmp(uint8_t x, uint8_t y, uint8_t width, uint8_t height, char bitmap[]);
/*-----通过缓冲区画一个图片到5110------*/
void LCD_DrawBmp(uint8_t x, uint8_t y, uint8_t width, uint8_t height, char bitmap[]);


uint8_t ProcessBar_Create(uint8_t x, uint8_t y, uint8_t width, uint8_t length, uint8_t dir, int minVal, int maxVal, int defVal);
void ProcessBar_SetVal(uint8_t PB_Index, int val);
void ProcessBar_Refresh(int PB_Index);
void ProcessBar_ReConfig(int PB_Index, int minVal, int maxVal, int defVal);

// not complete yet, only scroll 8 pixel
void LCD_bScrollUp(void);
void LCD_ScrollUp(void);

void LCD_LogOut(char *str);
void LCD_LogClear(void);

#endif
