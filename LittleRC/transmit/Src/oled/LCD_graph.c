#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "LCD_graph.h"
#include "OLED_I2C.h"

uint8_t lcd_buf[LCD_COLUMN][LCD_ROW];//LCD_Y/8  LCD_X

point_t buf_ptr;
point_t start_ptr;

font_t cfont;

processBar_t PBPool[10];
int processBarCount = 0;

//移植时候需要修改的函数

/*Set row(0~5) column(0~83) */
void LCD_SetRC(uint8_t row, uint8_t column)
{
	if (row >= LCD_ROW) row = 0;
	if (column >= LCD_COLUMN) column = 0;

	OLED_SetPos(row, column);
}
//LCD_RefreshAll();
//LCD_Refresh


/////////////////////////////////////////////////////////////////////////////////////////////////////
/////*              以下为使用缓冲区写字画图的相关函数               *//////
//*-------------以下坐标统一为两种----------------*//
/////////////////////////////////////////////////////////////////////////////////////////////////////



/*----获得一个8位整型数的绝对值----*/
uint8_t Abs(char a)
{
	if ((uint8_t)a >= 128)
		return 256 - a;
	else
		return a;
}
/*----交换两个8位整型数----*/
void Swap(uint8_t *a, uint8_t *b)
{
	uint8_t temp;

	temp = *a;
	*a = *b;
	*b = temp;
}
/*-----设置缓冲区指针坐标-------*/
void LCD_bSetXY(uint8_t x, uint8_t y)
{
	if (y >= LCD_Y) y = 0;          //纠正错误纵坐标输入
	if (x >= LCD_X) x = 0;          //纠正错误横坐标输入

	buf_ptr.y = y;                  //设置纵坐标
	buf_ptr.x = x;                  //设置横坐标
}
/*----将整个缓冲区刷新到LCD上-----*/
void LCD_RefreshAll()
{
	uint16_t i, j;

	LCD_SetRC(0, 0);
	for (i = 0; i < LCD_COLUMN; i++) {
		for (j = 0; j < LCD_ROW; j++)
			OLED_WriteDat(lcd_buf[i][j]);
	}
}
/*-------将缓冲区指定区域刷新到LCD对应区域-------*/
void LCD_Refresh(uint8_t x, uint8_t y, uint8_t width, uint8_t height)
{
	uint8_t width_i, height_i;
	uint8_t start_row, start_column, end_row, end_column;

	start_row = y >> 3;                     //刷新起始行(8bit 每行)
	start_column = x;                       //刷新起始列
	end_row = (y + height - 1) >> 3;        //刷新终止行
	end_column = x + width - 1;             //刷新终止列

	//刷新第height_i行
	for (height_i = start_row; height_i <= end_row; height_i++) {           //刷新指定行
		LCD_SetRC(start_column, height_i);                                  //从行首开始刷新
		//刷新该行的width列
		for (width_i = start_column; width_i <= end_column; width_i++) {    //刷新每行的指定列
			OLED_WriteDat(lcd_buf[height_i][width_i]);
		}
	}
}

/*----清除缓冲区1字节数据----*/
void Buf_ClearByte(uint8_t x, uint8_t y)
{
	uint8_t column, column_bit;

	LCD_bSetXY(x, y);
	column = buf_ptr.y >> 3;                                                        //要清除的缓冲区的字节行坐标
	column_bit = buf_ptr.y % 8;                                                     //坐标在该字节8bit的bit位置
	lcd_buf[column][buf_ptr.x] &= ~(0x01 << (column_bit));                          //清除buf_ptr指向的缓冲区字节
	if (column_bit) lcd_buf[column + 1][buf_ptr.x] &= ~(0xff >> (8 - column_bit));  //字节跨两行就清除下一行
}
/*------写一个字节数据到缓冲区-------*/
void Buf_WriteByte(uint8_t byte)
{
	uint8_t column, column_bit;

	column = buf_ptr.y >> 3;                                                        //要写的缓冲区的字节行坐标
	column_bit = buf_ptr.y % 8;                                                     //坐标在该字节8bit的bit位置
	lcd_buf[column][buf_ptr.x] |= byte << (column_bit);                             //将1字节数据写入buf_ptr指向的缓冲区字节
	if (column_bit) lcd_buf[column + 1][buf_ptr.x] |= byte >> (8 - column_bit);     //字节跨两行就写下一行
	if (buf_ptr.x >= LCD_X - 1) {                                                   //到达行尾缓冲区指针自动跳转到下一行行首
		buf_ptr.x = 0;
		buf_ptr.y += 8;
		if (buf_ptr.y >= LCD_Y)       //到达页面末尾，缓冲区指针跳到页面开头
			buf_ptr.y = 0;
	} else {
		buf_ptr.x++;                                //缓冲区指针指向下一列
	}
}
/*------清空整个缓冲区-------*/
void LCD_bClearAll(void)
{
	uint8_t i, j;
	uint8_t y = 0;

	for (j = 0; j < LCD_COLUMN; j++) {
		for (i = 0; i < LCD_ROW; i++)
			lcd_buf[j][i] = 0x00; //写0到缓冲区以清除该字节数据
		y += 8;
	}
}
/*------清空整个缓冲区并刷新-------*/
void LCD_ClearAll(void)
{
	LCD_bClearAll();
	OLED_Fill(0x00);
	LCD_RefreshAll();
}
/*-------清空缓冲区指定区域-------*/
void LCD_bClear(uint8_t x, uint8_t y, uint8_t width, uint8_t height)
{
	uint8_t i, j;

	height = y + height;                            //获得刷新的终止纵坐标
	width = width + x;                              //获得刷新的终止横坐标
	for (j = y; j < height; j++)
		for (i = x; i < width; i++)
			Buf_ClearByte(i, j);          //写0到缓冲区以清除该字节数据
}


void LCD_SetFont(uint8_t *font)
{
	cfont.font = font;
	cfont.x_size = cfont.font[0];
	cfont.y_size = cfont.font[1];
	cfont.offset = cfont.font[2];
	cfont.numchars = cfont.font[3];
	cfont.inverted = 0;
}

//获取当前字体高度
uint8_t LCD_GetFontHeight(void)
{
	return cfont.y_size;
}

//获取当前字体宽度
uint8_t LCD_GetFontWidth(void)
{
	return cfont.x_size;
}

void LCD_InvertFont(bool inv)
{
	if (inv)
		cfont.inverted = true;
	else
		cfont.inverted = false;
}

void LCD_bPutChar(char c)
{
	int color;

	//	if ((buf_ptr.x >= LCD_X) || (buf_ptr.y >= LCD_Y))
	//		return; //TODO: make it clear if auto warp is suitable, or use none wrap instead

	if ((buf_ptr.x + cfont.x_size > LCD_X) || (buf_ptr.y >= LCD_Y)) {
		return;//drop the char whitch not enough space to display
	}

	if (cfont.inverted == true)
		color = LCD_WHITE;
	else
		color = LCD_BLACK;

	if ((cfont.y_size % 8) == 0) {
		int font_idx = ((c - cfont.offset) * (cfont.x_size * (cfont.y_size / 8))) + 4;
		int rowcnt;
		int cnt;
		int b;
		for (rowcnt = 0; rowcnt < (cfont.y_size / 8); rowcnt++) {
			for (cnt = 0; cnt < cfont.x_size; cnt++) {
				for (b = 0; b < 8; b++) {
					if ((cfont.font[font_idx + cnt + (rowcnt * cfont.x_size)] & (1 << b)) != 0)
						LCD_bDrawPoint(buf_ptr.x + cnt, buf_ptr.y + (rowcnt * 8) + b, color);
					else
						LCD_bDrawPoint(buf_ptr.x + cnt, buf_ptr.y + (rowcnt * 8) + b, !color);
				}
			}
		}
	} else {
		int font_idx = ((c - cfont.offset) * ((cfont.x_size * cfont.y_size / 8))) + 4;
		int cbyte = cfont.font[font_idx];
		int cbit = 7;
		int cx;
		int cy;
		for (cx = 0; cx < cfont.x_size; cx++) {
			for (cy = 0; cy < cfont.y_size; cy++) {
				if ((cbyte & (1 << cbit)) != 0)
					LCD_bDrawPoint(buf_ptr.x + cx, buf_ptr.y + cy, color);
				else
					LCD_bDrawPoint(buf_ptr.x + cx, buf_ptr.y + cy, !color);
				cbit--;
				if (cbit < 0) {
					cbit = 7;
					font_idx++;
					cbyte = cfont.font[font_idx];
				}
			}
		}
	}

	start_ptr.x = buf_ptr.x;
	start_ptr.y = buf_ptr.y;
	buf_ptr.x += cfont.x_size;
}

void LCD_PutChar(char c)
{
	LCD_bPutChar(c);
	LCD_Refresh(start_ptr.x, start_ptr.y, cfont.x_size, cfont.y_size);
}
/*----写一个ASCII字符到缓冲区-----*/
void LCD_bPutCharXY(uint8_t x, uint8_t y, char c)
{
	LCD_bSetXY(x, y);                                    //设置打印坐标
	LCD_bPutChar(c);
}
/*----通过缓冲区写一个ASCII字符到LCD-----*/
void LCD_PutCharXY(uint8_t x, uint8_t y, char c)
{
	LCD_bPutCharXY(x, y, c);
	LCD_Refresh(start_ptr.x, start_ptr.y, cfont.x_size, cfont.y_size);
}
/*------写一个字符串到缓冲区(不需要坐标，紧跟上个字符)-----*/
uint8_t LCD_bPutStr(char *str)
{
	uint8_t str_len = 0;

	while (*str) {
		LCD_bPutChar(*str++);
		str_len++;
	}
	return str_len;
}
/*----通过缓冲区写一个字符串到LCD(不需要坐标，紧跟上个字符)----*/
uint8_t LCD_PutStr(char *str)
{
	uint8_t str_len;

	str_len = LCD_bPutStr(str);
	LCD_Refresh(0, start_ptr.y, LCD_X, cfont.y_size);
	return str_len;
}
/*-----写一个字符串到缓冲区------*/
uint8_t LCD_bPutStrXY(uint8_t x, uint8_t y, char *str)
{
	if (x == LCD_CENTER)
		x = (LCD_X - cfont.x_size * strlen(str)) / 2;

	LCD_bSetXY(x, y);
	return LCD_bPutStr(str); //返回字符串长度
}
/*------通过缓冲区写一个字符串到LCD------*/
uint8_t LCD_PutStrXY(uint8_t x, uint8_t y, char *str)
{
	uint8_t str_len;

	if (x == LCD_CENTER)
		x = (LCD_X - cfont.x_size * strlen(str)) / 2;

	str_len = LCD_bPutStrXY(x, y, str);
	LCD_Refresh(0, start_ptr.y, LCD_X, cfont.y_size);
	return str_len; //返回字符串长度
}



//*********    以下为画点，线，折线，矩形，圆，bmp图片函数      *********//

/*------画一个点到缓冲区-------*/
void LCD_bDrawPoint(uint8_t x, uint8_t y, uint8_t color)
{
	uint8_t row;

	if ((x < LCD_X) && (y < LCD_Y)) {
		row = y >> 3;                               //获得点所在的行
		if (color == LCD_BLACK)
			lcd_buf[row][x] |= 0x01 << (y % 8);     //LCD_BLACK point
		else
			lcd_buf[row][x] &= ~(0x01 << (y % 8));  //LCD_WHITE point
	}
}
/*------通过缓冲区画一个点到LCD------*/
void LCD_DrawPoint(uint8_t x, uint8_t y, uint8_t color)
{
	LCD_bDrawPoint(x, y, color);
	LCD_Refresh(x, y, 1, 8);
}
/*-----获得两数中较小的数------*/
uint8_t Min(uint8_t a, uint8_t b)
{
	return a <= b ? a : b;
}
/*------获得画线的单位增加大小------*/
short GetUnitAdd(uint8_t x1, uint8_t x2)
{
	if (x1 == x2)
		return 0;
	else if (x2 > x1)
		return 1;
	else
		return -1;
}
/*------画一条线到缓冲区--------*/
void LCD_bDrawLine(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint8_t color)
{
	uint8_t steep = Abs(y1 - y0) > Abs(x1 - x0);
	uint8_t dx, dy;
	int8_t err;
	int8_t ystep;

	if (steep) {
		Swap(&x0, &y0);
		Swap(&x1, &y1);
	}

	if (x0 > x1) {
		Swap(&x0, &x1);
		Swap(&y0, &y1);
	}

	dx = x1 - x0;
	dy = Abs(y1 - y0);

	err = dx / 2;

	if (y0 < y1)
		ystep = 1;
	else
		ystep = -1;

	for (; x0 <= x1; x0++) {
		if (steep)
			LCD_bDrawPoint(y0, x0, color);
		else
			LCD_bDrawPoint(x0, y0, color);
		err -= dy;
		if (err < 0) {
			y0 += ystep;
			err += dx;
		}
	}
}
/*-----通过缓冲区画一条线到LCD-------*/
void LCD_DrawLine(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint8_t color)
{
	uint8_t x, y, width, height;

	width = Abs(x1 - x0) + 1;       //获得刷新宽度
	height = Abs(y1 - y0) + 1;      //获得刷新高度
	x = Min(x0, x1);                //获得刷新起始横坐标
	y = Min(y0, y1);                //获得刷新起始纵坐标
	LCD_bDrawLine(x0, y0, x1, y1, color);
	LCD_Refresh(x, y, width, height);
}
/*--画折线到缓冲区(p为折线各个节点，line_num为折线的线段数目)--*/
void LCD_bDrawPolyLine(point_t *p, uint8_t line_num, uint8_t color)
{
	uint8_t i;

	for (i = 0; i < line_num; i++)
		LCD_DrawLine(p[i].x, p[i].y, p[i + 1].x, p[i + 1].y, color);
}
/*--通过缓冲区画折线到LCD(p为折线各个节点，line_num为折线的线段数目)--*/
void LCD_DrawPolyLine(point_t *p, uint8_t line_num, uint8_t color)
{
	LCD_bDrawPolyLine(p, line_num, color);
	LCD_RefreshAll();
}
/*---------画矩形到缓冲区--------*/
void LCD_bDrawRect(uint8_t x, uint8_t y, uint8_t width, uint8_t height, uint8_t color)
{
	if ((height == 0) || (width == 0))
		return;
	height--;
	width--;
	LCD_bDrawLine(x, y, x + width, y, color);
	LCD_bDrawLine(x + width, y, x + width, y + height, color);
	LCD_bDrawLine(x, y, x, y + height, color);
	LCD_bDrawLine(x, y + height, x + width, y + height, color);
}
/*--------通过缓冲区画矩形到LCD--------*/
void LCD_DrawRect(uint8_t x, uint8_t y, uint8_t width, uint8_t height, uint8_t color)
{
	LCD_bDrawRect(x, y, width, height, color);
	LCD_Refresh(x, y, width + 1, height + 1);
}
/*---------画实心矩形到缓冲区--------*/
void LCD_bDrawFillRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t color)
{
	int16_t i;

	for (i = x; i < x + w; i++) {
		int16_t j;
		for (j = y; j < y + h; j++)
			LCD_bDrawPoint(i, j, color);
	}
}
/*--------通过缓冲区画实心矩形到LCD--------*/
void LCD_DrawFillRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t color)
{
	LCD_bDrawFillRect(x, y, w, h, color);
	LCD_Refresh(x, y, x + w - 1, y + h - 1);
}

/*--------画圆到缓冲区--------*/
void LCD_bDrawCircle(uint8_t x0, uint8_t y0, uint8_t r, uint8_t color)
{
	int8_t f = 1 - r;
	int8_t ddF_x = 1;
	int8_t ddF_y = -2 * r;
	int8_t x = 0;
	int8_t y = r;


	LCD_bDrawPoint(x0, y0 + r, color);
	LCD_bDrawPoint(x0, y0 - r, color);
	LCD_bDrawPoint(x0 + r, y0, color);
	LCD_bDrawPoint(x0 - r, y0, color);

	while (x < y) {
		if (f >= 0) {
			y--;
			ddF_y += 2;
			f += ddF_y;
		}
		x++;
		ddF_x += 2;
		f += ddF_x;

		LCD_bDrawPoint(x0 + x, y0 + y, color);
		LCD_bDrawPoint(x0 - x, y0 + y, color);
		LCD_bDrawPoint(x0 + x, y0 - y, color);
		LCD_bDrawPoint(x0 - x, y0 - y, color);

		LCD_bDrawPoint(x0 + y, y0 + x, color);
		LCD_bDrawPoint(x0 - y, y0 + x, color);
		LCD_bDrawPoint(x0 + y, y0 - x, color);
		LCD_bDrawPoint(x0 - y, y0 - x, color);
	}
}
/*--------通过缓冲区画圆到LCD--------*/
void LCD_DrawCircle(uint8_t x0, uint8_t y0, uint8_t r, uint8_t color)
{
	LCD_bDrawCircle(x0, y0, r, color);
	LCD_Refresh(x0 - r, y0 - r, x0 + r, y0 + r);
}
/*--------画实心圆到缓冲区--------*/
void LCD_bDrawFillCircle(uint8_t x0, uint8_t y0, uint8_t r, uint8_t color)
{
	int8_t f = 1 - r;
	int8_t ddF_x = 1;
	int8_t ddF_y = -2 * r;
	int8_t x = 0;
	int8_t y = r;

	int16_t i;

	for (i = y0 - r; i <= y0 + r; i++)
		LCD_bDrawPoint(x0, i, color);

	while (x < y) {
		if (f >= 0) {
			y--;
			ddF_y += 2;
			f += ddF_y;
		}
		x++;
		ddF_x += 2;
		f += ddF_x;

		for (i = y0 - y; i <= y0 + y; i++) {
			LCD_bDrawPoint(x0 + x, i, color);
			LCD_bDrawPoint(x0 - x, i, color);
		}
		for (i = y0 - x; i <= y0 + x; i++) {
			LCD_bDrawPoint(x0 + y, i, color);
			LCD_bDrawPoint(x0 - y, i, color);
		}
	}
}
/*--------通过缓冲区画实心圆到LCD--------*/
void LCD_DrawFillCircle(uint8_t x0, uint8_t y0, uint8_t r, uint8_t color)
{
	LCD_bDrawFillCircle(x0, y0, r, color);
	LCD_Refresh(x0 - r, y0 - r, x0 + r, y0 + r);
}


/*----画一个图片到缓冲区-----*/
void LCD_bDrawBmp(uint8_t x, uint8_t y, uint8_t width, uint8_t height, char bitmap[])
{
	uint16_t p = 0;
	uint8_t width_i, height_i;

	height = ((height - 1) >> 3) + 1;

	for (height_i = 0; height_i < height; height_i++) { //画第height_i行数据
		LCD_bSetXY(x, y + (height_i << 3));             //设置缓冲区指针指到行首
		for (width_i = 0; width_i < width; width_i++)   //画第height_i行的第width_i列的1字节数据

			Buf_WriteByte(bitmap[p++]);
	}
}
/*-----通过缓冲区画一个图片到LCD------*/
void LCD_DrawBmp(uint8_t x, uint8_t y, uint8_t width, uint8_t height, char bitmap[])
{
	LCD_bDrawBmp(x, y, width, height, bitmap);
	LCD_Refresh(x, y, width, height);
}


uint8_t ProcessBar_Create(uint8_t x, uint8_t y, uint8_t width, uint8_t length, uint8_t dir, int minVal, int maxVal, int defVal)
{
	processBar_t *newPB;
	uint8_t PB_Index;

	newPB = &PBPool[processBarCount];
	newPB->x = x;
	newPB->y = y;
	newPB->dir = dir;
	newPB->width = width;
	newPB->length = length;
	newPB->minVal = minVal;
	newPB->maxVal = maxVal;
	newPB->value = defVal;

	if (dir == PB_VERTICAL)
		LCD_DrawRect(x, y, width, length, LCD_BLACK);
	else
		LCD_DrawRect(x, y, length, width, LCD_BLACK);

	ProcessBar_SetVal(processBarCount, defVal);
	processBarCount++;

	return PB_Index;
}

void ProcessBar_SetVal(uint8_t PB_Index, int val)
{
	processBar_t *PB;

	PB = &PBPool[PB_Index];

	if (val < PB->minVal)
		PB->value = PB->minVal;
	else if (val > PB->maxVal)
		PB->value = PB->minVal;
	else PB->value = val;

	if (PB->dir == PB_HORIZON)
		LCD_bDrawRect(PB->x + 1, PB->y + 1, PB->length - 2, PB->width - 2, LCD_WHITE);
	else
		LCD_bDrawRect(PB->x + 1, PB->y + 1, PB->width - 2, PB->length - 2, LCD_WHITE);

	if (PB->dir == PB_HORIZON) {
		LCD_bDrawRect(PB->x + 1,
		              PB->y + 1,
		              (PB->length - 2) * (PB->value - PB->minVal) / (PB->maxVal - PB->minVal),
		              (PB->width - 2), LCD_BLACK);
	} else {
		LCD_bDrawRect(PB->x + 1,
		              PB->y + PB->length - 1 - (PB->length - 2) * (PB->value - PB->minVal) / (PB->maxVal - PB->minVal),
		              (PB->width - 2),
		              (PB->length - 2) * (PB->value - PB->minVal) / (PB->maxVal - PB->minVal),
		              LCD_BLACK);
	}
	LCD_RefreshAll();
}

void ProcessBar_ReConfig(int PB_Index, int minVal, int maxVal, int defVal)
{
	processBar_t *PB;

	PB = &PBPool[PB_Index];
	PB->minVal = minVal;
	PB->maxVal = maxVal;
	PB->value = defVal;
}

void ProcessBar_Refresh(int PB_Index)
{
	processBar_t *PB;

	PB = &PBPool[PB_Index];
	if (PB->dir == PB_VERTICAL)
		LCD_DrawRect(PB->x, PB->y, PB->width, PB->length, LCD_BLACK);
	else
		LCD_DrawRect(PB->x, PB->y, PB->length, PB->width, LCD_BLACK);

	ProcessBar_SetVal(PB_Index, PB->value);
}

void LCD_bScrollUp()// not complete yet, only scroll 8 pixel
{
	for (int col = 0; col < LCD_COLUMN - 1; col++) {
		for (int row = 0; row < LCD_ROW; row++) {
			lcd_buf[col][row] = lcd_buf[col + 1][row];
		}
	}

	for (int row = 0; row < LCD_ROW; row++) {
		lcd_buf[LCD_COLUMN - 1][row] = 0X00;
	}
}

void LCD_ScrollUp()// not complete yet, only scroll 8 pixel
{
	LCD_bScrollUp();
	LCD_RefreshAll();
}

int screen_x = 0;
int screen_y = 0;

void LCD_LogOut(char *str)
{
#define __NEWLINE() do{						\
	screen_x = 0;							\
	if (screen_y + cfont.y_size >= LCD_Y) {	\
		LCD_ScrollUp();						\
	} else {								\
		screen_y += cfont.y_size;			\
	}										\
	LCD_bSetXY(screen_x, screen_y);			\
}while(0)

	LCD_bSetXY(screen_x, screen_y);
	while (*str) {
		if (*str == '\r'){
			str++;// ignore this shit
		}else if (*str == '\n'){
			__NEWLINE();
			str++;
		} else if (screen_x + cfont.x_size < LCD_X) {
			LCD_bPutChar(*str);
			screen_x += cfont.x_size;
			str++;
		} else {// wrap the text
			__NEWLINE();
			LCD_bPutChar(*str);
			screen_x += cfont.x_size;
			str++;
		}
	}

	LCD_RefreshAll();
}
void LCD_LogClear()
{
	screen_x = 0;
	screen_y = 0;
	LCD_ClearAll();
}
