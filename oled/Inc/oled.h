/*
 * oled.h
 *
 *  Created on: Mar 16, 2025
 *      Author: zhang
 */

#ifndef INC_OLED_H_
#define INC_OLED_H_

#include"main.h"
#include "i2c.h"
/**
 * @brief 向OLED发送命令
 * @param cmd 要发送的命令
 */
void Oled_WriteCommand(uint8_t cmd);

/**
 * @brief 向OLED发送数据
 * @param data 要发送的数据
 */
void Oled_WriteData(uint8_t data);

/**
 * @brief 初始化OLED显示屏
 */
void OLED_Init(void);

/**
 * @brief 清除OLED显示屏的内容
 */
void OLED_Clear(void);

/**
 * @brief 打开OLED显示屏
 */
void OLED_Display(void);

/**
 * @brief 关闭OLED显示屏
 */
void OLED_EnDisplay(void);

/**
 * @brief 设置OLED显示的光标位置
 * @param x X坐标（列）
 * @param y Y坐标（页）
 */
void OLED_Set_Pos(uint8_t x, uint8_t y);

/**
 * @brief 打开OLED显示
 */
void OLED_On(void);

/**
 * @brief 在OLED上显示数字
 * @param x X坐标（列）
 * @param y Y坐标（页）
 * @param num 要显示的数字
 * @param len 数字的长度
 * @param size2 字体大小
 * @param Color_Turn 是否反色显示（1: 反色, 0: 正常）
 */
void OLED_ShowNum(uint8_t x, uint8_t y, unsigned int num, uint8_t len, uint8_t size2, uint8_t Color_Turn);

/**
 * @brief 在OLED上显示小数
 * @param x X坐标（列）
 * @param y Y坐标（页）
 * @param num 要显示的小数
 * @param z_len 整数部分的长度
 * @param f_len 小数部分的长度
 * @param size2 字体大小
 * @param Color_Turn 是否反色显示（1: 反色, 0: 正常）
 */
void OLED_Showdecimal(uint8_t x, uint8_t y, float num, uint8_t z_len, uint8_t f_len, uint8_t size2, uint8_t Color_Turn);

/**
 * @brief 在OLED上显示单个字符
 * @param x X坐标（列）
 * @param y Y坐标（页）
 * @param chr 要显示的字符
 * @param Char_Size 字体大小
 * @param Color_Turn 是否反色显示（1: 反色, 0: 正常）
 */
void OLED_ShowChar(uint8_t x, uint8_t y, uint8_t chr, uint8_t Char_Size, uint8_t Color_Turn);

/**
 * @brief 在OLED上显示字符串
 * @param x X坐标（列）
 * @param y Y坐标（页）
 * @param chr 要显示的字符串
 * @param Char_Size 字体大小
 * @param Color_Turn 是否反色显示（1: 反色, 0: 正常）
 */
void OLED_ShowString(uint8_t x, uint8_t y, char* chr, uint8_t Char_Size, uint8_t Color_Turn);

/**
 * @brief 在OLED上显示中文字符
 * @param x X坐标（列）
 * @param y Y坐标（页）
 * @param no 中文字符的索引
 * @param Color_Turn 是否反色显示（1: 反色, 0: 正常）
 */
void OLED_ShowCHinese(uint8_t x, uint8_t y, uint8_t no, uint8_t Color_Turn);

/**
 * @brief 在OLED上显示位图
 * @param x0 起始X坐标（列）
 * @param y0 起始Y坐标（页）
 * @param x1 结束X坐标（列）
 * @param y1 结束Y坐标（页）
 * @param BMP 位图数据
 * @param Color_Turn 是否反色显示（1: 反色, 0: 正常）
 */
void OLED_DrawBMP(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint8_t* BMP, uint8_t Color_Turn);

/**
 * @brief 控制OLED的水平滚动
 * @param direction 滚动方向（0: 向左, 1: 向右）
 */
void OLED_HorizontalShift(uint8_t direction);

/**
 * @brief 控制OLED的部分区域水平滚动
 * @param direction 滚动方向（0: 向左, 1: 向右）
 * @param start 起始页
 * @param end 结束页
 */
void OLED_Some_HorizontalShift(uint8_t direction, uint8_t start, uint8_t end);

/**
 * @brief 控制OLED的垂直和水平滚动
 * @param direction 滚动方向（0: 向上, 1: 向下）
 */
void OLED_VerticalAndHorizontalShift(uint8_t direction);

/**
 * @brief 设置OLED的显示模式
 * @param mode 显示模式（0: 正常, 1: 反色）
 */
void OLED_DisplayMode(uint8_t mode);

/**
 * @brief 控制OLED的显示亮度
 * @param intensity 亮度值（0-255）
 */
void OLED_IntensityControl(uint8_t intensity);

#endif /* INC_OLED_H_ */
