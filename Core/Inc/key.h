/*
 * key.h
 *
 *  Created on: May 15, 2025
 *      Author: zhang
 */

#ifndef INC_KEY_H_
#define INC_KEY_H_

#include "main.h"
#include "gpio.h"

/* 外部变量声明 */
extern uint8_t refresh_flag;    // 数据刷新标志
extern uint8_t current_buffer;  // 当前使用的缓冲区编号

void key_scan(void);

#endif /* INC_KEY_H_ */