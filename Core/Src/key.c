/*
 * key.c
 *
 *  Created on: May 15, 2025
 *      Author: zhang
 */

#include "key.h"



uint8_t key1_state = 0, key2_state = 0, key3_state = 0, key4_state = 0;
uint8_t key[1];

void key_scan(void)
{

    if (HAL_GPIO_ReadPin(GPIOB , GPIO_PIN_12) == 0)
    {
        while (HAL_GPIO_ReadPin(GPIOB , GPIO_PIN_12) == 0)
            if (HAL_GPIO_ReadPin(GPIOB , GPIO_PIN_12) == 0)
            {
                if (key1_state == 0)
                {
                    key[0] = 1;
                    key1_state = 1;
                }
                else
                {
                    key[0] =2;
                    key1_state = 0;
                }
            }
    }

    if (HAL_GPIO_ReadPin(GPIOB , GPIO_PIN_13) == 0)
    {
        while (HAL_GPIO_ReadPin(GPIOB , GPIO_PIN_13) == 0)
            if (HAL_GPIO_ReadPin(GPIOB , GPIO_PIN_13) == 0)
            {
                if (key2_state == 0)
                {
                    key[0] =3;
                    key2_state = 1;
                }
                else
                {
                    key[0] =4;
                    key2_state = 0;
                }
            }
    }

    if (HAL_GPIO_ReadPin(GPIOB , GPIO_PIN_14) == 0)
    {
        while (HAL_GPIO_ReadPin(GPIOB , GPIO_PIN_14) == 0)
            if (HAL_GPIO_ReadPin(GPIOB , GPIO_PIN_14) == 0)
            {
                if (key3_state == 0)
                {
                    key[0] =5;
                    key3_state = 1;
                }
                else
                {
                    key[0] =6;
                    key3_state = 0;
                }
            }
    }

    if (HAL_GPIO_ReadPin(GPIOB , GPIO_PIN_15) == 0)
    {
        while (HAL_GPIO_ReadPin(GPIOB , GPIO_PIN_15) == 0)
            if (HAL_GPIO_ReadPin(GPIOB , GPIO_PIN_15) == 0)
            {
                if (key4_state == 0)
                {
                    key[0] =7;
                    key4_state = 1;
                }
                else
                {
                    key[0] =8;
                    key4_state = 0;
                }
            }
    }
}
