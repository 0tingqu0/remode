/*
 * oled.c
 *
 *  Created on: Mar 16, 2025
 *      Author: zhang
 */

/*
 *本驱动文件仅适配HAL库版本
 */
#include "oled.h"       //声明
#include "Font.h"    //字库文件


#define OLED_I2C_ADDRESS 0x3c
#define OLED_CMD        0x00
#define OLED_DATA          0x40

 uint8_t init_cmds[] = {
        0xAE, // 关闭显示
        0xD5, 0x80, // 设置显示时钟分频
        0xA8, 0x3F, // 设置多路复用比率
        0xD3, 0x00, // 设置显示偏移
        0x40, // 设置显示起始行
        0xA1, // 设置段重映射（正常方向）
        0xC8, // 设置 COM 扫描方向（正常方向）
        0xDA, 0x12, // 设置 COM 引脚硬件配置
        0x81, 0xCF, // 设置对比度
        0xD9, 0xF1, // 设置预充电周期
        0xDB, 0x30, // 设置 VCOMH 电压
        0xA4, // 设置整个显示开启
        0xA6, // 设置正常显示（非反色）
        0x8D, 0x14, // 设置电荷泵
        0xAF // 开启显示
    };
 /**
  * @function: void OLED_Init(void)
  * @description: OLED初始化
  * @return {*}
  */
 void OLED_Init(void)
 {

     uint8_t i = 0;
     for(i=0; i<23; i++)
     {
         Oled_WriteCommand(init_cmds[i]);
     }
     OLED_Clear();
 }

 /**
  * @function: void OLED_WR_CMD(uint8_t cmd)
  * @description: 向设备写控制命令
  * @param {uint8_t} cmd 芯片手册规定的命令
  * @return {*}
  */
 void Oled_WriteCommand(uint8_t cmd)
 {
     HAL_I2C_WriteCommand(&hi2c1, OLED_I2C_ADDRESS, OLED_CMD, cmd);
 }

 /**
  * @function: void OLED_WR_DATA(uint8_t data)
  * @description: 向设备写控制数据
  * @param {uint8_t} data 数据
  * @return {*}
  */
 void Oled_WriteData(uint8_t data)
 {
     HAL_I2C_WriteData(&hi2c1 ,OLED_I2C_ADDRESS,OLED_DATA,&data,1);
 }

 /**
  * @function: void OLED_On(void)
  * @description: 更新显示

  * @return {*}
  */
 void OLED_On(void)
 {
     uint8_t i,n;
     for(i=0;i<8;i++)
     {
         Oled_WriteCommand(0xb0+i);    //设置页地址（0~7）
         Oled_WriteCommand(0x00);      //设置显示位置—列低地址
         Oled_WriteCommand (0x10);      //设置显示位置—列高地址
         for(n=0;n<128;n++)
             Oled_WriteData(1);
     }
 }


 /**
  * @function: OLED_Clear(void)
  * @description: 清屏,整个屏幕是黑色的!和没点亮一样!!!
  * @return {*}
  */
 void OLED_Clear(void)
 {
     uint8_t i,n;
     for(i=0;i<8;i++)
     {
         Oled_WriteCommand (0xb0+i);    //设置页地址（0~7）
         Oled_WriteCommand(0x00);      //设置显示位置—列低地址
         Oled_WriteCommand(0x10);      //设置显示位置—列高地址
         Oled_WriteCommand(0x2e);
         for(n=0;n<128;n++)
             Oled_WriteData(0);
     }
 }

 /**
  * @function: void OLED_Display_On(void)
  * @description: 开启OLED显示
  * @return {*}
  */
 void OLED_Display(void)
 {
     Oled_WriteCommand(0X8D);  //SET DCDC命令
     Oled_WriteCommand(0X14);  //DCDC ON
     Oled_WriteCommand(0XAF);  //DISPLAY ON,打开显示
 }


 /**
  * @function: void OLED_Display_Off(void)
  * @description: 关闭OLED显示
  * @return {*}
  */
 void OLED_EnDisplay(void)
 {
     Oled_WriteCommand(0X8D);  //SET DCDC命令
     Oled_WriteCommand(0X10);  //DCDC OFF
     Oled_WriteCommand(0XAE);  //DISPLAY OFF，关闭显示
 }

 /**
  * @function: void OLED_Set_Pos(uint8_t x, uint8_t y)
  * @description: 坐标设置
  * @param {uint8_t} x,y
  * @return {*}
  */
 void OLED_Set_Pos(uint8_t x, uint8_t y)
 {
     Oled_WriteCommand(0xb0+y);    //设置页地址（0~7）
     Oled_WriteCommand(((x&0xf0)>>4)|0x10); //设置显示位置—列高地址
     Oled_WriteCommand(x&0x0f);    //设置显示位置—列低地址
 }


 /**
  * @function: unsigned int oled_pow(uint8_t m,uint8_t n)
  * @description: m^n函数
  * @param {uint8_t} m,n
  * @return {unsigned int} result
  */
 unsigned int oled_pow(uint8_t m,uint8_t n)
 {
     unsigned int result=1;
     while(n--)result*=m;
     return result;
 }

 /**
  * @function: void OLED_ShowChar(uint8_t x, uint8_t y, uint8_t chr, uint8_t Char_Size,uint8_t Color_Turn)
  * @description: 在OLED12864特定位置开始显示一个字符
  * @param {uint8_t} x字符开始显示的横坐标
  * @param {uint8_t} y字符开始显示的纵坐标
  * @param {uint8_t} chr待显示的字符
  * @param {uint8_t} Char_Size待显示字符的字体大小,选择字体 16/12
  * @param {uint8_t} Color_Turn是否反相显示(1反相、0不反相)
  * @return {*}
  */
 void OLED_ShowChar(uint8_t x,uint8_t y,uint8_t chr,uint8_t Char_Size,uint8_t Color_Turn)
 {
     unsigned char c=0,i=0;
         c=chr-' ';//得到偏移后的值
         if(x>128-1){x=0;y=y+2;}
         if(Char_Size ==16)
         {
             OLED_Set_Pos(x,y);
             for(i=0;i<8;i++)
                 {
                   if(Color_Turn)
                       Oled_WriteData(~F8X16[c*16+i]);
                   else
                       Oled_WriteData(F8X16[c*16+i]);
                 }
             OLED_Set_Pos(x,y+1);
             for(i=0;i<8;i++)
                 {
                   if(Color_Turn)
                       Oled_WriteData(~F8X16[c*16+i+8]);
                   else
                       Oled_WriteData(F8X16[c*16+i+8]);
                 }

             }
          else
          {
                 OLED_Set_Pos(x,y);
                 for(i=0;i<6;i++)
                 {
                   if(Color_Turn)
                       Oled_WriteData(~F6x8[c][i]);
                   else
                       Oled_WriteData(F6x8[c][i]);
                 }
           }
 }

 /**
  * @function: void OLED_ShowString(uint8_t x, uint8_t y, uint8_t *chr, uint8_tChar_Size, uint8_t Color_Turn)
  * @description: 在OLED12864特定位置开始显示字符串
  * @param {uint8_t} x待显示字符串的开始横坐标x:0~127
  * @param {uint8_t} y待显示字符串的开始纵坐标 y:0~7，若选择字体大小为16，则两行数字之间需要间隔2，若选择字体大小为12，间隔1
  * @param {uint8_t} *chr待显示的字符串
  * @param {uint8_t} Char_Size待显示字符串的字体大小,选择字体 16/12，16为8X16，12为6x8
  * @param {uint8_t} Color_Turn是否反相显示(1反相、0不反相)
  * @return {*}
  */
 void OLED_ShowString(uint8_t x,uint8_t y,char*chr,uint8_t Char_Size, uint8_t Color_Turn)
 {
     uint8_t  j=0;
     while (chr[j]!='\0')
     {       OLED_ShowChar(x,y,chr[j],Char_Size, Color_Turn);
             if (Char_Size == 12) //6X8的字体列加6，显示下一个字符
                 x += 6;
             else  //8X16的字体列加8，显示下一个字符
                 x += 8;

             if (x > 122 && Char_Size==12) //TextSize6x8如果一行不够显示了，从下一行继续显示
             {
                 x = 0;
                 y++;
             }
             if (x > 120 && Char_Size== 16) //TextSize8x16如果一行不够显示了，从下一行继续显示
             {
                 x = 0;
                 y++;
             }
             j++;
     }
 }

 /**
  * @function: void OLED_ShowNum(uint8_t x,uint8_t y,unsigned int num,uint8_t len,uint8_t size2, Color_Turn)
  * @description: 显示数字
  * @param {uint8_t} x待显示的数字起始横坐标,x:0~126
  * @param {uint8_t} y待显示的数字起始纵坐标, y:0~7，若选择字体大小为16，则两行数字之间需要间隔2，若选择字体大小为12，间隔1
  * @param {unsigned int} num:输入的数据
  * @param {uint8_t } len:输入的数据位数
  * @param {uint8_t} size2:输入的数据大小，选择 16/12，16为8X16，12为6x8
  * @param {uint8_t} Color_Turn是否反相显示(1反相、0不反相)
  * @return {*}
  */
 void OLED_ShowNum(uint8_t x,uint8_t y,unsigned int num,uint8_t len,uint8_t size2, uint8_t Color_Turn)
 {
     uint8_t t,temp;
     uint8_t enshow=0;
     for(t=0;t<len;t++)
     {
         temp=(num/oled_pow(10,len-t-1))%10;
         if(enshow==0&&t<(len-1))
         {
             if(temp==0)
             {
                 OLED_ShowChar(x+(size2/2)*t,y,' ',size2, Color_Turn);
                 continue;
             }else enshow=1;

         }
         OLED_ShowChar(x+(size2/2)*t,y,temp+'0',size2, Color_Turn);
     }
 }


 /**
  * @function: void OLED_Showdecimal(uint8_t x,uint8_t y,float num,uint8_t z_len,uint8_t f_len,uint8_t size2, uint8_t Color_Turn)
  * @description: 显示正负浮点数
  * @param {uint8_t} x待显示的数字起始横坐标,x:0~126
  * @param {uint8_t} y待显示的数字起始纵坐标, y:0~7，若选择字体大小为16，则两行数字之间需要间隔2，若选择字体大小为12，间隔1
  * @param {float} num:输入的浮点型数据
  * @param {uint8_t } z_ len:整数部分的位数
  * @param {uint8_t } f_len: 小数部分的位数
  * @param {uint8_t} size2:输入的数据大小，选择 16/12，16为8X16，12为6x8
  * @param {uint8_t} Color_Turn是否反相显示(1反相、0不反相)
  * @return {*}
  */
 void OLED_Showdecimal(uint8_t x,uint8_t y,float num,uint8_t z_len,uint8_t f_len,uint8_t size2, uint8_t Color_Turn)
 {
     uint8_t t,temp,i=0;//i为负数标志位
     uint8_t enshow;
     int z_temp,f_temp;
     if(num<0)
     {
         z_len+=1;
         i=1;
         num=-num;
     }
     z_temp=(int)num;
     //整数部分
     for(t=0;t<z_len;t++)
     {
         temp=(z_temp/oled_pow(10,z_len-t-1))%10;
         if(enshow==0 && t<(z_len-1))
         {
             if(temp==0)
             {
                 OLED_ShowChar(x+(size2/2)*t,y,' ',size2, Color_Turn);
                 continue;
             }
             else
             enshow=1;
         }
         OLED_ShowChar(x+(size2/2)*t,y,temp+'0',size2, Color_Turn);
     }
     //小数点
     OLED_ShowChar(x+(size2/2)*(z_len),y,'.',size2, Color_Turn);

     f_temp=(int)((num-z_temp)*(oled_pow(10,f_len)));
   //小数部分
     for(t=0;t<f_len;t++)
     {
         temp=(f_temp/oled_pow(10,f_len-t-1))%10;
         OLED_ShowChar(x+(size2/2)*(t+z_len)+5,y,temp+'0',size2, Color_Turn);
     }
     if(i==1)//如果为负，就将最前的一位赋值‘-’
     {
         OLED_ShowChar(x,y,'-',size2, Color_Turn);
         i=0;
     }
 }



 /**
  * @function: void OLED_ShowCHinese(uint8_t x,uint8_t y,uint8_t no, uint8_t Color_Turn)
  * @description: 在OLED特定位置开始显示16X16汉字
  * @param {uint8_t} x待显示的汉字起始横坐标x: 0~112，两列汉字之间需要间隔16
  * @param {uint8_t} y待显示的汉字起始纵坐标 y: 0~6 , 两行汉字之间需要间隔2
  * @param {uint8_t} no待显示的汉字编号
  * @param {uint8_t} Color_Turn是否反相显示(1反相、0不反相)
  * @return {*}
  */
 void OLED_ShowCHinese(uint8_t x,uint8_t y,uint8_t no, uint8_t Color_Turn)
 {
     uint8_t t=0;
     OLED_Set_Pos(x,y);
     for(t=0;t<16;t++)
         {
                 if (Color_Turn)
                     Oled_WriteData(~Hzk[2*no][t]); //显示汉字的上半部分
                 else
                     Oled_WriteData(Hzk[2*no][t]); //显示汉字的上半部分
         }

         OLED_Set_Pos(x,y+1);
     for(t=0;t<16;t++)
         {
                 if (Color_Turn)
                     Oled_WriteData(~Hzk[2*no+1][t]); //显示汉字的上半部分
                 else
                     Oled_WriteData(Hzk[2*no+1][t]);//显示汉字的上半部分

          }
 }

 /**
  * @function: void OLED_DrawBMP(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint8_t *  BMP,uint8_t Color_Turn)
  * @description: 在OLED特定区域显示BMP图片
  * @param {uint8_t} x0图像开始显示横坐标  x0:0~127
  * @param {uint8_t} y0图像开始显示纵坐标  y0:0~7
  * @param {uint8_t} x1图像结束显示横坐标  x1:1~128
  * @param {uint8_t} y1图像结束显示纵坐标  y1:1~8
  * @param {uint8_t} *BMP待显示的图像数据
  * @param {uint8_t} Color_Turn是否反相显示(1反相、0不反相)
  * @return {*}
  */
 void OLED_DrawBMP(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint8_t *  BMP,uint8_t Color_Turn)
 {
    uint32_t j = 0;
    uint8_t x = 0, y = 0;

   if(y1%8==0)
         y = y1/8;
   else
         y = y1/8 + 1;
     for(y=y0;y<y1;y++)
     {
         OLED_Set_Pos(x0,y);
     for(x=x0;x<x1;x++)
         {
             if (Color_Turn)
                 Oled_WriteData(~BMP[j++]);//显示反相图片
             else
                 Oled_WriteData(BMP[j++]);//显示图片

         }
     }
 }


 /**
  * @function: void OLED_HorizontalShift(uint8_t direction)
  * @description: 屏幕内容水平全屏滚动播放
  * @param {uint8_t} direction           LEFT       0x27         RIGHT  0x26
  * @return {*}
  */
 void OLED_HorizontalShift(uint8_t direction)

 {
     Oled_WriteCommand(0x2e);//停止滚动
     Oled_WriteCommand(direction);//设置滚动方向
     Oled_WriteCommand(0x00);//虚拟字节设置，默认为0x00
     Oled_WriteCommand(0x00);//设置开始页地址
     Oled_WriteCommand(0x07);//设置每个滚动步骤之间的时间间隔的帧频
     //  0x00-5帧， 0x01-64帧， 0x02-128帧， 0x03-256帧， 0x04-3帧， 0x05-4帧， 0x06-25帧， 0x07-2帧，
     Oled_WriteCommand(0x07);//设置结束页地址
     Oled_WriteCommand(0x00);//虚拟字节设置，默认为0x00
     Oled_WriteCommand(0xff);//虚拟字节设置，默认为0xff
     Oled_WriteCommand(0x2f);//开启滚动-0x2f，禁用滚动-0x2e，禁用需要重写数据
 }

 /**
  * @function: void OLED_Some_HorizontalShift(uint8_t direction,uint8_t start,uint8_t end)
  * @description: 屏幕部分内容水平滚动播放
  * @param {uint8_t} direction           LEFT       0x27         RIGHT  0x26
  * @param {uint8_t} start 开始页地址  0x00~0x07
  * @param {uint8_t} end  结束页地址  0x01~0x07
  * @return {*}
  */
 void OLED_Some_HorizontalShift(uint8_t direction,uint8_t start,uint8_t end)
 {
     Oled_WriteCommand(0x2e);//停止滚动
     Oled_WriteCommand(direction);//设置滚动方向
     Oled_WriteCommand(0x00);//虚拟字节设置，默认为0x00
     Oled_WriteCommand(start);//设置开始页地址
     Oled_WriteCommand(0x07);//设置每个滚动步骤之间的时间间隔的帧频,0x07即滚动速度2帧
     Oled_WriteCommand(end);//设置结束页地址
     Oled_WriteCommand(0x00);//虚拟字节设置，默认为0x00
     Oled_WriteCommand(0xff);//虚拟字节设置，默认为0xff
     Oled_WriteCommand(0x2f);//开启滚动-0x2f，禁用滚动-0x2e，禁用需要重写数据

 }

 /**
  * @function: void OLED_VerticalAndHorizontalShift(uint8_t direction)
  * @description: 屏幕内容垂直水平全屏滚动播放
  * @param {uint8_t} direction               右上滚动     0x29
  *                                                            左上滚动   0x2A
  * @return {*}
  */
 void OLED_VerticalAndHorizontalShift(uint8_t direction)
 {
     Oled_WriteCommand(0x2e);//停止滚动
     Oled_WriteCommand(direction);//设置滚动方向
     Oled_WriteCommand(0x01);//虚拟字节设置
     Oled_WriteCommand(0x00);//设置开始页地址
     Oled_WriteCommand(0x07);//设置每个滚动步骤之间的时间间隔的帧频，即滚动速度
     Oled_WriteCommand(0x07);//设置结束页地址
     Oled_WriteCommand(0x01);//垂直滚动偏移量
     Oled_WriteCommand(0x00);//虚拟字节设置，默认为0x00
     Oled_WriteCommand(0xff);//虚拟字节设置，默认为0xff
     Oled_WriteCommand(0x2f);//开启滚动-0x2f，禁用滚动-0x2e，禁用需要重写数据
 }

 /**
  * @function: void OLED_DisplayMode(uint8_t mode)
  * @description: 屏幕内容取反显示
  * @param {uint8_t} direction           ON  0xA7  ，
  *                                                          OFF 0xA6    默认此模式，设置像素点亮
  * @return {*}
  */
 void OLED_DisplayMode(uint8_t mode)
 {
     Oled_WriteCommand(mode);
 }

 /**
  * @function: void OLED_IntensityControl(uint8_t intensity)
  * @description: 屏幕亮度调节
  * @param  {uint8_t} intensity  0x00~0xFF,RESET=0x7F
  * @return {*}
  */
 void OLED_IntensityControl(uint8_t intensity)
 {
     Oled_WriteCommand(0x81);
     Oled_WriteCommand(intensity);
 }
