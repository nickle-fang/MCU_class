#define uint8 unsigned char

//#ifndef __LCD1602_H
//#define __LCD1602_H


#define    LCD_RS(x)  x ? HAL_GPIO_WritePin(RS_GPIO_Port, RS_Pin, GPIO_PIN_SET): HAL_GPIO_WritePin(RS_GPIO_Port, RS_Pin, GPIO_PIN_RESET)
#define    LCD_RW(x)  x ? HAL_GPIO_WritePin(RW_GPIO_Port, RW_Pin, GPIO_PIN_SET): HAL_GPIO_WritePin(RW_GPIO_Port, RW_Pin, GPIO_PIN_RESET)
#define    LCD_EN(x)  x ? HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, GPIO_PIN_SET): HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, GPIO_PIN_RESET)

#define    data0(x)  x ? HAL_GPIO_WritePin(D0_GPIO_Port, D0_Pin, GPIO_PIN_SET): HAL_GPIO_WritePin(D0_GPIO_Port, D0_Pin, GPIO_PIN_RESET)
#define    data1(x)  x ? HAL_GPIO_WritePin(D1_GPIO_Port, D1_Pin, GPIO_PIN_SET): HAL_GPIO_WritePin(D1_GPIO_Port, D1_Pin, GPIO_PIN_RESET)
#define    data2(x)  x ? HAL_GPIO_WritePin(D2_GPIO_Port, D2_Pin, GPIO_PIN_SET): HAL_GPIO_WritePin(D2_GPIO_Port, D2_Pin, GPIO_PIN_RESET)
#define    data3(x)  x ? HAL_GPIO_WritePin(D3_GPIO_Port, D3_Pin, GPIO_PIN_SET): HAL_GPIO_WritePin(D3_GPIO_Port, D3_Pin, GPIO_PIN_RESET)
#define    data4(x)  x ? HAL_GPIO_WritePin(D4_GPIO_Port, D4_Pin, GPIO_PIN_SET): HAL_GPIO_WritePin(D4_GPIO_Port, D4_Pin, GPIO_PIN_RESET)
#define    data5(x)  x ? HAL_GPIO_WritePin(D5_GPIO_Port, D5_Pin, GPIO_PIN_SET): HAL_GPIO_WritePin(D5_GPIO_Port, D5_Pin, GPIO_PIN_RESET)
#define    data6(x)  x ? HAL_GPIO_WritePin(D6_GPIO_Port, D6_Pin, GPIO_PIN_SET): HAL_GPIO_WritePin(D6_GPIO_Port, D6_Pin, GPIO_PIN_RESET)
#define    data7(x)  x ? HAL_GPIO_WritePin(D7_GPIO_Port, D7_Pin, GPIO_PIN_SET): HAL_GPIO_WritePin(D7_GPIO_Port, D7_Pin, GPIO_PIN_RESET)




/*============================================================
液晶指令说明：
0x01-------------->清除屏幕，置AC为0，光标回位。
0x02-------------->DDRAM 地址为0，显示回原位，DDRAM内容不变.
0x03-------------->DDRAM 地址为0，显示回原位，DDRAM内容不变.
0x04-------------->设置光标移动方向减量方式，并指定显示不移动。
0x05-------------->设置光标移动方向减量方式，并指定显示移动。
0x06-------------->设置光标移动方向增量方式，并指定显示不移动。
0x07-------------->设置光标移动方向增量方式，并指定显示移动。
0x08-------------->设置显示关、光标关、光标所在字符不闪烁.
0x0c-------------->显示开
0x0e-------------->显示开，光标开
0x0f-------------->显示开，光标开，光标闪烁
0x10-------------->光标移位，左移
0x14-------------->光标移位，右移
0x18-------------->显示移位，左移
0x1c-------------->显示移位，右移
0x20-------------->4BIT模式，显示一行，5x7字体
0x24-------------->4BIT模式，显示一行，5x10字体
0x28-------------->4BIT模式，显示两行，5x7字体
0x2c-------------->4BIT模式，显示两行，5x10字体
0x30-------------->8BIT模式，显示一行，5x7字体
0x34-------------->8BIT模式，显示一行，5x10字体
0x38-------------->8BIT模式，显示两行，5x7字体
0x3c-------------->8BIT模式，显示两行，5x10字体
0x30-------------->8BIT模式，显示一行，5x7字体
0x30-------------->8BIT模式，显示一行，5x7字体
0x30-------------->8BIT模式，显示一行，5x7字体
0x30-------------->8BIT模式，显示一行，5x7字体

显示地址：
===============================================================
=0x00 0x01 0x02 0x03 0x04 0x05 0x06 0x07 ................ 0x27=
=0x40 0x41 0x42 0x43 0x44 0x45 0x46 0x47 ................ 0x67=
===============================================================
每行可以显示40个字符，可以看到的只有16个字符，可以通过指令使字符
整体移动来显示所有字符。
四线控制的方式：先送字节的高四位，再送低四位。
值得注意的是当使用的I/O口为高四位时数据先给另一个变量，变量再将
数据高四位送到I/O口，接着是将变量左移四位，再送到I/O口上去。
当使用的I/O口为低四位时数据先送入I/O口。
使用时注意一下。给另一个变量，变量右移四位后送到I/O
口上去，接着将数据给变量直接
============================================================*/
void LCD_Init                     (void);
void LCD_write_cmd          (unsigned char cmd);
void LCD_write_data         (unsigned char w_data);
void LCD_Write_byte    (unsigned char half_byte);
void LCD_set_xy             (unsigned char x, unsigned char y);
void LCD_write_string       (unsigned char X,unsigned char Y, char *s);
//==================================================
void LCD1602_ClearScreen(void);

void LCD_write_aword(unsigned char X,unsigned char Y, char s);


//#endif



//void LCD_clear(void); 
//void LCD_write_char(uint8 row, uint8 col, char ch); 
//void LCD_write_str(uint8 x, uint8 y, char str[]); 
//void write_cmd(uint8 cmd);

