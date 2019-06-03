#include "1602.h"
#include "main.h"

#define us_val 20
#define ms_val 5

void LCD1602_Wait_Ready(void);

extern void delay_us(uint32_t nus);

void LCD_Init(void)
{
	

    /*********************液晶初始化**************************/        
    delay_us(us_val);
		LCD_RS(0);
		LCD_RW(0);
   
		//LCD_write_cmd(0x01);         //清屏
		//HAL_Delay(ms_val);
    LCD_write_cmd(0x38);         // 8bit显示模式,2行,5x7字体
    HAL_Delay(10);   
	  LCD_write_cmd(0x0C);	      //文字不动，地址自动+1
		HAL_Delay(10);
		LCD_write_cmd(0x0f);         //显示光标闪烁
		HAL_Delay(10);
		LCD_write_cmd(0x01);         //清屏
		HAL_Delay(10);
		LCD_write_cmd(0x02);         //光标归位
		HAL_Delay(10);
}
/*--------------------------------------------------
函数说明：写命令到液晶


---------------------------------------------------*/
void LCD_write_cmd(unsigned char cmd)
{
		//LCD1602_Wait_Ready();
		LCD_RS(0);
		HAL_Delay(1);
		LCD_RW(0);
		HAL_Delay(1);
    LCD_Write_byte(cmd);
    HAL_Delay(10);
}
/*--------------------------------------------------
函数说明：写数据到液晶


---------------------------------------------------*/
void LCD_write_data(unsigned char w_data)
{
		//LCD1602_Wait_Ready();
    LCD_RS(1);
		HAL_Delay(1);
		LCD_RW(0);
		HAL_Delay(1);
    LCD_Write_byte(w_data);
    delay_us(us_val);
}
/*--------------------------------------------------
函数说明：写4bit到液晶
--------------------------------------------------*/
void LCD_Write_byte(unsigned char num)
{
		if (num&0x01)
				data0(1);
		else
				data0(0);

		if (num&0x02)
				data1(1);
		else
				data1(0);

		if (num&0x04)
				data2(1);
		else
				data2(0);

		if (num&0x08)
				data3(1);
		else
				data3(0);

		if (num&0x10)
				data4(1);
		else
				data4(0);

		if (num&0x20)
				data5(1);
		else
				data5(0);

		if (num&0x40)
				data6(1);
		else
				data6(0);
		
		if (num&0x80)
				data7(1);
		else
				data7(0);
		HAL_Delay(1);
    LCD_EN(1);
    HAL_Delay(ms_val);
    LCD_EN(0); 
		HAL_Delay(5);
}

/*----------------------------------------------------
LCD_set_xy        : 设置LCD显示的起始位置
输入参数：x、y    : 显示字符串的位置，X:0-15，Y:0-1                
-----------------------------------------------------*/
void LCD_set_xy( unsigned char x, unsigned char y )
{
    unsigned char address = 0;
    if (y==0) 
    {
        address = 0x80 + x;
    }
    else 
    {
        address = 0xc0 + x;
    }
//		y ? (address=0xc0+x): (address=0x80+x) ;
    LCD_write_cmd(address);
		//HAL_Delay(10);
}
/*---------------------------------------------------
LCD_write_string  : 英文字符串显示函数
输入参数：*s      ：英文字符串指针；
          X、Y    : 显示字符串的位置                
---------------------------------------------------*/
void LCD_write_string(unsigned char X,unsigned char Y, char *s)
{
		HAL_Delay(10);
    LCD_set_xy(X,Y); 
		
		while(*s)
		{
			LCD_write_data(*s);
			s ++;
		}
}


void LCD1602_ClearScreen(void)
{
  LCD_write_cmd(0x01);
	
}

void LCD_write_aword(unsigned char X,unsigned char Y, char s)
{
	LCD_set_xy(X, Y);
	LCD_write_data(s);
}

void LCD1602_Wait_Ready(void)     //fatal error  此函数不可用
{
        uint16_t sta;
        
        LCD_Write_byte(0xff);
        LCD_RS(0);
        LCD_RW(1);
        do
        {
                LCD_EN(1); ;
                HAL_Delay(5);        //延时5ms，非常重要
                sta = HAL_GPIO_ReadPin(D7_GPIO_Port, D7_Pin);     //读取状态字
                LCD_EN(0);
        }while(sta & 0x8000);
}
