#include "1602.h"
#include "main.h"

#define us_val 20
#define ms_val 5

void LCD1602_Wait_Ready(void);

extern void delay_us(uint32_t nus);

void LCD_Init(void)
{
	

    /*********************Һ����ʼ��**************************/        
    delay_us(us_val);
		LCD_RS(0);
		LCD_RW(0);
   
		//LCD_write_cmd(0x01);         //����
		//HAL_Delay(ms_val);
    LCD_write_cmd(0x38);         // 8bit��ʾģʽ,2��,5x7����
    HAL_Delay(10);   
	  LCD_write_cmd(0x0C);	      //���ֲ�������ַ�Զ�+1
		HAL_Delay(10);
		LCD_write_cmd(0x0f);         //��ʾ�����˸
		HAL_Delay(10);
		LCD_write_cmd(0x01);         //����
		HAL_Delay(10);
		LCD_write_cmd(0x02);         //����λ
		HAL_Delay(10);
}
/*--------------------------------------------------
����˵����д���Һ��


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
����˵����д���ݵ�Һ��


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
����˵����д4bit��Һ��
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
LCD_set_xy        : ����LCD��ʾ����ʼλ��
���������x��y    : ��ʾ�ַ�����λ�ã�X:0-15��Y:0-1                
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
LCD_write_string  : Ӣ���ַ�����ʾ����
���������*s      ��Ӣ���ַ���ָ�룻
          X��Y    : ��ʾ�ַ�����λ��                
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

void LCD1602_Wait_Ready(void)     //fatal error  �˺���������
{
        uint16_t sta;
        
        LCD_Write_byte(0xff);
        LCD_RS(0);
        LCD_RW(1);
        do
        {
                LCD_EN(1); ;
                HAL_Delay(5);        //��ʱ5ms���ǳ���Ҫ
                sta = HAL_GPIO_ReadPin(D7_GPIO_Port, D7_Pin);     //��ȡ״̬��
                LCD_EN(0);
        }while(sta & 0x8000);
}
