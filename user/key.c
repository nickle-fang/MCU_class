#include "key.h"
#include "1602.h"
#include "NRF24L01.h"

extern uint8_t flag;
extern int posi;
extern unsigned char display_1[16];
extern unsigned char display_2[16];
extern unsigned char tx_buf[25];
extern TIM_HandleTypeDef htim2;

void key_init()
{
	HAL_GPIO_WritePin(R1_GPIO_Port, R1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(R2_GPIO_Port, R2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(R3_GPIO_Port, R3_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(R4_GPIO_Port, R4_Pin, GPIO_PIN_RESET);
}

unsigned char read_pin()
{
	unsigned char read;
	int read_bit[4];
	read_bit[0] = HAL_GPIO_ReadPin(C1_GPIO_Port, C1_Pin);
	read_bit[1] = HAL_GPIO_ReadPin(C2_GPIO_Port, C2_Pin);
	read_bit[2] = HAL_GPIO_ReadPin(C3_GPIO_Port, C3_Pin);
	read_bit[3] = HAL_GPIO_ReadPin(C4_GPIO_Port, C4_Pin);
	
	read = 0x0f & (8 * read_bit[0] + 4 * read_bit[1] + 2 * read_bit[2] + read_bit[3]);
	return read;
}

void write_pin(uint8_t hex)
{
	HAL_GPIO_WritePin(R1_GPIO_Port, R1_Pin, (hex&0x08)>>3);
	HAL_GPIO_WritePin(R2_GPIO_Port, R2_Pin, (hex&0x04)>>2);
	HAL_GPIO_WritePin(R3_GPIO_Port, R3_Pin, (hex&0x02)>>1);
	HAL_GPIO_WritePin(R4_GPIO_Port, R4_Pin, (hex&0x01));
}


void key_scan()
{
	key_init();
	if (read_pin() != 0x0f)
	{
		HAL_GPIO_TogglePin(LED_0_GPIO_Port, LED_0_Pin);      //调试用
		HAL_Delay(10);              //延时以消除抖动
		if (read_pin() != 0x0f)    //确认非抖动
		{
			flag = 1;
			write_pin(0x07);         //扫描第1行
			switch(read_pin())
			{
				case 0x07: key_val = 1; break;
				case 0x0b: key_val = 2; break;
				case 0x0d: key_val = 3; break;
				case 0x0e: key_val = 4; break;
				default: break;
			}
			
			if(key_val == 0)
			{
				write_pin(0x0b);        //扫描第2行
				switch(read_pin())
				{
					case 0x07: key_val = 5; break;
					case 0x0b: key_val = 6; break;
					case 0x0d: key_val = 7; break;
					case 0x0e: key_val = 8; break;
					default: break;
				}
			}
			
			if(key_val == 0)
			{
				write_pin(0x0d);        //扫描第3行
				switch(read_pin())
				{
					case 0x07: key_val = 9; break;
					case 0x0b: key_val = 10; break;
					case 0x0d: key_val = 11; break;
					case 0x0e: key_val = 12; break;
					default: break;
				}
			}
			
			if(key_val == 0)
			{
				write_pin(0x0e);        //扫描第4行
				switch(read_pin())
				{
					case 0x07: key_val = 13; break;
					case 0x0b: key_val = 14; break;
					case 0x0d: key_val = 15; break;
					case 0x0e: key_val = 16; break;
					default: break;
				}
			}
		}
	}
}


void key1()
{
	LCD1602_ClearScreen();
	//LCD_write_string(posi,0,"1");
	display_1[posi] = '1';
	display(display_1);
	posi ++;
}

void key2()
{
	LCD1602_ClearScreen();
	//LCD_write_string(posi,0,"2");
	display_1[posi] = '2';
	display(display_1);
	posi ++;
}

void key3()
{
	LCD1602_ClearScreen();
	//LCD_write_string(posi,0,"3");
	display_1[posi] = '3';
	display(display_1);
	posi ++;
}

void key4()
{
	LCD1602_ClearScreen();
	//LCD_write_string(posi,0,"4");
	display_1[posi] = '4';
	display(display_1);
	posi ++;
}

void key5()
{
	LCD1602_ClearScreen();
	//LCD_write_string(posi,0,"5");
	display_1[posi] = '5';
	display(display_1);
	posi ++;
}

void key6()
{
	LCD1602_ClearScreen();
	//LCD_write_string(posi,0,"6");
	display_1[posi] = '6';
	display(display_1);
	posi ++;
}

void key7()
{
	LCD1602_ClearScreen();
	//LCD_write_string(posi,0,"7");
	display_1[posi] = '7';
	display(display_1);
	posi ++;
}

void key8()
{
	LCD1602_ClearScreen();
	//LCD_write_string(posi,0,"8");
	display_1[posi] = '8';
	display(display_1);
	posi ++;
}

void key9()
{
	LCD1602_ClearScreen();
	//LCD_write_string(posi,0,"9");
	display_1[posi] = '9';
	display(display_1);
	posi ++;
}

void key10()
{
	LCD1602_ClearScreen();
	//LCD_write_string(posi,0,"0");
	display_1[posi] = '0';
	display(display_1);
	posi ++;
}

void key11()
{
	LCD1602_ClearScreen();
	clear_display_1_char();
	posi = 0;
}

void key12()
{
	int i;
	for(i = 0; i <= 2; i++)
	{
		if(NRF24L01_TxPacket(display_1) == TX_OK)
		{
			LCD_write_string(0, 1, "MSG sended");
		}
	}
//		HAL_Delay(500);
//	  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
//  	HAL_Delay(150);
//  	HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
//  	HAL_Delay(60);
//  	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
//  	HAL_Delay(150);
//  	HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
}

void key13()
{
}

void key14()
{
}

void key15()
{
}

void key16()
{
}


void clear_display_1_char()
{
	int i;
	for (i = 0; i<= 15; i++)
	{
		display_1[i] = 0;
	}
}

void clear_display_2_char()
{
	int i;
	for (i = 0; i<= 15; i++)
	{
		display_2[i] = 0;
	}
}


void display(unsigned char *s)
{
	while(*s != 0)
	{
		LCD_write_data(*s);
		s ++;
	}
}
