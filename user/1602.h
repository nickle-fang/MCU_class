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
Һ��ָ��˵����
0x01-------------->�����Ļ����ACΪ0������λ��
0x02-------------->DDRAM ��ַΪ0����ʾ��ԭλ��DDRAM���ݲ���.
0x03-------------->DDRAM ��ַΪ0����ʾ��ԭλ��DDRAM���ݲ���.
0x04-------------->���ù���ƶ����������ʽ����ָ����ʾ���ƶ���
0x05-------------->���ù���ƶ����������ʽ����ָ����ʾ�ƶ���
0x06-------------->���ù���ƶ�����������ʽ����ָ����ʾ���ƶ���
0x07-------------->���ù���ƶ�����������ʽ����ָ����ʾ�ƶ���
0x08-------------->������ʾ�ء����ء���������ַ�����˸.
0x0c-------------->��ʾ��
0x0e-------------->��ʾ������꿪
0x0f-------------->��ʾ������꿪�������˸
0x10-------------->�����λ������
0x14-------------->�����λ������
0x18-------------->��ʾ��λ������
0x1c-------------->��ʾ��λ������
0x20-------------->4BITģʽ����ʾһ�У�5x7����
0x24-------------->4BITģʽ����ʾһ�У�5x10����
0x28-------------->4BITģʽ����ʾ���У�5x7����
0x2c-------------->4BITģʽ����ʾ���У�5x10����
0x30-------------->8BITģʽ����ʾһ�У�5x7����
0x34-------------->8BITģʽ����ʾһ�У�5x10����
0x38-------------->8BITģʽ����ʾ���У�5x7����
0x3c-------------->8BITģʽ����ʾ���У�5x10����
0x30-------------->8BITģʽ����ʾһ�У�5x7����
0x30-------------->8BITģʽ����ʾһ�У�5x7����
0x30-------------->8BITģʽ����ʾһ�У�5x7����
0x30-------------->8BITģʽ����ʾһ�У�5x7����

��ʾ��ַ��
===============================================================
=0x00 0x01 0x02 0x03 0x04 0x05 0x06 0x07 ................ 0x27=
=0x40 0x41 0x42 0x43 0x44 0x45 0x46 0x47 ................ 0x67=
===============================================================
ÿ�п�����ʾ40���ַ������Կ�����ֻ��16���ַ�������ͨ��ָ��ʹ�ַ�
�����ƶ�����ʾ�����ַ���
���߿��Ƶķ�ʽ�������ֽڵĸ���λ�����͵���λ��
ֵ��ע����ǵ�ʹ�õ�I/O��Ϊ����λʱ�����ȸ���һ�������������ٽ�
���ݸ���λ�͵�I/O�ڣ������ǽ�����������λ�����͵�I/O����ȥ��
��ʹ�õ�I/O��Ϊ����λʱ����������I/O�ڡ�
ʹ��ʱע��һ�¡�����һ������������������λ���͵�I/O
����ȥ�����Ž����ݸ�����ֱ��
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

