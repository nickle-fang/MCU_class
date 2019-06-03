#include "NRF24L01.h"
#include "main.h"

extern SPI_HandleTypeDef hspi1;


//NRF24L01 �������� 

uint8_t ff = 0xFF;
HAL_StatusTypeDef ERRORERROR;

unsigned char idel_mode_flag = 0;
unsigned char mode_time_counter = 0;	

const uint8_t TX_ADDRESS[TX_ADR_WIDTH]={0x00,0x98,0x45,0x71,0x10}; //���͵�ַ
const uint8_t RX_ADDRESS[RX_ADR_WIDTH]={0x11,0xa9,0x56,0x82,0x21}; //���յ�ַ	

extern uint8_t TX_frequency;	//24L01Ƶ�ʳ�ʼ��Ϊ5a
extern uint8_t RX_frequency;	//24L01Ƶ�ʳ�ʼ��Ϊ5a

uint8_t bandwidth = 0x06;  //�����ʼ��Ϊ0.25Mbps

//��ʼ��24L01��IO��
void NRF24L01_RX_Init(void)
{
	Set_NRF24L01_RX_CE;                                    //��ʼ��ʱ������
  Set_NRF24L01_RX_CSN;                                   //��ʼ��ʱ������

	//MX_SPI4_Init();                                     //��ʼ��SPI   
	Clr_NRF24L01_RX_CE; 	                                  //ʹ��24L01
	Set_NRF24L01_RX_CSN;                                   //SPIƬѡȡ��
}
void NRF24L01_TX_Init(void)
{
	Set_NRF24L01_TX_CE;                                    //��ʼ��ʱ������
  Set_NRF24L01_TX_CSN;                                   //��ʼ��ʱ������

	//MX_SPI4_Init();                                       //��ʼ��SPI     
	Clr_NRF24L01_TX_CE; 	                                  //ʹ��24L01
	Set_NRF24L01_TX_CSN;                                    //SPIƬѡȡ��
}
//�ϵ���NRF24L01�Ƿ���λ
//д5������Ȼ���ٶ��������бȽϣ�
//��ͬʱ����ֵ:0����ʾ��λ;���򷵻�1����ʾ����λ	
uint8_t NRF24L01_RX_Check(void)
{
	uint8_t buf[5]={0XA5, 0XA5, 0XA5, 0XA5, 0XA5};
	uint8_t buf1[5];
	uint8_t i; 
	NRF24L01_RX_Write_Buf(SPI_WRITE_REG+TX_ADDR, buf,5);//д��5���ֽڵĵ�ַ
	NRF24L01_RX_Read_Buf(TX_ADDR, buf1, 5);              //����д��ĵ�ַ
	for(i=0; i<5; i++)
	{
		if(buf1[i] != 0XA5)
			break;
	};
	if(i!=5)
		return 1;                               //NRF24L01����λ	
	return 0;		                                //NRF24L01��λ
}	 	 
uint8_t NRF24L01_TX_Check(void)
{
	uint8_t buf[5]={0XA5,0XA5,0XA5,0XA5,0XA5};
	uint8_t buf1[5];
	uint8_t i; 
	NRF24L01_TX_Write_Buf(SPI_WRITE_REG+TX_ADDR, buf, 5);//д��5���ֽڵĵ�ַ.	
	NRF24L01_TX_Read_Buf(TX_ADDR, buf1, 5);              //����д��ĵ�ַ
	for(i=0;i<5;i++)
	{
		if(buf1[i] != 0XA5)
			break;
	};
	if(i!=5)
		return 1;                               //NRF24L01����λ	
	return 0;		                              //NRF24L01��λ
}	 
//ͨ��SPIд�Ĵ���
uint8_t NRF24L01_RX_Write_Reg(uint8_t regaddr,uint8_t data)
{
	uint8_t status;
  Clr_NRF24L01_RX_CSN;                    //ʹ��SPI����
	HAL_SPI_TransmitReceive(&hspi1, &regaddr, &status, 1, NRF24L01_TIME_OUT); //���ͼĴ����� 
	HAL_SPI_Transmit(&hspi1, &data, 1, NRF24L01_TIME_OUT);           //д��Ĵ�����ֵ
	
//	HAL_SPI_TransmitReceive_DMA(&hspi1, &regaddr, &status, 1); //���ͼĴ����� 
//	HAL_SPI_Transmit_DMA(&hspi1, &data, 1);           //д��Ĵ�����ֵ
//	HAL_Delay(0);
  Set_NRF24L01_RX_CSN;                    //��ֹSPI����	   
  return(status);       		         //����״ֵ̬
}
uint8_t NRF24L01_TX_Write_Reg(uint8_t regaddr,uint8_t data)  //////////////////////////////////////////time out �����⵼�� TX_Mode()����!!!
{
	uint8_t status;
  Clr_NRF24L01_TX_CSN;                    //ʹ��SPI����
	HAL_SPI_TransmitReceive(&hspi1, &regaddr, &status, 1, NRF24L01_TIME_OUT); //���ͼĴ����� 
	HAL_SPI_Transmit(&hspi1, &data, 1, NRF24L01_TIME_OUT);           //д��Ĵ�����ֵ
  Set_NRF24L01_TX_CSN;                    //��ֹSPI����	   
  return(status);       		              //����״ֵ̬
}
//��ȡSPI�Ĵ���ֵ ��regaddr:Ҫ���ļĴ���
uint8_t NRF24L01_RX_Read_Reg(uint8_t regaddr)
{
	uint8_t reg_val, status;    
 	Clr_NRF24L01_RX_CSN;                //ʹ��SPI����
	HAL_SPI_TransmitReceive(&hspi1, &regaddr, &status, 1, NRF24L01_TIME_OUT);     //���ͼĴ�����
  HAL_SPI_Receive(&hspi1, &reg_val, 1, NRF24L01_TIME_OUT);		//��ȡ�Ĵ�������
  Set_NRF24L01_RX_CSN;                //��ֹSPI����		    
  return reg_val;                 //����״ֵ̬
}	
uint8_t NRF24L01_TX_Read_Reg(uint8_t regaddr)
{
	uint8_t reg_val, status;    
 	Clr_NRF24L01_TX_CSN;                //ʹ��SPI����
	HAL_SPI_TransmitReceive(&hspi1, &regaddr, &status, 1, NRF24L01_TIME_OUT);     //���ͼĴ�����
  HAL_SPI_Receive(&hspi1, &reg_val, 1, NRF24L01_TIME_OUT);		//��ȡ�Ĵ�������
  Set_NRF24L01_TX_CSN;                //��ֹSPI����		    
  return reg_val;                 //����״ֵ̬
}
//��ָ��λ�ö���ָ�����ȵ�����
//*pBuf:����ָ��
//����ֵ,�˴ζ�����״̬�Ĵ���ֵ 
void NRF24L01_RX_Read_Buf(uint8_t regaddr,uint8_t *pBuf,uint8_t datalen)
{      
	//uint8_t status;
	Clr_NRF24L01_RX_CSN;                     //ʹ��SPI����
	//HAL_SPI_Transmit_DMA(&hspi1, &regaddr, 1);   //���ͼĴ���ֵ(λ��),����ȡ״ֵ̬
	//HAL_SPI_TransmitReceive_DMA(&hspi1, &regaddr, &status, 1);
	//HAL_Delay(0);
	HAL_SPI_Transmit(&hspi1, &regaddr, 1, NRF24L01_TIME_OUT);     //���ͼĴ�����
	//HAL_SPI_TransmitReceive_DMA(&hspi1, &regaddr, &status, 1);   //���ͼĴ���ֵ(λ��),����ȡ״̬
	HAL_SPI_Receive(&hspi1, pBuf, datalen, NRF24L01_TIME_OUT);     //��������
  //HAL_SPI_Transmit_DMA(&hspi1, &regaddr, 1);   //���ͼĴ���ֵ(λ��),����ȡ״ֵ̬ 
	//HAL_SPI_Receive_DMA(&hspi1, pBuf, datalen);//��������
	//HAL_Delay(0);
  Set_NRF24L01_RX_CSN;                     //�ر�SPI����
}
void NRF24L01_TX_Read_Buf(uint8_t regaddr,uint8_t *pBuf,uint8_t datalen)
{      
	//uint8_t status;
	Clr_NRF24L01_TX_CSN;                     //ʹ��SPI����
	//HAL_SPI_Transmit_DMA(&hspi2, &regaddr, 1);   //���ͼĴ���ֵ(λ��),����ȡ״ֵ̬
	//HAL_Delay(0);
	//HAL_SPI_TransmitReceive_DMA(&hspi1, &regaddr, &status, 1);
	//HAL_Delay(0);
	HAL_SPI_Transmit(&hspi1, &regaddr, 1, NRF24L01_TIME_OUT);     //���ͼĴ�����
	//HAL_SPI_TransmitReceive_DMA(&hspi2, &regaddr, &status, 1);   //���ͼĴ���ֵ(λ��),����ȡ״̬?
	//HAL_Delay(0);
	HAL_SPI_Receive(&hspi1, pBuf, datalen, NRF24L01_TIME_OUT);     //��������
  //HAL_SPI_Transmit_DMA(&hspi1, &regaddr, 1);   //���ͼĴ���ֵ(λ��),����ȡ״ֵ̬ 
	//HAL_SPI_Receive_DMA(&hspi2, pBuf, datalen);//��������
	//HAL_Delay(0);
  Set_NRF24L01_TX_CSN;                     //�ر�SPI����
}
//��ָ��λ��дָ�����ȵ�����
//*pBuf:����ָ��
//����ֵ,�˴ζ�����״̬�Ĵ���ֵ
uint8_t NRF24L01_RX_Write_Buf(uint8_t regaddr, uint8_t *pBuf, uint8_t datalen)
{
	uint8_t status;
 	Clr_NRF24L01_RX_CSN;                                    //ʹ��SPI����
	//HAL_SPI_Transmit(&hspi1, &regaddr, 1, NRF24L01_TIME_OUT);     //���ͼĴ�����
	//HAL_SPI_Transmit_DMA(&hspi1, &regaddr, 1);                //���ͼĴ���ֵ(λ��),����ȡ״ֵ̬
	HAL_SPI_TransmitReceive(&hspi1, &regaddr, &status, 1, NRF24L01_TIME_OUT);     //���ͼĴ�����
	//HAL_SPI_TransmitReceive_DMA(&hspi1, &regaddr, &status, 1);     //���ͼĴ�����
	//HAL_Delay(0);
	HAL_SPI_Transmit(&hspi1, pBuf, datalen, NRF24L01_TIME_OUT);     //д������	
	//HAL_SPI_Transmit_DMA(&hspi1, pBuf, datalen);     //д������		
	//HAL_Delay(0);
  //HAL_SPI_Transmit_DMA(&hspi1, &regaddr, 1);                //���ͼĴ���ֵ(λ��),����ȡ״ֵ̬
  //HAL_SPI_Transmit_DMA(&hspi1, pBuf, datalen); //д������	 
  Set_NRF24L01_RX_CSN;                                    //�ر�SPI����
  return status;                                       //���ض�����״ֵ̬
}		
uint8_t NRF24L01_TX_Write_Buf(uint8_t regaddr, uint8_t *pBuf, uint8_t datalen)
{
	uint8_t status;
 	Clr_NRF24L01_TX_CSN;  
	//HAL_Delay(100000000);
	//ʹ��SPI����
	//HAL_SPI_Transmit(&hspi2, &regaddr, 1, NRF24L01_TIME_OUT);     //���ͼĴ�����
	//HAL_Delay(0);
	//HAL_SPI_Transmit_DMA(&hspi2, &regaddr, 1);                //���ͼĴ���ֵ(λ��),����ȡ״ֵ̬
	HAL_SPI_TransmitReceive(&hspi1, &regaddr, &status, 1, NRF24L01_TIME_OUT);     //���ͼĴ�����
	//HAL_SPI_TransmitReceive_DMA(&hspi1, &regaddr, &status, 1);     //���ͼĴ�����
	//HAL_Delay(0);
	HAL_SPI_Transmit(&hspi1, pBuf, datalen, NRF24L01_TIME_OUT);     //д������	
	//HAL_SPI_Transmit_DMA(&hspi2, pBuf, datalen);     //д������		
	//HAL_Delay(0);
  //HAL_SPI_Transmit_DMA(&hspi1, &regaddr, 1);           //���ͼĴ���ֵ(λ��),����ȡ״ֵ̬
  //HAL_SPI_Transmit_DMA(&hspi1, pBuf, datalen);         //д������	 
  Set_NRF24L01_TX_CSN;                                   //�ر�SPI����
  return status;                                         //���ض�����״ֵ̬
}	
//����NRF24L01����һ������
//txbuf:�����������׵�ַ
//����ֵ:�������״��
uint8_t NRF24L01_TxPacket(uint8_t *txbuf)
{
	uint8_t state;   
	Clr_NRF24L01_TX_CE;
  NRF24L01_TX_Write_Buf(WR_TX_PLOAD,txbuf,TX_PLOAD_WIDTH);//д���ݵ�TX BUF  25���ֽ�
 	Set_NRF24L01_TX_CE;                                     //��������	   
	//return TX_OK;

	while(READ_NRF24L01_TX_IRQ != 0);                         //�ȴ��������
	state=NRF24L01_TX_Read_Reg(STATUS);                     //��ȡ״̬�Ĵ�����ֵ	   
	NRF24L01_TX_Write_Reg(SPI_WRITE_REG+STATUS,state);      //���TX_DS��MAX_RT�жϱ�־
	if(state&MAX_TX)                                        //�ﵽ����ط�����
	{
		NRF24L01_TX_Write_Reg(FLUSH_TX,0xff);               //���TX FIFO�Ĵ��� 
		return MAX_TX; 
	}
	if(state&TX_OK)                                      //�������
	{
		return TX_OK;
	}
	return 0xff;                                         //����ԭ����ʧ��
}

//����NRF24L01����һ������
//txbuf:�����������׵�ַ
//����ֵ:0��������ɣ��������������
uint8_t NRF24L01_RxPacket(uint8_t *rxbuf)
{
	uint8_t state;		    							      
	state=NRF24L01_RX_Read_Reg(STATUS);                //��ȡ״̬�Ĵ�����ֵ   
	NRF24L01_RX_Write_Reg(SPI_WRITE_REG+STATUS,state); //���TX_DS��MAX_RT�жϱ�־
	if(state&RX_OK)                                 //���յ�����
	{
	  //HAL_UART_Transmit(&huart3, &RX_frequency, sizeof(RX_frequency), 0xffff);
		NRF24L01_RX_Read_Buf(RD_RX_PLOAD,rxbuf,RX_PLOAD_WIDTH);//��ȡ����
		NRF24L01_RX_Write_Reg(FLUSH_RX,0xff);          //���RX FIFO�Ĵ��� 
		return 0; 
	}	   
	return 1;                                      //û�յ��κ�����
}

//�ú�����ʼ��NRF24L01��RXģʽ
//����RX��ַ,дRX���ݿ��,ѡ��RFƵ��,�����ʺ�LNA HCURR
//��CE��ߺ�,������RXģʽ,�����Խ���������		   
void RX_Mode(void)
{
	Clr_NRF24L01_RX_CE;	  
	NRF24L01_RX_Write_Reg( SETUP_AW, 0x3 );
	
  //дRX�ڵ��ַ
  NRF24L01_RX_Write_Buf(SPI_WRITE_REG+RX_ADDR_P0,(uint8_t*)RX_ADDRESS,RX_ADR_WIDTH);

  //Disable ͨ��0���Զ�Ӧ��
	NRF24L01_RX_Write_Reg(SPI_WRITE_REG+EN_AA, 0x00);    
  //ʹ��ͨ��0�Ľ��յ�ַ  	 
	NRF24L01_RX_Write_Reg(SPI_WRITE_REG+EN_RXADDR, 0x01);
  //����RFͨ��Ƶ��		  
  NRF24L01_RX_Write_Reg(SPI_WRITE_REG+RF_CH, RX_frequency);	     
  //ѡ��ͨ��0����Ч���ݿ�� 	    
  NRF24L01_RX_Write_Reg(SPI_WRITE_REG+RX_PW_P0,RX_PLOAD_WIDTH);
  //����TX�������,20db����,0.25Mbps,���������濪��   
	NRF24L01_RX_Write_Reg(SPI_WRITE_REG+RF_SETUP,bandwidth);
  //���û�������ģʽ�Ĳ���;PWR_UP,EN_CRC,16BIT_CRC,PRIM_RX����ģʽ 
  NRF24L01_RX_Write_Reg(SPI_WRITE_REG+CONFIG, 0x0f); 
  //CEΪ��,�������ģʽ 
	Set_NRF24L01_RX_CE;                                
}			

//�ú�����ʼ��NRF24L01��TXģʽ
//����TX��ַ,дTX���ݿ��,����RX�Զ�Ӧ��ĵ�ַ,���TX��������,
//ѡ��RFƵ��,�����ʺ�LNA HCURR PWR_UP,CRCʹ��
//��CE��ߺ�,������RXģʽ,�����Խ���������		   
//CEΪ�ߴ���10us,����������.	 
void TX_Mode(void)
{												 
	Clr_NRF24L01_TX_CE;	    
	//Set up Address Width
	NRF24L01_TX_Write_Reg( SETUP_AW, 0x3 );
      
	//дTX�ڵ��ַ 
  NRF24L01_TX_Write_Buf(SPI_WRITE_REG+TX_ADDR, (uint8_t*)TX_ADDRESS,TX_ADR_WIDTH);    
  //����TX�ڵ��ַ,��ҪΪ��ʹ��ACK	  
	NRF24L01_TX_Write_Buf(SPI_WRITE_REG+RX_ADDR_P0, (uint8_t*)RX_ADDRESS,RX_ADR_WIDTH); 
	NRF24L01_TX_Write_Reg(SPI_WRITE_REG+RX_PW_P0, TX_PLOAD_WIDTH );
  //Disable ͨ��0���Զ�Ӧ��
  NRF24L01_TX_Write_Reg(SPI_WRITE_REG+EN_AA, 0x00);     
  //Disable All ͨ���Ľ��յ�ַ
	NRF24L01_TX_Write_Reg(SPI_WRITE_REG+EN_RXADDR, 0x00); 
  //�����Զ��ط����ʱ��:500us + 86us;����Զ��ط�����:10��
  NRF24L01_TX_Write_Reg(SPI_WRITE_REG+SETUP_RETR, 0);
	//����RFͨ��Ϊ24
  NRF24L01_TX_Write_Reg(SPI_WRITE_REG+RF_CH, TX_frequency);
	//NRF24L01_Write_Reg(SPI_WRITE_REG+RF_CH,frequency + );
  //����TX�������,20db����,0.25Mbps,���������濪��   
  NRF24L01_TX_Write_Reg(SPI_WRITE_REG+RF_SETUP, bandwidth);
  //���û�������ģʽ�Ĳ���;PWR_UP,EN_CRC,16BIT_CRC,PRIM_RX����ģʽ,���������ж�
  NRF24L01_TX_Write_Reg(SPI_WRITE_REG+CONFIG, ( 1 << 3 ) | //Enable CRC
                                              ( 1 << 1 )| // PWR_UP
																        			( 1 << 2)); // 16bit CRC
  // CEΪ��,10us����������
	Set_NRF24L01_TX_CE;                                  
}	


