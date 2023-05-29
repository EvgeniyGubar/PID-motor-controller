#define ENCOD 0b00000011 //������ � �����
#define PORT_ENC PORTC
#define PIN_ENC PINC
 
uint8_t encoder;  //�������� ���������� ����������� ���������
uint8_t tmp;

//������������� ������ �������� � ��������� ���������� INT0
void encoder_init()
{
	PORT_ENC|=(1<<PD4)|(1<<PD5); //�������� ��������
}


uint8_t encoder_proc()
{
	static char enc=ENCOD;
	char buf=((PIN_ENC&(1<<PD5))>>4)|((PIN_ENC&(1<<PD4))>>4);//���������� � ��������� ���� �����

	if (enc==ENCOD&&buf!=enc) //���� ��������� ���� ������� + ���� ����� �� ����� ����������� ���������
	{
		if(buf==0b00000001)//���� ����� ���� 01
		{encoder++;
		enc=0;}//����������� ���������� �� 1 + �������� � ����� �������� ������
		if(buf==0b00000010)//���� ����� ���� 10
		{encoder--;
		enc=0;} //��������� ���������� �� 1 + �������� � ����� �������� ������
	}
	if (buf==ENCOD) {enc=ENCOD;}//���� ������ �� ������ ������� �� ��������� ���
	
	//return encoder;
	
  	if (encoder>tmp)
  	{
		tmp=encoder;
	  	return -2;
	  	
  	}
  	if (encoder<tmp)
  	{
		tmp=encoder;
	  	return 2;
	}
  	return 0;
}