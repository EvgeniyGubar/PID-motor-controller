#define ENCOD 0b00000011 //макрос в помощ
#define PORT_ENC PORTC
#define PIN_ENC PINC
 
uint8_t encoder;  //глобальн переменная управляемая энкодером
uint8_t tmp;

//Инициализация портов энкодера и настройка прерывания INT0
void encoder_init()
{
	PORT_ENC|=(1<<PD4)|(1<<PD5); //включить подтяжку
}


uint8_t encoder_proc()
{
	static char enc=ENCOD;
	char buf=((PIN_ENC&(1<<PD5))>>4)|((PIN_ENC&(1<<PD4))>>4);//спрашиваем и сохраняем сост ножек

	if (enc==ENCOD&&buf!=enc) //если предыдущь сост единицы + сост ножек не равно предыдущему состоянию
	{
		if(buf==0b00000001)//если новое сост 01
		{encoder++;
		enc=0;}//увеличиваем переменную на 1 + преходим в режим ожидания единиц
		if(buf==0b00000010)//если новое сост 10
		{encoder--;
		enc=0;} //уменьшаем переменную на 1 + преходим в режим ожидания единиц
	}
	if (buf==ENCOD) {enc=ENCOD;}//если сейчас на ножках единицы то запомнить это
	
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