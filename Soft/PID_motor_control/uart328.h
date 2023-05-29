
void USART_Init(unsigned int ubrr)
{
	/*Set baud rate */
	UBRR0H = (unsigned char)(ubrr>>8);
	UBRR0L = (unsigned char)ubrr;
	/*Enable receiver and transmitter */
	UCSR0B = (1<<RXEN0)|(1<<TXEN0);
	/* Set frame format: 8data, 2stop bit */
	UCSR0C = (1<<USBS0)|(3<<UCSZ00);
}

void USART_TX(unsigned char data)
{
	/* Wait for empty transmit buffer */
	while (!(UCSR0A & (1<<UDRE0)));
	/* Put data into buffer, sends the data */
	UDR0 = data;
}

unsigned char USART_RX( void )
{
	//ждем приема байта
	while( ( UCSR0A & ( 1 << RXC0 ) ) == 0  );	
	//считываем принятый байт
	return UDR0;
}

/*char Serial_get( void )    //чтение строки. Надо делать
{
	//ждем приема байта
	char *str[i]=USART_RX();
	//считываем принятый байт
	return UDR0;
}*/

void Serial_print( char *str )
{
	unsigned char c;
	while( ( c = *str++ ) != 0 ) {
		USART_TX( c );
	}
}

void Serial_println( char *str )
{
	unsigned char c;
	while( ( c = *str++ ) != 0 ) {
		USART_TX( c );
	}
	Serial_print("\r\n");	
}

