/*****************************************************************************************

                   ***** DYREKTYWY PREPROCESORA *****

*****************************************************************************************/


#define F_CPU 16000000UL
#include <avr/io.h>
#include <stdint.h>
#include <avr/interrupt.h>
#include <util/delay.h>

// Makra upraszczaj¹ce dostêp do rejsestrów ukladu I/O
#define DDR(x) tempDDR(x)
#define tempDDR(x) (DDR##x)
#define PORT(x) tempPORT(x)
#define tempPORT(x) (PORT##x)
#define PIN(x) tempPIN(x)
#define tempPIN(x) (PIN##x) 

// Ustawienie sprzetowe:
#define LCD_D7PORT  B
#define LCD_D7 7
#define LCD_D6PORT  B
#define LCD_D6 6
#define LCD_D5PORT  B
#define LCD_D5 5
#define LCD_D4PORT  B
#define LCD_D4 4

#define LCD_RSPORT D
#define LCD_RS 0

#define LCD_EPORT D
#define LCD_E 1

#define LED_1PORT B 
#define LED_1PIN 0
#define LED_2PORT B
#define LED_2PIN 1

#define PRZYCISK_1_PORT C
#define PRZYCISK_1_PIN 7
#define PRZYCISK_2_PORT C
#define PRZYCISK_2_PIN 6


#define ADCIN PA5

// Definicja adresow wyswietlacza 2x16:
#define LCD_WIERSZ1 0x00		// pierwszy znak 1. wiersza
#define LCD_WIERSZ2 0x40		// pierwszy znak 2. wiersza

// Makra do operacji na sygnalach RS i E:
#define RS_ON 	PORT(LCD_RSPORT) |= (1<<LCD_RS)			// stan wysoki na linii RS
#define RS_OFF 	PORT(LCD_RSPORT) &= ~(1<<LCD_RS)		// stan niski na linii RS

#define E_ON 	PORT(LCD_EPORT) |= (1<<LCD_E)			// stan wysoki na linii E
#define E_OFF 	PORT(LCD_EPORT) &= ~(1<<LCD_E)			// stan niski na linii E


// Komendy steruj¹ce wyswietlaczem:
#define LCD_COMM_CLEAR					0b00000001
#define LCD_COMM_HOME					0b00000010
#define LCD_COMM_ENTRY					0b00000100
#define LCD_COMM_ENTRYR					0b00000010
#define LCD_COMM_ONOFF					0b00001000
#define LCD_COMM_DISPLAYON				0b00000100
#define LCD_COMM_DISPLAYOFF				0b00000000
#define LCD_COMM_KURSORON				0b00000010
#define LCD_COMM_KURSOROFF				0b00000000
#define LCD_COMM_BLINKON				0b00000001
#define LCD_COMM_BLINKOFF				0b00000000
#define LCD_COMM_FUNC					0b00100000
#define LCD_COMM_4BIT					0b00000000
#define LCD_COMM_2WIERSZE				0b00001000
#define LCD_COMM_5x7					0b00000000
#define LCD_COMM_DDRAM					0b10000000

#define LED_1 PORT(LED_1PORT) ^= (1<<LED_1PIN)
#define LED_2 PORT(LED_2PORT) ^= (1<<LED_2PIN)

#define PRZYCISK_1 PORT(PRZYCISK_1_PORT) |= (1<<PRZYCISK_1_PIN)
#define PRZYCISK_2 PORT(PRZYCISK_2_PORT) |= (1<<PRZYCISK_2_PIN)
#define KEY1 (1<<PC7)


/*****************************************************************************************

                   ***** DEKLARACJE I DEFINICJE ZMIENNYCH GLOBALNYCH *****

*****************************************************************************************/

volatile uint16_t Timer1;
volatile uint16_t Timer2;
volatile uint16_t Timer3;
volatile uint16_t P1;

uint8_t czas = 10;
uint16_t temp, moc;//zmienna do obliczeñ napiêcia
uint8_t low, s = 0;

typedef struct {
	volatile uint8_t *KPIN;
	uint8_t key_mask;
	uint8_t wait_time_s;
	void (*kfun1)(void);
	void (*kfun2)(void);
	uint8_t klock;
	uint8_t flag;
} TBUTTON;

void serwo(void) //opisac
{
	pwm();
	OCR1B = 3999;
	_delay_ms(5000);
	pwm();
	OCR1B = 999;
}

void czasplus(void) //opisac
{
	czas+=10;
	lcd_clear();
	lcd_home();  //forced cursor to first line
	lcd_string("Czas: ");
	lcd_int(czas);
	lcd_string("s");
	lcd_second_line();
	lcd_string("Moc: "); 
	lcd_int(moc);
	lcd_string(" W");
}

void grzanie(void) //opisac
{
	while(czas > 0)
	{
		if(Timer2 == 0)
		{
			Timer2 = 1000;
			czas--;
			LED_1;
			lcd_clear();
			lcd_home();  //forced cursor to first line
			lcd_string("Czas: ");
			lcd_int(czas);
			lcd_string("s");
			lcd_second_line();
			lcd_string("Moc: ");
			lcd_int(moc);
			lcd_string(" W");
		}
		pwm();
		OCR1B = 3999;
	}
	pwm();
	OCR1B = 0;
}

void key_press( TBUTTON * btn );

TBUTTON button; // definicja KLAWISZA

/*****************************************************************************************

                   ***** DEKLARACJE FUNKCJI *****

*****************************************************************************************/

static inline void lcd_wyslijPOlBajtu (uint8_t data);
void lcd_wyslijBajt(unsigned char _data);
void lcd_wyslijKomende(uint8_t cmd);
void lcd_wyslijDane(uint8_t data);
void lcd_clear(void);
void lcd_init(void);
void lcd_home(void);
void lcd_kurson_on(void);
void lcd_kurson_off(void);
void lcd_blink_on(void);
void lcd_blink_off(void);
void lcd_display_on(void);
void lcd_display_off(void);
void lcd_char(char znak);
void lcd_string(char *napis);
void lcd_int(int liczba);
void lcd_locate(uint8_t y, uint8_t x);

/*****************************************************************************************

                   ***** FUNKCJA GLOWNA *****

*****************************************************************************************/



int main() //opisac
{

	DDRD = 0xFF;
	DDRA = 0xFF;
	DDRB = 0xFF;
	PORTD = 0b11000011;
	PORTC |= KEY1;
	PORTC = PORTC|(1<<6);
	lcd_init();	
	init_timer();
	
	lcd_home(); //forced cursor to first line
	lcd_string("ADC test");
	przetwornik();
	pwm();
	OCR1B = 0;
		
	button.KPIN = &PINC;
	button.key_mask = KEY1;
	button.wait_time_s = 3;
	button.kfun1 = czasplus;
	button.kfun2 = grzanie;	
	
	
		 
	sei();//Globalne uruchomienie przerwañ	
	
	while(1)
	{
				
		key_press( &button );
		
		if(!(PINC & (1<<6)))
		{
			serwo();
		}			
		
		ADCSRA |= (1<<ADSC);//ADSC: Uruchomienie pojedynczej konwersji
		
		while(ADCSRA & (1<<ADSC)); //czeka na zakoñczenie konwersji
		{ 
			low=ADCL;   //wpisywanie wartosci
			temp = ADCH<<8;
			moc = temp | low;
			moc = 400+(moc*0.390626);
		}   
		lcd_clear();
		lcd_home();  //forced cursor to first line
		lcd_string("Czas: ");
		lcd_int(czas);
		lcd_string("s");
		lcd_second_line();
		lcd_string("Moc: "); 
		lcd_int(moc);
		lcd_string(" W");
		_delay_ms(30);
		
	}
} 

/*****************************************************************************************

                   ***** DEFINICJE FUNKCJI *****

*****************************************************************************************/

void key_press( TBUTTON * btn ) {
	
	register uint8_t key_press = (*btn->KPIN & btn->key_mask);
	
	if( !btn->klock && !key_press ) {
		btn->klock=1;
		
		// reakcja na PRESS krótkie wciniêcie klawisza
		if(btn->kfun1) btn->kfun1();
		btn->flag=1;
		P1 = (btn->wait_time_s*1000);
		
	}
	else if( btn->klock && key_press ) {
		(btn->klock)++;
		if( !btn->klock ) {
			P1=0;
			btn->flag=0;
		}
		} else if( btn->flag && !P1 ) {
		// reakcja na d³u¿sze wciniêcie klawisza
		if(btn->kfun2) btn->kfun2();
		btn->flag=0;
	}
}

static inline void lcd_wyslijPOlBajtu (uint8_t data)
{


	if(data&(1<<0)){
		PORT(LCD_D4PORT) |= (1<<LCD_D4);
	}
	else
	{
		PORT(LCD_D4PORT) &=~(1<<LCD_D4);
	}
	if(data&(1<<1)){
		PORT(LCD_D5PORT) |= (1<<LCD_D5);
	}
	else
	{
		PORT(LCD_D5PORT) &=~(1<<LCD_D5);
	}
	if(data&(1<<2)){
		PORT(LCD_D6PORT) |= (1<<LCD_D6);
	}
	else
	{
		PORT(LCD_D6PORT) &=~(1<<LCD_D6);
	}
	if(data&(1<<3)){
		PORT(LCD_D7PORT) |= (1<<LCD_D7);
	}
	else
	{
		PORT(LCD_D7PORT) &=~(1<<LCD_D7);
	}
}
void lcd_wyslijBajt (unsigned char _data)
{
	E_ON;
	lcd_wyslijPOlBajtu(_data>>4);
	E_OFF;
	
	E_ON;
	lcd_wyslijPOlBajtu(_data);
	E_OFF;
	
	_delay_us(120);
}
void lcd_wyslijKomende (uint8_t cmd)
{
	RS_OFF;
	lcd_wyslijBajt(cmd);
}
void lcd_wyslijDane (uint8_t data)
{
	RS_ON;
	lcd_wyslijBajt(data);
}
void lcd_clear (void)
{
	lcd_wyslijKomende(LCD_COMM_CLEAR);
	_delay_ms(10);
}
void lcd_second_line()
{
	lcd_wyslijKomende(0b11000000);
	_delay_ms(15);
}
void lcd_init (void)
{
	DDR(LCD_D7PORT)|= (1<<LCD_D7);
	DDR(LCD_D6PORT)|= (1<<LCD_D6);
	DDR(LCD_D5PORT)|= (1<<LCD_D5);
	DDR(LCD_D4PORT)|= (1<<LCD_D4);
	DDR(LCD_RSPORT)|= (1<<LCD_RS);
	DDR(LCD_EPORT) |= (1<<LCD_E);
	
	RS_ON;
	E_ON;
	
	_delay_ms(15);
	E_OFF;
	RS_OFF;
	
	E_ON;
	lcd_wyslijPOlBajtu(0b0011);
	E_OFF;
	_delay_ms(4.1);
	
	E_ON;
	lcd_wyslijPOlBajtu(0b0011);
	E_OFF;
	_delay_us(100);
	
	E_ON;
	lcd_wyslijPOlBajtu(0b0011);
	E_OFF;
	_delay_us(100);
	
	//tryb czterobitowy, 2 wiersze, znak 5x7
	lcd_wyslijKomende(LCD_COMM_FUNC | LCD_COMM_4BIT | LCD_COMM_2WIERSZE | LCD_COMM_5x7);

	// w³¹czenie kursora
	lcd_wyslijKomende(LCD_COMM_ONOFF| LCD_COMM_KURSOROFF);
	
	//w³¹czenie wywietlacza
	lcd_wyslijKomende(LCD_COMM_ONOFF| LCD_COMM_DISPLAYON);
	
	//przesuwanie kursora w prawo bez przesuwania zawartoci ekranu
	lcd_wyslijKomende(LCD_COMM_ENTRY | LCD_COMM_ENTRYR);
	
	//czyszczenie ekranu
	lcd_wyslijKomende(LCD_COMM_CLEAR);
}
void lcd_home(void)
{
	lcd_wyslijKomende(LCD_COMM_HOME);
	_delay_ms(10);
}
void lcd_kurson_on(void)
{
	lcd_wyslijKomende(LCD_COMM_KURSORON);
	_delay_ms(10);
}
void lcd_kurson_off(void)
{
	lcd_wyslijKomende(LCD_COMM_KURSOROFF);
	_delay_ms(10);
}
void lcd_blink_on(void)
{
	lcd_wyslijKomende(LCD_COMM_BLINKON);
	_delay_ms(10);
}
void lcd_blink_off(void)
{
	lcd_wyslijKomende(LCD_COMM_BLINKOFF);
	_delay_ms(10);
}
void lcd_display_on(void)
{
	lcd_wyslijKomende(LCD_COMM_DISPLAYON);
	_delay_ms(10);
}
void lcd_display_off(void)
{
	lcd_wyslijKomende(LCD_COMM_DISPLAYOFF);
	_delay_ms(10);
}
void lcd_char(char znak)
{
	lcd_wyslijDane(znak);
} // //wyswietlenie znaku
void lcd_string(char * napis)
{
	register char znak;
	while((znak=*(napis++)))
	{
		lcd_char(znak);
	}
} //wyswietlanie napisu
void lcd_int(int liczba)
{
	char bufor[17];
	lcd_string (itoa (liczba,bufor,10));
}
void lcd_locate(uint8_t y, uint8_t x)
{
	switch(y)
	{
		case 1: y = LCD_WIERSZ1; break;
		case 2: y = LCD_WIERSZ2; break;
	}
	lcd_wyslijKomende(LCD_COMM_DDRAM + y + x);
}
void init_timer()
{
	
	TCCR0 |= (1<<COM00); //Zmiana stanu wyjcia OC1A na niski przy porównaniu A
	TCCR0 &= ~(1<<COM01);		  //Zmiana stanu wyjcia OC1B na niski przy porównaniu B
	TCCR0 |= (1<<WGM01);    //ctc
	TCCR0 &= ~(1<<WGM00);
	TCCR0 &= ~(1<<FOC0);        //fors a³tput komper
	
	TCCR0 |= (1<<CS00) | (1<<CS01); 
	TCCR0 &= ~(1<<CS02); // preskaler 64
	
	TIMSK |= (1<<OCIE0);
	
	OCR0=250;    //Wartoæ pocz¹tkowa porównania A (Wyjcie OC1A - PB1),	
}
void pwm()
{
	DDRD=0xFF;
	
	TCCR1A |= (1<<WGM10) | (1<<WGM11) | (1<<COM1B1); 
	TCCR1A &= ~(1<<COM1B0);
	TCCR1A &= ~(1<<COM1A0);
	TCCR1A &= ~(1<<COM1A1);

	
	TCCR1B &= ~(1<<CS10);
	TCCR1B &= ~(1<<CS12);
	TCCR1B |= (1<<CS11) | (1<<WGM13) | (1<<WGM12);
	
	
	OCR1A = 39999;
}
void przetwornik(void)
{
	//Inicjalizacja ADC
	ADCSRA |= (1<<ADEN) //ADEN=1 (w³¹czenie przetwornika ADC)
			|(1<<ADPS0) // ustawienie preskalera na 128
			|(1<<ADPS1)
			|(1<<ADPS2);
	
	ADMUX  =    (1<<REFS0) | (0<<REFS1)          //VCC jako napiêcie referencyjne
				|(1<<MUX2) | (1<<MUX0);			 //wybór kana³u pomiarowego (ADC5 - Pin 5)
	ADMUX &= ~(1<<ADLAR);
	
	DDRA &= ~(1<<ADCIN);               //Ustawienie Wejcia ADC
}

/*****************************************************************************************

                   ***** PROCEDURY PRZERWAN *****

*****************************************************************************************/

ISR(TIMER0_COMP_vect)
{
	uint16_t n;
	
	n = Timer1;
	if (n) Timer1 = --n;
	
	n = Timer2;
	if (n) Timer2 = --n;
	
	n = Timer3;
	if (n) Timer3 = --n;
	
	 n = P1;
	if (n) P1 = --n;
}
