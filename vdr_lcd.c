/*************************************************************************
Title:    VDR-LCD - ein HD44780 basiertes serialles LCD fuer vdr-lcdproc
Author:   Sven Sperner <cethss@gmail.com>
Software: GCC 4.2.4
	  ./configure --target=avr --prefix=/usr/local --enable-languages=c --disable-libssp --enable-__cxa_atexit --enable-clocale=gnu --disable-nls --with-dwarf2 --with-gmp=/usr/local --with-mpfr=/
Hardware: Atmel AtTiny2313, 16MHz
          Nana Ya lmm62s125a1d
          UART 9600,8,n,1 (Max232)
**************************************************************************/

// wenn die CPU/Quarz-Frequenz noch nicht angegeben wurde
#ifndef F_CPU
    //#define F_CPU 	1000000UL	// 1MHz int.Osc. div.8 (Fuse)
    //#define F_CPU 	8000000UL	// 8MHz internal Oscillator
    #define F_CPU 	16000000UL	// 16MHz Crystal
    //#define F_CPU 	14745600UL	// 16MHz Crystal
#endif
//#define UART_BAUD_RATE	9600    // 9600 baud
#define UART_BAUD_RATE	19200   // 19200 baud

#include <avr/pgmspace.h>
#include <avr/io.h>
#include <stdlib.h>
#include <string.h>
#include <util/delay.h>

#include "lcd.h"
#include "uart.h"


void delay_ms(int ms)
{
  int t;
  for( t=0 ; t<=ms ; t++ )
	_delay_ms(1); 
}

void delay_us(int us)					
{
	TCNT0=0;
	while(TCNT0<us);
}


int main(void)
{
  char uart_in = 0;		//Uart Buffer
  char cmd_data = 0;		// Command or Data
  unsigned int cursor_address = 0;
  unsigned int tmp;
    
    char buff[7];
/*
    //PWM fuer Kontrast und Hintergrundbeleuchtung
    TCCR1A = (1<<WGM10)|(1<<COM1A1)|(1<<COM1B1);	//Standard-PWM...
    TCCR1B = (1<<CS10);
    TCCR3A = (1<<WGM10)|(1<<COM1A1)|(1<<COM1B1);	//Standard-PWM...
    TCCR3B = (1<<CS10);
    OCR1A=23;			//Kontrast3
    OCR3A=100;			//Hintergrundbeleuchtung2
    DDRD |= (1<<PD4)|(1<<PD5);	//OC3A+OC1A -> Ausgänge

    PORTB |= (1 << 4); 			// Pullup fӮr PD2 aktivieren
    PORTD |= (1 << 5); 			// Pullup fӮr PD3 aktivieren

    //DDRB |= ~(1 << 2);	// PB2 output (PWM / OC0A)
    //DDRB |= ~(1 << 3);	// PB3 output (PWM / OC1A)
    DDRB |= ~(1 << 4);		// PB4 output (PWM / OC1B)
    DDRD |= ~(1 << 5);		// PD5 output (PWM / OC0B)
    // - Timer0 fuer Hintergrundbeleuchtung
    TCCR0A = 0;
    TCCR0A |= (1<<COM0B1|1<<WGM01|1<<WGM00);	// FastPWM,OC0A disconected
    TCCR0B |= (1<<CS00);	// CLK,NoPrescaling
    TCNT0 = 0x00;		// Timer zurӮcksetzen
    //OCR0A = 0xff;		// -> PB2
    OCR0B = 0x00;		// -> PD5
    // - Timer1 fuer Kontrast
    TCCR1A = 0;
    TCCR1A |= (1<<COM1B1|1<<WGM12|1<<WGM11|1<<WGM10);	// FastPWM,OC1A disconected
    TCCR1B |= (1<<CS10);	// CLK,NoPrescaling
    TCNT1 = 0x0000;		// Timer zurӮcksetzen
    //OCR1A = 0x0000;		// -> PB3 (nicht verdrahtet)
    OCR1B = 0x0000;		// -> PB4 (nicht verdrahtet)
    TIMSK = (_BV (TOIE1) | _BV (TOIE0));
*/

    //Eingaenge/Taster aktivieren
    DDRD &= ~(1 << PD2);	// PD2 set to input (int0)
    DDRD &= ~(1 << PD3);	// PD3 set to input (int1)
    DDRD &= ~(1 << PD4);	// PD4 set to input
    DDRD &= ~(1 << PD5);	// PD5 set to input
    DDRD &= ~(1 << PD6);	// PD6 set to input
    PORTD |= (1 << PD2); 	// Pullup an PD2 aktivieren
    PORTD |= (1 << PD3); 	// Pullup an PD3 aktivieren
    PORTD |= (1 << PD4); 	// Pullup an PD4 aktivieren
    PORTD |= (1 << PD5); 	// Pullup an PD5 aktivieren
    PORTD |= (1 << PD6); 	// Pullup an PD6 aktivieren
    //GIMSK |= (1<<INT0|1<<INT1);	// enable external Interrupts

    //Initialisiere UART
    sei();		// UART ist Int-gesteuert
    uart_init( UART_BAUD_SELECT(UART_BAUD_RATE,F_CPU) );
    uart_puts_P("UART0 is up and running...\r\n");

    //Initialisiere LCD
    lcd_init(LCD_DISP_ON);
    lcd_clrscr();
    lcd_command(LCD_DISP_ON_CURSOR);
    lcd_puts_P("VDR-LCD is up...");
    delay_ms(5000);
    lcd_clrscr();
    lcd_home();
    

    //Main-Loop
    for (;;)
    {
	if ( !( PIND & (1<<PIND2) ) ) {
	  uart_putc(0x30);
	  delay_ms(150);
	}
	if ( !( PIND & (1<<PIND3) ) ) {
	  uart_putc(0x31);
	  delay_ms(150);
	}
	if ( !( PIND & (1<<PIND4) ) ) {
	  uart_putc(0x32);
	  delay_ms(150);
	}
	if ( !( PIND & (1<<PIND5) ) ) {
	  uart_putc(0x33);
	  delay_ms(150);
	}
	if ( !( PIND & (1<<PIND6) ) ) {
	  uart_putc(0x34);
	  delay_ms(150);
	}
	//uart_putc( '!' );
        uart_in = uart_getc();
        if ( uart_in & UART_NO_DATA )
        {
		//GLCD_TextGoTo(250,22);
		//GLCD_WriteText( "UART-NoData" );
        }
        else
        {
            if ( uart_in & UART_FRAME_ERROR )
            {
                //uart_puts_P("UART Frame Error: ");
            }
            else if ( uart_in & UART_OVERRUN_ERROR )
            {
                //uart_puts_P("UART Overrun Error: ");
            }
            else if ( uart_in & UART_BUFFER_OVERFLOW )
            {
                //uart_puts_P("Buffer overflow error: ");
            }
	    else if ( cmd_data != 0 && uart_in != 0 )
	    {
		if ( cmd_data == 0x11 )	//PicAnLCD Command-Code
		{
		    if( !(uart_in & 0x30) )	//kein 8BitMode!
			lcd_command( uart_in );
		    else
			lcd_command( 0x2F );	//->4Bit,2L,5x10
		    cmd_data = 0; uart_in = 0;
		}//Command
		if ( cmd_data == 0x12 )	//PicAnLCD Data-Code
		{
		    lcd_data( uart_in );
		    cmd_data = 0; uart_in = 0;
		}//Data
		if ( cmd_data == 0xFE )	//los,pert,lcdser - instruction
		{
		    if( !(uart_in & 0x30) )	//kein 8BitMode!
			lcd_command( uart_in );
		    else
			lcd_command( 0x2F );	//->4Bit,2L,5x10
		    cmd_data = 0; uart_in = 0;
		}//los,pert,lcdser,vdrlcd
		if ( cmd_data == 0xC0 )	//VDR-Wakeup Command-Code
		{
		    if( !(uart_in & 0x30) )	//kein 8BitMode!
			lcd_command( uart_in );
		    else
			lcd_command( 0x2F );	//->4Bit,2L,5x10
		    cmd_data = 0; uart_in = 0;
		}//Command-WakeUp
		if ( cmd_data == 0xC4 )	//VDR-Wakeup Data-Code
		{
		    lcd_data( uart_in );
		    cmd_data = 0; uart_in = 0;
		}//Data-WakeUp
		if ( cmd_data == 0xC8 )	//VDR-Wakeup Backlight On
		{
		}//WakeUp-BlOn
		if ( cmd_data == 0xC9 )	//VDR-Wakeup Backlight Off
		{
		}//WakeUp-BlOff
		if ( cmd_data == 0xCF )	//VDR-Wakeup End
		{
		    lcd_clrscr();
		    lcd_home();
		}//WakeUp-End
		if ( cmd_data == 0x01 )	//Cursor(picanlcd)
		{
		    if( uart_in > 120 )
			lcd_gotoxy( (uart_in - 121), 1 );
		    else
			lcd_gotoxy( uart_in, 0 );
		    cmd_data = 0; uart_in = 0;
		}//Cursor
		if ( cmd_data == 0x10 )	//Uart-Out
		{
		    uart_putc( uart_in );
		    cmd_data = 0; uart_in = 0;
		}//Uart-Out
	    }//C-D
	    else if ( uart_in==0x11 || uart_in==0x12 ||	//picanlcd cmd, data
		      uart_in==0x01 ||			//picanlcd cursor
	              uart_in==0xFE ||	//los,lcdserializer,vdr-lcd,pertelian cmd
		      uart_in==0xC0 || uart_in==0xC4 ||	//vdr-wakeup cmd,data
		      uart_in==0xC8 || uart_in==0xC9 ||	//vdr-wakeup bl on/off
		      uart_in==0xCF ||			//vdr-wakeup end
		      uart_in==0x10 )			//Uart-Out (Test)
	    {
		cmd_data = uart_in;
		uart_in = 0;
	    }
	    //else if( c > 0xa0 && c < 0xff )	//japanese Chars
	    else if( uart_in > 0x1f && uart_in < 0x80 )	//regular Chars 32-127
	    {
		lcd_putc( uart_in );
	    }
	    else switch( uart_in )
	    {
		//case 0x01:	//(Cursor)
		case 0x02:	//HomeCursor
		  lcd_home();
		  break;
		case 0x03:	//CursorLeft
		  lcd_command(LCD_MOVE_CURSOR_LEFT);
		  break;
		case 0x04:	//CursorRight
		  lcd_command(LCD_MOVE_CURSOR_RIGHT);
		  break;
		case 0x05:	//SaveCursorPosition
		  cursor_address = lcd_getxy();
		  break;
		case 0x06:	//RestoreCursorPosition
		  if( cursor_address > 120 )
		      lcd_gotoxy( (cursor_address - 121), 1 );
		  else
		      lcd_gotoxy( cursor_address, 0 );
		  break;
		case 0x07:	//Bell
		  lcd_home();lcd_puts_P("Bell            ");delay_ms(10000);
		  break;
		case 0x08:	//Backspace
		  lcd_command(LCD_MOVE_CURSOR_LEFT);
		  lcd_putc(' ');
		  lcd_command(LCD_MOVE_CURSOR_LEFT);
		  break;
		case 0x09:	//HorizontalTab
		  tmp = lcd_getxy();
		  if( tmp > 120 )
		      lcd_gotoxy( (tmp - 117), 1 );
		  else
		      lcd_gotoxy( tmp + 4, 0 );
		  break;
		case 0x0A:	//LineFeed
		  tmp = lcd_getxy();
		  if( tmp > 120 )
		  {
		      lcd_scrollup();
		      lcd_gotoxy( tmp - 121, 1 );
		  }
		  else
		      lcd_gotoxy( tmp, 1 );
		  break;
		case 0x0B:	//VertikalTab
		  lcd_scrollup();
		  break;
		case 0x0C:	//FormFeed
		  lcd_clrscr();
		  lcd_home();
		  break;
		case 0x0D:	//CarriageReturn
		  tmp = lcd_getxy();
		  if( tmp > 120 )
		      lcd_gotoxy( 0, 1 );
		  else
		      lcd_home();
		  break;
		case 0x0E:	//ShiftDisplayLeft
		  lcd_command(LCD_MOVE_DISP_LEFT);
		  break;
		case 0x0F:	//ShiftDisplayRight
		  lcd_command(LCD_MOVE_DISP_RIGHT);
		  break;
		//case 0x10:	//(Uart-Out)
		//case 0x11:	//(Command)
		//case 0x12:	//(Data)
		case 0x13:	//SetNumberOfLines
		  lcd_home();lcd_puts_P("SetNumberOfLines");delay_ms(10000);
		  break;
//		case 0x15:	//GeneralPurposeOutput
//		  lcd_home();lcd_puts_P("GeneralPurposeO.");delay_ms(10000);
//		  break;
		case 0x16:	//PrintSignedDecimalNumber
		  lcd_home();lcd_puts_P("PrintSignedDezim");delay_ms(10000);
		  break;
		case 0x17:	//PrintUnsignedDecimalNumber
		  lcd_home();lcd_puts_P("PrintUnsignedDez");delay_ms(10000);
		  break;
		case 0x19:	//SetCursorDisplayOption
		  lcd_home();lcd_puts_P("SetCursorDispOpt");delay_ms(10000);
		  break;
	    }//switch(c)
	    uart_in = 0;

        } //else(UART_DATA)        

    } //for(;;)
    
} //main()











