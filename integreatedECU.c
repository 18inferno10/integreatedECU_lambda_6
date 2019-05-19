#include <avr/io.h>
#include<util/delay.h>
#include<avr/interrupt.h>

#define SET_BIT(PORT,BIT) PORT|=(1<<BIT)
#define CLR_BIT(PORT ,BIT) PORT&=~(1<<BIT)

// port B is selected as LCD data port
#define LCD_DATA PORTB
// port D is selected as LCD command port
#define ctrl PORTD
// read/write signal is connected to port D pin 6
#define rw PD6
// register select signal is connected to port D pin 5
#define rs PD5
// enable signal is connected to port D pin 7
#define en PD7

unsigned int Head_light=0,Indicator=0;
volatile uint8_t parking_flag=0, brake_flag=0;
volatile uint16_t x=0,front=0,rear=0;

void pins_init()
{
CLR_BIT(DDRD,PD1);
SET_BIT(PORTD,PD1);
CLR_BIT(DDRD,PD2);
SET_BIT(PORTD,PD2);
CLR_BIT(DDRD,PD3);
SET_BIT(PORTD,PD3);

//LCD PINS INIT
SET_BIT(DDRB,PB0);
SET_BIT(DDRB,PB1);
SET_BIT(DDRB,PB2);
SET_BIT(DDRB,PB3);
SET_BIT(DDRB,PB4);
SET_BIT(DDRB,PB5);
SET_BIT(DDRB,PB6);
SET_BIT(DDRB,PB7);
SET_BIT(DDRD,PD5);
SET_BIT(DDRD,PD6);
SET_BIT(DDRD,PD3);
// exterior Lighting System Pins
CLR_BIT(DDRC,PC0); //Brake_SW
SET_BIT(PORTC,PC0);
CLR_BIT(DDRD,PD0); //Parking_SW
SET_BIT(PORTD,PD0);
//// tpms ///
CLR_BIT(DDRC,PC3);
CLR_BIT(DDRC,PC4);
SET_BIT(PORTC,PC3);
SET_BIT(PORTC,PC4);

SET_BIT(PORTC,PC6);
CLR_BIT(PORT,PC6);
}

void init_MyInterrupt()
{
    //interupts enabling for codeBlocks
    PCMSK2|=(1<<PCINT16); //parking_SW
    PCICR|=(1<<PCIE2);

    PCMSK0|=(1<<PCINT8); //Brake_SW
    PCICR|=(1<<PCIE0);
    sei();

}

void initADC()
{
    ADMUX=(1<<REFS0);
    ADCSRA=(1<<ADEN)|(7<<ADPS0);
}

uint16_t readADC(uint8_t ch)
{
    ADMUX&=0xf8;
    ch=ch&0b00000111;
    ADMUX|=ch;
    ADCSRA|=(1<<ADSC);
    while(!(ADCSRA&(1<<ADIF)));
    ADCSRA|=(1<<ADIF);
    return(ADC);
}

void init_LCD(void)
{
// initialization in 8bit mode of 16X2 LCD
LCD_cmd(0x38);
_delay_ms(1);
// make clear LCD
LCD_cmd(0x01);
_delay_ms(1);
// return home
LCD_cmd(0x02);
_delay_ms(1);
 // make increment in cursor
LCD_cmd(0x06);
_delay_ms(1);
// “8” go to first line and “0” is for 0th position
LCD_cmd(0x80);
_delay_ms(1);
return;
}
void LCD_cmd(unsigned char cmd)
{
LCD_DATA = cmd;      // data lines are set to send command
PORTD  &= ~(1<<rs);  // RS sets 0
PORTD  &= ~(1<<rw);  // RW sets 0
PORTD  |= (1<<en);   // make enable from high to low
_delay_ms(100);
PORTD  &= ~(1<<en);
}
void LCD_write(unsigned char data)
{
LCD_DATA= data;       // data lines are set to send command
PORTD  |= (1<<rs);    // RS sets 1
PORTD  &= ~(1<<rw);   // RW sets 0
PORTD  |= (1<<en);    // make enable from high to low
_delay_ms(100);
PORTD &= ~(1<<en);
return ;
}
void LCD_String (char *str)
{
     int i;
     for(i=0;str[i]!=0;i++)   //send each char of string till the NULL
     {
        LCD_write(str[i]);  // call LCD data write
      }
}


int main()
{

    pins_initialisation();
  	initADC();
  	init_MyInterrupt();
  	init_LCD();  // initialize LCD
	_delay_ms(100); // delay of 100 Milli seconds
	LCD_cmd(0x0C);  // display on, cursor off
	_delay_ms(100);
	LCD_cmd(0x0E);

while(1)
{
    Range_cal=ReadADC(5);
    _delay_ms(20);

    Indicator=readADC(2);
    _delay_ms(20);

    Head_light=readADC(1);
    _delay_ms(20);

    front=readADC(2);
    _delay_ms(20);
    rear=readADC(4);
    _delay_ms(20);


    ///// temperature Control ////
    if(PIND&(1<<PD1) || PIND&(1<<PD2))
    {
        LCD_String("AC ON");
        _delay_ms(20);
             LCD_cmd(0x01);
    }
    //////// wireless Door Unlock /////////
    if(Range_cal<500)
    {
        if(PIND&(1<<PD3))
        {
                LCD_String("Door Unlocked");
               _delay_ms(20);
                LCD_cmd(0x01);
        }

    }

    /////////// exterior lighting system ///////////

            //If parking switch is pressed
             if(parking_flag==1)
            {
                LCD_String("PARKING ON");//****8
               _delay_ms(20);
                LCD_cmd(0x01);
            }

            //If BRAKE switch is pressed
            if(brake_flag==1)
            {
              LCD_String("BRAKE ON");
               _delay_ms(20);
              LCD_cmd(0x01);
            }

            if(Head_light<400)
            { LCD_String("HL - Low Beam");
    		 _delay_ms(20);
    		 LCD_cmd(0x01);
			}
            if(Head_light>600)
            { LCD_String("HL-High Beam");
    		 _delay_ms(20);
             LCD_cmd(0x01);
			}

            if(Indicator<400)
            {
              LCD_String ("LeftIndicatorON");
              _delay_ms(20);
             LCD_cmd(0x01);
			}
            if(Indicator>600)
            {
              LCD_String("RightIndicatorON");
             _delay_ms(20);
             LCD_cmd(0x01);
			}

            /////// TPMS //////////
            if((front<=214)&&(rear>=214))
            {

                LCD_String ("Front Tire Low Pressure");
                _delay_ms(20);
                SET_BIT(PORTC,PC6); //led on
                _delay_ms(20);
                LCD_cmd(0x01);
            }

            if((front>=214)&&(rear<=214))
            {
                LCD_String ("Rear Tire Low Pressure");
                _delay_ms(20);
                SET_BIT(PORTC,PC6); //led on
                _delay_ms(20);
                LCD_cmd(0x01);
            }

return 0;
}
ISR(PCINT2_vect)
{
    parking_flag=!parking_flag;
    parking_flag=1;
}
ISR(PCINT0_vect)
{
  	brake_flag=!brake_flag;
  	brake_flag=1;
}

