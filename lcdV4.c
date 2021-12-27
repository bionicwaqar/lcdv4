#include "TM4C123.h" /* include register defintion file of TM4C123GH6PM */
#define LCD GPIOB   /* Define "LCD" as a symbolic name for GPIOB */
#define RS 0x01 /* PORTB BIT5 mask */
#define RW 0x02 /* PORTB BIT6 mask */
#define EN 0x04 /* PORTB BIT7 mask */
#define HIGH 1
#define LOW 0

/*define useful symbolic names for LCD commands */
#define clear_display     0x01
#define returnHome        0x02
#define moveCursorRight   0x06
#define moveCursorLeft    0x08
#define shiftDisplayRight 0x1C
#define shiftDisplayLeft  0x18
#define cursorBlink       0x0F
#define cursorOff         0x0C
#define cursorOn          0x0E
#define Function_set_4bit 0x28
#define Function_set_8bit 0x38
#define Entry_mode        0x06
#define Function_8_bit    0x32
#define Set5x7FontSize    0x20
#define FirstRow          0x80

/*
//---SYSTEM CONTROL REGISTERS---LED//
#define SYSCTL_RCGCGPIO_R (*((volatile unsigned long *) 0x400FE608))

#define GPIO_PORTF_DEN_R (*((volatile unsigned long *) 0x4002551C))

#define GPIO_PORTF_DIR_R (*((volatile unsigned long *) 0x40025400))

#define GPIO_PORTF_DATA_R (*((volatile unsigned long *) 0x40025038))

#define GPIO_PORTF_CLK_EN 0x20

#define GPIO_PORTF_PIN1_EN 0x02

#define GPIO_PORTF_PIN2_EN 0x04

#define GPIO_PORTF_PIN3_EN 0x08

#define LED_ON1 0x02

#define LED_ON2 0x04

#define LED_ON3 0x08

*/

#define DELAY_VALUE 9000
void Delay(unsigned long );
void delay(unsigned long);
void Delay(unsigned long n)

{

volatile unsigned long i;

for(i=0;i<n;i++);

}



/* KEYPAD*/
void Init_GPIO_C(void);
unsigned char scan_keypad(void);
	
void Init_GPIO_C()
{
	SYSCTL->RCGCGPIO |= 0x24; //PORTC & F clock
	GPIOC->DIR |=0x00; //PORTC rows 
	GPIOC->PDR |=0xFF;
	GPIOC->DEN |=0xFF;
	
	GPIOF->DIR |=0xFF; //PORTF colummns
	GPIOF->DEN |=0xFF;
}
static int c=0;
static unsigned int col;
static unsigned const char key[4][4] ={		//KEYPAD INTERFACE
									{'1','2','3','A'},
									{'4','5','6','B'},
									{'7','8','9','C'},
									{'*','0','#','D'}
								};

								/* prototypes of LCD functions */
void delay_ms(int n); /* mili second delay function */
void delay_us(int n); /* micro second delay function */ 
void LCD_init(void);  /* LCD initialization function */
void LCD_Cmd(unsigned char command); /*Used to send commands to LCD */
void LCD_Write_Char(unsigned char data); /* Writes ASCII character */
void LCD_Write_Nibble(unsigned char data, unsigned char control); /* Writes 4-bits */
void LCD_String (char *str);	/* Send string to LCD function */


unsigned char scan_keypad(void)
{
	unsigned int i=0,j=0,x=0,y=0;
	//int i=0;
//	unsigned char input='x';
//	unsigned char read=(unsigned char)GPIOC->DATA;
//	unsigned char read1;
	
	while(1) {		
//		read1=GPIOC->DATA&0xF0;
//	read1=read1>>3;
//	GPIOF->DATA = read1;
//	return read1;
		
/*read=(unsigned char)GPIOC->DATA;
		Delay(50);	
read=~read;
read=read>>3;
GPIOF->DATA=read;
		
		
Delay(50);	
*/		
		
//read=read>>3;
		
	for( i=0;i<4;i++)	//COLUMN TRAVERSAL
	{
		GPIOF->DATA = 0x01<<(i+1);
	//	read1=(unsigned char)GPIOF->DATA;
	//	read1= (unsigned char)(read1<<3) ;
	//	Delay(100);
	//	read=(unsigned char)GPIOC->DATA;
		Delay(50000);	
	//	read=~read;
		//	GPIOF->DATA = 0x04;
		
		
		for( j=0;j<4;j++)
		{
			//Delay(4500);
			if(((GPIOC->DATA)&0xF0)&(1U<<(j+4)))
			{
				x=j;
				y=i;
				j=0;
				i=0;
				
				return key[x][y];
			//	LCD_Write_Char(key[x][y]);
			//Delay(18000);
			}
		//	else LCD_Write_Char('X');
			
		}
	
/*		if(~(GPIOC->DATA)& 0x10)
		{
				while(~(GPIOC->DATA) & 0x10){}
		//	input= key[0][i];	
			//	input='1';
		//	if(read1&0x10)
		//		return  key[0][0];
		//	if(read1&0x20)
			//	return key[0][1];
		//	else return key[0][2];
					return key[0][i];
		}
		 if((~(GPIOC->DATA)) & 0x20)
		{
			while((~(GPIOC->DATA)) & 0x20){}
		//	input= key[1][i];
		//		input='2';
		//		return '2';
				return key[1][i];
		}
		 if((~(GPIOC->DATA)) & 0x40)
		{
			while((~(GPIOC->DATA)) & 0x40){}
			//input= key[2][i];
			//	input='3';
				return '3';
		}
		 if((~(GPIOC->DATA)) & 0x80)
		{
			while((~(GPIOC->DATA)) & 0x80){}
		//	input= key[3][i];
			//	input='4';
				return '4';
				
		}
		//Delay(9000);
		//if(input!='x')*/
			
	}
	
	//return input;
	
}
}



#define DELAY_VALUE 9000
void Delay(unsigned long );
void delay(unsigned long);



int main()
{
	/*LED BLINKING CODE
	SYSCTL_RCGCGPIO_R |= GPIO_PORTF_CLK_EN; //enable clock for PORTF

GPIO_PORTF_DEN_R |= GPIO_PORTF_PIN1_EN; //enable pins 1 on PORTF

GPIO_PORTF_DIR_R |= GPIO_PORTF_PIN1_EN; //make pins 1 as output pins

GPIO_PORTF_DEN_R |= GPIO_PORTF_PIN2_EN; //enable pins 2 on PORTF

GPIO_PORTF_DIR_R |= GPIO_PORTF_PIN2_EN; //make pins 2 as output pins

GPIO_PORTF_DEN_R |= GPIO_PORTF_PIN3_EN; //enable pins 3 on PORTF

GPIO_PORTF_DIR_R |= GPIO_PORTF_PIN3_EN; //make pins 3 as output pins
*/	
	
	//LCD CODE
	 LCD_init();
	Init_GPIO_C();
unsigned char input1='\0';
	//GPIOF->DATA=0x01;
	while(1)
   {
	 	 
 	LCD_Cmd(clear_display);
  LCD_Cmd(FirstRow); /* Force cusor to begining of first row */
// delay_ms(500);
	//	 for(unsigned int i=1;i<=250;i++)
	//{
	//	Delay(9000);	
	//}
	/*
for (unsigned char i=0;i<4;i++)
{
	for (unsigned char j=0;j<4;j++)
		LCD_Write_Char(key[i][j]);
	 for(unsigned int k=1;k<=250;k++)
	{
		Delay(9000);	
	}
}	
	*/
 
// LCD_Write_Char('L');
 //delay_ms(500); 
 LCD_Write_Char('K');
 LCD_Write_Char('E');
 LCD_Write_Char('Y');
 LCD_Write_Char(':');
 LCD_Write_Char('-');
		 for(unsigned int i=1;i<=250;i++)
	{
		Delay(9000);	
	}
input1=scan_keypad();
LCD_Write_Char(input1);
//	GPIOF->DATA=GPIOF->DATA+1;
  	 for(unsigned int i=1;i<=250;i++)
	{
		Delay(27000);	
	}
	// 	 LCD_Cmd(0x71);
	//	 LCD_Write_Char(0x71);
		Delay(24);	
	//delay_us(40);
	//	LCD_Cmd(0xFC);
	//delay_us(40);


   }
 
 
}

/* LCD and GPIOB initialization Function */ 
void LCD_init(void)
{
 SYSCTL->RCGCGPIO |=(1<<1); /* Enable Clock to GPIOB */
 LCD->DIR |=0xFF; /* Set GPIOB all pins a digital output pins */
 LCD->DEN |=0xFF; /* Declare GPIOB pins as digital pins */

 LCD_Cmd(Set5x7FontSize);  /* select 5x7 font size and 2 rows of LCD */
 LCD_Cmd(Function_set_4bit); /* Select 4-bit Mode of LCD */
 LCD_Cmd(moveCursorRight); /* shift cursor right */
 LCD_Cmd(clear_display); /* clear whatever is written on display */
 LCD_Cmd(cursorBlink);  /* Enable Display and cursor blinking */
 
}

void LCD_Cmd(unsigned char command)
{
    LCD_Write_Nibble(command & 0xF0, 0);   /* Write upper nibble to LCD */
    LCD_Write_Nibble((command << 4)&0xF0, 0);     /* Write lower nibble to LCD */
    
    if (command < 4)
     //   delay_ms(2);         /* 2ms delay for commands 1 and 2 */
				Delay(9000);
		//Delay();
    else
     //   delay_us(40);        /* 40us delay for other commands */
		Delay(24);
		
		
		
}


void LCD_Write_Nibble(unsigned char data, unsigned char control)
{

    data &= 0xF0;       /* Extract upper nibble for data */
    control &= 0x0F;    /* Extract lower nibble for control */
  //  LCD->DATA = data | control;       /* Set RS and R/W to zero for write operation */
     LCD->DATA = data | control | EN;  /* Provide Pulse to Enable pin to perform wite operation */
	// LCD->DATA = data | control; 
  //delay_ms(20000);   
	Delay(9000);
	//	delay_us(2000);
 // LCD->DATA = data; /*Send data */
		LCD->DATA = data | control;
  //  LCD->DATA = 0; /* stop writing data to LCD */
}
void LCD_Write_Char(unsigned char data)
{
    LCD_Write_Nibble(data & 0xF0, RS);    /* Write upper nibble to LCD and RS = 1 to write data */
    LCD_Write_Nibble((data << 4)&0xF0, RS);      /* Write lower nibble to LCD and RS = 1 to write data */
   // delay_us(40);
		Delay(24);
	/* GPIO_PORTF_DATA_R = 0x02; /
	/Turn on RED LED

	Delay(); //Delay almost 1 sec

	GPIO_PORTF_DATA_R = 0x00; //Turn off LED

	Delay(); //Delay almost 1 sec
	
	*/
}


void LCD_String (char *str)	/* Send string to LCD function */
{
	int i;
	for(i=0;str[i]!=0;i++)  /* Send each char of string till the NULL */
	{
		LCD_Write_Char(str[i]);  /* Call LCD data write */
	}
}

/* Mili seconds delay function /*
/*void delay_ms(int n)
{
 int i,j;
 for(i=0;i<n;i++)
	{
 for(j=0;j<3180;j++)
 {}
 }
}
*/

void delay_ms(int n)
{
 int i,j;
 for(i=0;i<n;i++)
	
 for(j=0;j<31800;j++);
 
 }

/* Micro seconds delay function */
void delay_us(int n)
{
 int i,j;
 for(i=0;i<n;i++)
 for(j=0;j<3;j++);
 
 
}

