#include <p18f4620.h>
#include <stdio.h>
#include <stdlib.h>
#include <xc.h>
#include <math.h>
#include <usart.h>

#pragma config OSC      =   INTIO67  
#pragma config WDT      =   OFF
#pragma config LVP      =   OFF
#pragma config BOREN    =   OFF

#define DS3231_ID       0x68
#define DS3231_Add_00   0x00
#define DS3231_Add_0E   0x0E
#define DS3231_Add_11   0x11

#define DS1621_ID       0x48
#define ACCESS_CFG      0xAC
#define START_CONV      0xEE
#define READ_TEMP       0xAA
#define CONT_CONV       0x02

#define ACK             1
#define NAK             0
#define _XTAL_FREQ      8000000

#define TFT_RST         PORTDbits.RD2   // Location of TFT Reset
#define TFT_CS          PORTDbits.RD1   // Location of TFT Chip Select
#define TFT_DC          PORTDbits.RD0   // Location of TFT D/C
#define setup_sw        PORTAbits.RA0   // Location of Time Setup Switch
#define RPS_Sample_LED  PORTEbits.RE2	// Location of Sample LED

#define TS_1            1               // Size of Normal Text
#define TS_2            2               // Size of Number Text 

void setup_time(char);
void DS3231_Init(char);
void DS3231_Read_Time(char);
void DS3231_Write_Time(char);

void DS1621_Init(char);
int DS1621_Read_Temp(char);
void delay_ms(int ms);
void delay_500ms(void);
void do_update_pwm(char);
int get_RPS(void);
void INT0_isr(void);
void INT1_isr(void);

#define start_x             2
#define start_y             2
#define temp_x              28
#define temp_y              23
#define circ_x              40
#define circ_y              35
#define data_tempc_x        15
#define data_tempc_y        35
#define tempc_x             45
#define tempc_y             35
#define cirf_x              95
#define cirf_y              35
#define data_tempf_x        70
#define data_tempf_y        35
#define tempf_x             100
#define tempf_y             35
#define time_x              50
#define time_y              58
#define data_time_x         15
#define data_time_y         70
#define date_x              50
#define date_y              93
#define data_date_x         15
#define data_date_y         105
#define dc_x                15
#define dc_y                130
#define RPM_x               90
#define RPM_y               130
#define data_dc_x           4
#define data_dc_y           142
#define data_RPM_x          75
#define data_RPM_y          142
#include "ST7735_TFT.inc"

#define SCL_PIN PORTEbits.RE0				                                        // Place locaton for SCL pin
#define SCL_DIR TRISEbits.RE0				                                        // Place locaton for SCL pin
#define SDA_PIN PORTEbits.RE1				                                         // Place locaton for SDA pin
#define SDA_DIR TRISEbits.RE1				                                         // Place locaton for SDA pin

#include "soft_i2c.inc"

char buffer[31] = " ECE3301L Fall 2019\0";
char *nbr;
char *txt;
char tempC[]        = "25";
char tempF[]        = "77";
char time[]         = "00:00:00";
char date[]         = "00/00/00";
char dc_text[]      = "95%";
char RPM_text[]     = "3600";

unsigned char i, second, minute, hour, dow, day, month, year, old_second;
int DS1621_tempC;
int DS1621_tempF;
char contr; 
bit SWUP_flag, SWDN_flag;
char duty_cycle;
char dc = 0;
int rps, RPM;

void interrupt high_priority chkisr() 
{
    if (INTCONbits.INT0IF == 1) INT0_isr();
    if (INTCON3bits.INT1IF == 1) INT1_isr();
}

void INT0_isr(void)
{
    SWUP_flag=1;                                                                  // the flag is set one 
    
    INTCONbits.INT0IF = 0;                                                         // clear external interrupt INTCON
	              
}

void INT1_isr(void)
{
    
    SWDN_flag=1;                                                                      // the flag is set to one 
                   
	INTCON3bits.INT1IF = 0;                                                             // clear external interrupt INTCON3
}

void putch (char c)
{   
    while (!TRMT);       
    TXREG = c;
}

void init_UART()
{
    	OpenUSART (USART_TX_INT_OFF & USART_RX_INT_OFF & USART_ASYNCH_MODE & USART_EIGHT_BIT & USART_CONT_RX & USART_BRGH_HIGH, 25);
    	OSCCON = 0x70;
}

void Initialize_Screen(void) 
{ 
    LCD_Reset();                                                                     // Screen reset
    TFT_GreenTab_Initialize();         
    fillScreen(ST7735_BLACK);                                                        // Fills background of screen with color passed to it
 
    strcpy(txt, " ECE3301L Fall 2019\0");                                            // Text displayed 
    drawtext(start_x , start_y, txt, ST7735_WHITE  , ST7735_BLACK, TS_1);            // X and Y coordinates of where the text is to be displayed
    strcpy(txt, "Temperature:");
    drawtext(temp_x  , temp_y , txt, ST7735_MAGENTA, ST7735_BLACK, TS_1);          // including text color and the background of it
    drawCircle(circ_x, circ_y , 2  , ST7735_YELLOW);
    strcpy(txt, "C/");
    drawtext(tempc_x , tempc_y, txt, ST7735_YELLOW , ST7735_BLACK, TS_2);          // fillig yellow with black 
    strcpy(txt, "F");         
    drawCircle(cirf_x, cirf_y , 2  , ST7735_YELLOW);
    drawtext(tempf_x , tempf_y, txt, ST7735_YELLOW , ST7735_BLACK, TS_2);            //  filling yellow to black 
    strcpy(txt, "Time");
    drawtext(time_x  , time_y , txt, ST7735_BLUE   , ST7735_BLACK, TS_1);             // filling blue to black 
    strcpy(txt, "Date");
    drawtext(date_x  , date_y , txt, ST7735_RED    , ST7735_BLACK, TS_1);             // filling red to black 
    strcpy(txt, "DC");
    drawtext(dc_x    , dc_y   , txt, ST7735_WHITE  , ST7735_BLACK, TS_1);            // white / black 
    strcpy(txt, "RPM");
    drawtext(RPM_x   , RPM_y  , txt, ST7735_WHITE   , ST7735_BLACK, TS_1);            // white / black 
}

void Update_Screen(void)
{
    tempC[0] = DS1621_tempC/10  + '0';
    tempC[1] = DS1621_tempC%10  + '0';
    tempF[0] = DS1621_tempF/10  + '0';		                                     // update tempF			
    tempF[1] = DS1621_tempF%10  + '0';                                         // undating tempF
    time[0]  = '1';                                                                // temp set to character 1
    time[1]  = '5';                                                                     // temp set to character 5
    time[3]  = '4';                                                              // temp set to character 4
    time[4]  = '6';                                                                // temp set to character 6
    time[6]  = (second>>4)  + '0';                                                  // seocnds adress 
    time[7]  = (second&0x0F)  + '0';                                
    date[0]  = '1';                                                                 // date [0] set to 1 
    date[1]  = '1';                                                             // date[1] set to one 
    date[3]  = '1';                                                                  // date [4] set to one
    date[4]  = '4';
    date[6]  = (year>>4)  + '1';                                                     // year address 
    date[7]  = (year&0x0F)  + '9';
    
    
          
        
        char RPS = get_RPS();                                                     // measure RPS
        int HZ = RPS * 2;                                                             // calculate HZ equivalent
        int RPM = RPS * 60;                                                       // calculate RPM equivalent
        
        char dc1 = ((int)dc)/10;
        char dc2 = ((int)dc)% 10;
        dc_text[0] = dc1 + '0';
        dc_text[1] = dc2 + '0';
        
             
        
        char RPM1 = RPM/1000;                                                     // claulating the RPM1, RPM2,RPM3,RPM4
        char RPM2 = (RPM/100)%10;                                                 // RMP2 
        char RPM3 = (RPM/10)%10;                                                   // RMP3
        char RPM4 = RPM % 10;                                                     // RMP4
        
    RPM_text[0] = RPM1 + '0';                                                    // initializing the text for RMP1
    RPM_text[1] = RPM2 + '0';                                                   // initlaize rm2 to lcd
    RPM_text[2] = RPM3 + '0';                                                    // set the  rm3 text to lcd
    RPM_text[3] = RPM4 + '0';                                                      // the rpm4 
    
    drawtext(data_tempc_x, data_tempc_y, tempC   , ST7735_YELLOW , ST7735_BLACK, TS_2);       
    drawtext(data_tempf_x, data_tempf_y, tempF   , ST7735_YELLOW , ST7735_BLACK, TS_2);
    drawtext(data_time_x , data_time_y , time    , ST7735_CYAN   , ST7735_BLACK, TS_2);
    drawtext(data_date_x , data_date_y , date    , ST7735_GREEN  , ST7735_BLACK, TS_2);
    drawtext(data_dc_x   , data_dc_y   , dc_text , ST7735_MAGENTA, ST7735_BLACK, TS_2);
    drawtext(data_RPM_x  , data_RPM_y  , RPM_text, ST7735_MAGENTA, ST7735_BLACK, TS_2);        
} 

void setup_time(char Device)
{
    minute = 0x38;                                                                 // minute address
    hour = 0x15;                                                                  // hour address
    dow = 0x00;                                                                   // address for dow
    day = 0x07;                                                                   // address for day 
    month = 0x11;                                                                // month address
    year = 0x19;                                                                 // year at address 0x19
    I2C_Start();
    I2C_Write((Device<<1)|0);                                                        // adress of I2C_device
    I2C_Write(DS3231_Add_00);                                                      // writing the rest of the functions for I2C_write 
    I2C_Write(0);
    I2C_Write(minute);
    I2C_Write(hour);
    I2C_Write(dow);
    I2C_Write(day);
    I2C_Write(month);
    I2C_Write(year);
    __delay_ms(20);
}
 
void DS3231_Read_Time(char Device)
{
   
  I2C_Start();                                                                   // Start I2C 
  I2C_Write((Device << 1) | 0);                                                  // Device address
  I2C_Write(DS3231_Add_00 );                                                     // Send register address
  I2C_ReStart();                                                                 // Restarting I2C
  I2C_Write((Device << 1) | 1);                                                   // Initialize data read
  second = I2C_Read(ACK);                                                          //reading seconds
  minute = I2C_Read(ACK);                                                         //reading minutes
  hour = I2C_Read(ACK);                                                           //read an hour
  dow = I2C_Read(ACK);                                                           //reading dow
  day = I2C_Read(ACK);                                                             //reads the day
  month = I2C_Read(ACK);                                                        //reads the month
  year = I2C_Read(NAK);                                                          //the year NAK stops the data
  I2C_Stop();
  __delay_ms(30);                   
  
}



int DS1621_Read_Temp(char Device)
{
 BYTE Data_Ret1, Data_Ret2;                                                       //two data variables for the decimal 0.5  
 float Temp;
  I2C_Start();                                                                   // Start I2C 
  I2C_Write((Device << 1) | 0);                                                    // address Write mode
  I2C_Write(READ_TEMP);                                                               //this is the register address
  I2C_ReStart();                                                                  // Restart I2C
  I2C_Write((Device << 1) | 1);                                                  // Initializing the reading data 
  Data_Ret1 = I2C_Read(ACK);                                                    // reads ACK
  Data_Ret2 = I2C_Read(NAK);                                                     // NAK will stop 
  I2C_Stop(); 
     Temp = Data_Ret1;
  if (Data_Ret2 == 0x80) Temp = Temp + 0.5;
  
  return Temp;
}
void DS3231_Init(char Device)                                                      // seeting contoller DS3231 device 
{
  contr = I2C_Write_Address_Read_One_Byte(Device, DS3231_Add_0E);      
  contr = contr & 0xbb;
  contr = contr | 0x40;
  I2C_Write_Address_Write_One_Byte(Device, DS3231_Add_0E, contr);
}

void DS1621_Init(char Device)
{
  I2C_Write_Address_Write_One_Byte(Device, ACCESS_CFG, CONT_CONV);
  I2C_Write_Cmd_Only(Device, START_CONV);  
}

void main()
{
    init_UART();
    OSCCON = 0x70;
    ADCON1 = 0x0F;                                                                    //SETTING THE PORTS 
    nRBPU = 0;                                                                   // Enable PORTB internal pull up resistor
    TRISA = 0xFF;							                                     // Setup your TRIS for the ports here and below
	TRISB = 0xFF;							                                      // setting the TRISA ON
    TRISC = 0x01;                                                                // set the TRICS ON 
    TRISD = 0x00;                                                                // TRISD OFF
    TRISE = 0x00;                         	                                    // TRISE OFF
    
    I2C_Init(100000);                                                              // Initialize I2C Master with 100KHz clock 
    txt = buffer;    
    
    Initialize_Screen();
	
	DS1621_Init(DS1621_ID);                                                        // Initialize DS1621 chip
	DS3231_Init(DS3231_ID);                                                       // Initialize DS3231 RTC chip
    DS1621_Init(DS1621_ID);                                                        // Initialize DS1621 chip
	DS3231_Init(DS3231_ID);                                                       // Initialize DS3231 RTC chip
    
    INTCONbits.INT0IF = 0;                                                      // INT0 IF is in INTCON
	INTCON3bits.INT1IF = 0;                                                     // INT1 IF is in INTCON3

    INTCONbits.INT0IE = 1;                                                           // INT0 IE is in INTCON
	INTCON3bits.INT1IE = 1;                                                       // INT1 IE is in INTCON3
	   
    INTCON2bits.INTEDG0 = 0;                                                         // Edge programming for INT0, INT1 and 
	INTCON2bits.INTEDG1 = 0;                                                     // INT2 are in INTCON2
            
    INTCONbits.PEIE = 1;                                                         // Enable Peripheral interrupt
	INTCONbits.GIE = 1;                                                           // enable global interrupt
    
    while(TRUE)
    {  
        if(setup_sw == 0)
        {     
            DS3231_Read_Time(DS3231_ID);

            if (second != old_second)
            {
				DS1621_tempC = DS1621_Read_Temp(DS1621_ID);
				DS1621_tempF = DS1621_tempC*9/5 + 32;
	
                printf ("Time: %02x:%02x:%02x  Date: %02x:%02x:%02x  ",hour,minute,second,month,day,year);
                printf ("Temp: %2d C %2d F\r", DS1621_tempC,DS1621_tempF);
                          
				Update_Screen();
  
     
                old_second = second;                                                // update second
            }
        }
        else
        {
          setup_time(DS3231_ID);
        }
        
        if (SWUP_flag == 1)
        {
            dc=dc+5;
            if (dc>95) 
                dc=0;
            SWUP_flag=0;
            do_update_pwm(dc);                                                 // produce  PWM
            
        }
        if (SWDN_flag == 1)
        {
            
            dc = dc -5;
            if (dc>100) dc = 95;
            SWDN_flag=0;
            do_update_pwm(dc);                                              // update PWM
        }
    }
}

int get_RPS(void)
{
    TMR1L = 0;                                                                   // clearing TMR1L
    T1CON = 0x03;                                                                    // address for counter on 
    RPS_Sample_LED = 1;                                                               // pulse turing on 
    delay_500ms ();                                                             //a 500 msec is apply 
    RPS_Sample_LED = 0;                                                            // pulse off
    char RPS = TMR1L;                                                              // taking the value of the pulse 
    T1CON = 0x02;                                                                // turn counter off 
    return (RPS);                                                              // return RPS counter 
}

void do_update_pwm(char duty_cycle)
{
    float dc_f;
    int dc_I;
    PR2 = 0b00000100 ;                                                            // a 25 Khz is generated 
    T2CON = 0b00000111 ; //
    dc_f = ( 4.0 * duty_cycle / 20.0) ;                                         // dusty cycle claculaton for the  25 Khz
    
    dc_I = (int) dc_f;                                                            //setting the int part 
    if (dc_I > duty_cycle) dc_I++;                                                   // fixing the duty cycle fraction 
    CCP1CON = ((dc_I & 0x03) << 4) | 0b00001100;
    CCPR1L = (dc_I) >> 2;
}

void delay_500ms(void)
{
   	T0CON = 0x04;                                                                   // Timer 0 a 16 bit
	TMR0L = 0xEE;                                                               // TMR lower bytes
	TMR0H = 0x85;                                                                       // TMR high bytes 
    INTCONbits.TMR0IF = 0;                                                           // flag for t0 us clear 
    T0CONbits.TMR0ON = 1;                                                               // timer 0 is off 
    while (INTCONbits.TMR0IF == 0);                                              // atfer flag is 1 the while is done 
    T0CONbits.TMR0ON = 0;                                                               // timer 0 is off
}

void delay_ms(int ms)
{
#define CPU_CLOCK       _XTAL_FREQ/4
#define COUNT_PER_MS    CPU_CLOCK/1000
#define COUNT_SCALED    COUNT_PER_MS/16
    
    int count;
    count = (0xffff - COUNT_SCALED) - 1;
    count = count * ms;
    
	T0CON = 0x03;                                                                   // Timer 0 the 16-bit mode and  prescaler 1:16
	TMR0L = count & 0x00ff;                                                                 // TMR0L lower bite 
	TMR0H = count >> 8;                                                          // TMR0H 
	INTCONbits.TMR0IF = 0;                                                          // flag clear 
	T0CONbits.TMR0ON = 1;                                                               // timer off

	while (INTCONbits.TMR0IF == 0);                                                 // atfer flag is 1 the while is done 
	T0CONbits.TMR0ON = 0;                                                           // timer 0 off
}
