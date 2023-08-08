/*
 * File:   pwm_asl.c
 * Author: raed
 * PWM + ADC + SERIAL + LCD
 * Created on March 30, 2019, 1:05 PM
 * LCD is set to work on the simulator, must be fixed to work with real
 */
#include <stdio.h>

#define _XTAL_FREQ   4000000UL     // needed for the delays, set to 4 MH= your crystal frequency
// CONFIG1H
#pragma config OSC = XT         // Oscillator Selection bits (XT oscillator)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRT = OFF       // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = SBORDIS  // Brown-out Reset Enable bits (Brown-out Reset enabled in hardware only (SBOREN is disabled))
#pragma config BORV = 3         // Brown Out Reset Voltage bits (Minimum setting)

// CONFIG2H
#pragma config WDT = ON         // Watchdog Timer Enable bit (WDT enabled)
#pragma config WDTPS = 32768    // Watchdog Timer Postscale Select bits (1:32768)

// CONFIG3H
#pragma config CCP2MX = PORTC   // CCP2 MUX bit (CCP2 input/output is multiplexed with RC1)
#pragma config PBADEN = ON      // PORTB A/D Enable bit (PORTB<4:0> pins are configured as analog input channels on Reset)
#pragma config LPT1OSC = OFF    // Low-Power Timer1 Oscillator Enable bit (Timer1 configured for higher power operation)
#pragma config MCLRE = ON       // MCLR Pin Enable bit (MCLR pin enabled; RE3 input pin disabled)

// CONFIG4L
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = ON         // Single-Supply ICSP Enable bit (Single-Supply ICSP enabled)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection bit (Block 0 (000800-003FFFh) not code-protected)
#pragma config CP1 = OFF        // Code Protection bit (Block 1 (004000-007FFFh) not code-protected)
#pragma config CP2 = OFF        // Code Protection bit (Block 2 (008000-00BFFFh) not code-protected)
#pragma config CP3 = OFF        // Code Protection bit (Block 3 (00C000-00FFFFh) not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot block (000000-0007FFh) not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection bit (Block 0 (000800-003FFFh) not write-protected)
#pragma config WRT1 = OFF       // Write Protection bit (Block 1 (004000-007FFFh) not write-protected)
#pragma config WRT2 = OFF       // Write Protection bit (Block 2 (008000-00BFFFh) not write-protected)
#pragma config WRT3 = OFF       // Write Protection bit (Block 3 (00C000-00FFFFh) not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot Block (000000-0007FFh) not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection bit (Block 0 (000800-003FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection bit (Block 1 (004000-007FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR2 = OFF      // Table Read Protection bit (Block 2 (008000-00BFFFh) not protected from table reads executed in other blocks)
#pragma config EBTR3 = OFF      // Table Read Protection bit (Block 3 (00C000-00FFFFh) not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot Block (000000-0007FFh) not protected from table reads executed in other blocks)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.
#include <xc.h>
#include <stdio.h>
#include "my_ser.h"
#include "my_adc.h"
#include "my_pwm.h"
#include "lcd_x8.h"
//function prototypes
#define STARTVALUE  3036
#define COOLER_PIN   PORTCbits.RC2
#define HEATER_PIN   PORTCbits.RC5 

unsigned int RPS_count = 0;

unsigned int HS = 0;
unsigned int selectedMode = 0;
  float SP=0;
  float RT=0;
  float coolError=0;


 
unsigned char heaterDutyCycle = 0; // Initialize to 0% (heater off)

void setupPorts(void) ;
void initTimers01(void) ;
void setupPorts3(void);
void disableInterrupts(void);
void enableInterrupts(void);

void Timer3_isr(void);
void __interrupt(high_priority) highIsr(void)//new syntax
{ 
   
    RPS_count = ((unsigned int) TMR1H << 8) | (TMR1L); //

    TMR0H = (unsigned char) ((STARTVALUE >> 8) & 0x00FF);
    TMR0L = (unsigned char) (STARTVALUE & 0x00FF);
    TMR1H = 0;
    TMR1L = 0;
    INTCONbits.T0IF = 0;
    
    if (PIR2bits.TMR3IF) Timer3_isr();
    
    if (INTCON3bits.INT1IF) { //Push button
        INTCON3bits.INT1IF = 0; // Clear INT1 interrupt flag
             HS++;
            if(HS==4){
                HS=0;
            }
    }
    if (INTCONbits.INT0IF) { //Push button
        INTCONbits.INT0IF = 0; // Clear INT1 interrupt flag
             selectedMode++;
            if(selectedMode==4){
                selectedMode=0; 
            } 
            

    }
}



void updateLCD(void);
void turnONHeater(void);
void turnONCooler(void);
void turnOFFCooler(void);
void turnOFFHeater(void);

enum MODE{
   OFF ,COOL , HEAT, AUTOCOOL 
};

// Function to set the heater duty cycle

void main(void) {
    
    float coolAmount;
    int raw_val;
  //  unsigned char channel;
   // float voltage;
    setupPorts();
    setupPorts3(); //for timer 3
    lcd_init();
    init_adc_no_lib();
    init_pwm1();
    //PORTCbits.RC4 =1;
    PORTCbits.RC5 = 1;
  
    lcd_putc('\f'); //clears the display
   
    int RPS;
     initTimers01();   // These will be used to measure the speed
    
     TRISCbits.RC0 = 1; //Timer1 clock
   
    
     enableInterrupts();  //so as timer interrupt work correctly

    while (1) {
        CLRWDT(); // no need for this inside the delay below
        updateLCD();
        delay_ms(200); //read ADC AN0,AN1, AN2 every 2 seconds
             
             SP = read_adc_voltage((unsigned char) 0); //AI0 ->SP
             RT = read_adc_voltage((unsigned char) 2); //AI2 ->RT
                
          
               if (selectedMode==0){//OFF
                   turnOFFHeater();
                   turnOFFCooler();
                    coolAmount = 0;
                    set_pwm1_raw(coolAmount);
                   
               }
              
               else if (selectedMode==1){
                   turnONCooler();
                   turnOFFHeater();
                  
                   //POTENTIOMETER controls COOLER
                   raw_val = read_adc_voltage(1); // read raw value for POT1 
                   set_pwm1_percent(raw_val);  // set the Pwm to that value 0-100
                   
               } 
                 //In this mode, simply turn ON the heater
                    
               else  if (selectedMode==2){ //HEAT 
                   turnOFFCooler();
                   turnONHeater();
                   coolAmount = 0;
                    set_pwm1_raw(coolAmount);
               }
              
             
                 else if(selectedMode==3){//auto cool
                     turnOFFHeater();//initially
                     turnONCooler();
                     
                      coolError= RT - SP;
                     if(coolError > 0){ //if RT>SP
                        float tst= coolError*100.0/10.0;
                        if(tst < 25){
                            tst=25;//If this value is less than 25% set it to 25%
                        }
                        turnONCooler();
                        set_pwm1_percent(tst);
                      }  
                     
                     if( RT < (SP-HS)){ //if it cooler than needed-> need to 1-heat and 2-stop cooling
                         set_pwm1_percent(0);
                         turnOFFCooler();
                       // Turn the heater here ON with 50% -- MADE IN TIMER 
                       }
                         
                  }
         
        RPS = RPS_count;


    }
}
 char*  getMode(){
    switch (selectedMode){
        case 0: return "OFF         ";
        case 1: return "COOL         ";
        case  2 : return "HEAT      ";
        case  3 : return "AUTOCOOL       ";
    }
    return "OFF";
}
void updateLCD(void){
    char Buffer[32];
    lcd_gotoxy(1, 1);
    sprintf(Buffer, "RT: %3.1fC  H  C ",RT);
    lcd_puts(Buffer);
    
     lcd_gotoxy(1, 2);
    sprintf(Buffer, "SP: %3.1fC  %c  %c ",SP,HEATER_PIN==0?'N':'Y',COOLER_PIN==0?'N':'Y');
    lcd_puts(Buffer);
    
     lcd_gotoxy(1, 3);
    sprintf(Buffer, "HS : %d ",HS);
    lcd_puts(Buffer);
    
     lcd_gotoxy(1, 4);
      char* s= getMode();
    sprintf(Buffer, "MD: %s ",s);
    lcd_puts(Buffer);
}
void disableInterrupts(void) {
    INTCONbits.GIEH = 0;
    INTCONbits.GIEL = 0;
}

void enableInterrupts(void) {
    INTCONbits.GIEH = 1;
    INTCONbits.GIEL = 1;
}
void turnOFFCooler(void){
    COOLER_PIN =0 ;
    set_pwm1_raw(0);  // set the Pwm to that value 0--1023

}
void turnOFFHeater(void){
    HEATER_PIN =0;  
                   
}
void turnONCooler(void){
    COOLER_PIN =1 ;
}
void turnONHeater(void){
    HEATER_PIN =1;  
}

void Timer3_isr(void) {

    switch (selectedMode) {
        case 0://OFF
            HEATER_PIN = 0; // Turn OFF the Heater
            break;
        case 2://heat
            HEATER_PIN = 1; // Turn ON the Heater
            break;
        case 1://cool
            HEATER_PIN=0;
            COOLER_PIN=1;
            break;
        case 3: //auto cool 
          if( RT < (SP -HS)) {
                HEATER_PIN=!HEATER_PIN; //toggle
            }
            
            break;
        default:
            // Handle other cases if needed
            break;
    }

}
void setupPorts(void) {
    ADCON0 = 0;
    ADCON1 = 0b00001100; //3 analog channels, change this according to your application

    TRISB = 0xFF; // all pushbuttons are inputs
    TRISC = 0x80; // RX input , others output
    TRISA = 0xFF; // All inputs
    TRISD = 0x00; // All outputs
    TRISE = 0x00; // All outputs
    
    HS = 0;
    selectedMode = 0;
    
  
}
void initTimers01(void) {
    T0CON = 0;
    //T0CONbits.T0CS = 0;
    //T0CONbits.PSA = 0;
    //T0CONbits.T08BIT = 1;
    INTCONbits.T0IF = 0;
    T0CONbits.T0PS0 = 1; // 16 prescalar
    T0CONbits.T0PS1 = 1;
    T0CONbits.T0PS2 = 0;
    TMR0H = (unsigned char) ((STARTVALUE >> 8) & 0x00FF);
    TMR0L = (unsigned char) (STARTVALUE & 0x00FF);

    T1CONbits.TMR1CS = 1; //external clock ,emasuring the speed of the fan in RPS
    T1CONbits.T1CKPS1 = 0;
    T1CONbits.T1CKPS0 = 0;


    TMR1H = 0;
    TMR1L = 0;
    INTCONbits.GIE = 1; //enable only timer 0 interrupt
    INTCONbits.T0IE = 1;
    T1CONbits.TMR1ON = 1;
    T0CONbits.TMR0ON = 1;

}
#define TMR3_PRESCALER 8     // Set Timer 3 prescaler to 1:8
#define TMR3_PERIOD 6249     // Set Timer 3 period for 200 ms interrupt

void setupPorts3(void) {
    // Set internal oscillator frequency to 16MHz
    OSCCONbits.IRCF = 0b1110;

    // Configure Timer 3
    T3CONbits.TMR3CS = 0;   // Select internal clock source for Timer 3
  //  T3CONbits.T3CKPS = 0b11; // Set Timer 3 prescaler to 128
    T3CONbits.nT3SYNC = 1;  // Timer 3 is not synchronized with the external clock source
    T3CONbits.TMR3ON = 1;   // Enable Timer 3

    // Set Timer 3 period
    TMR3 = 0;               // Reset Timer 3 value

    // Configure RC5 as output
    TRISCbits.TRISC5 = 0;   // RC5 as output

    // Enable global interrupts
    INTCONbits.GIE = 1;
    
    //for interrupt every 200ms
//    Timer3 Period = (Desired Interrupt Time / Timer 3 Clock Period) - 1
//
//Given the desired interrupt time of 200 ms and a Timer 3 clock frequency of 31250 Hz (as calculated earlier), calculate the new Timer 3 period:
//
//Timer3 Period = (0.200 s / (1 / 31250 Hz)) - 1 = 6250 - 1 = 6249
    TMR3H = 6249 >> 8; // High byte of period
    TMR3L = 6249 & 0xFF; // Low byte of period
    T3CONbits.T3CKPS = 0b01; // Set Timer 3 prescaler to 1:8


    
    T3CONbits.T3CKPS = 0b01; // Set Timer 3 prescaler to 1:8


    // Set Timer 3 period
    TMR3H = TMR3_PERIOD >> 8; // High byte of period
    TMR3L = TMR3_PERIOD & 0xFF; // Low byte of period

}