/*
 * File:   main.c
 * Author: Byron Barrientos
 *
 * Created on October 17, 2022, 9:14 PM
 */

// CONFIG1
#pragma config FOSC = INTRC_NOCLKOUT        // Oscillator Selection bits 
                                            // (INTOSC oscillator: I/O function 
                                            // on RA6/OSC2/CLKOUT pin, I/O 
                                            // function on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled and 
                                // can be enabled by SWDTEN bit of the WDTCON 
                                // register)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF      // RE3/MCLR pin function select bit (RE3/MCLR 
                                // pin function is digital input, MCLR 
                                // internally tied to VDD)
#pragma config CP = OFF         // Code Protection bit (Program memory code 
                                // protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code 
                                // protection is disabled)
#pragma config BOREN = OFF      // Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO = OFF       // Internal External Switchover bit 
                                // (Internal/External Switchover mode disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enabled bit 
                                // (Fail-Safe Clock Monitor is disabled)
#pragma config LVP = OFF        // Low Voltage Programming Enable bit (RB3 pin 
                                // has digital I/O, HV on MCLR must be used for 
                                // programming)

// CONFIG2
#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit (Brown-out 
                                // Reset set to 4.0V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits 
                                // (Write protection off)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <stdint.h>

#define _XTAL_FREQ 500000
#define V_TMR0 240

//******************************************************************************
// Variables
//******************************************************************************

int V1_ADRESH;
int V2_ADRESH;
int V3_ADRESH;
int SERVO;
uint8_t CONT_TMR0;

//******************************************************************************
// Prototipos de Funciones
//******************************************************************************

void setup(void);
void setupADC(void);
void setupPWM(void);
void convertir(int V_ADRESH);

//******************************************************************************
// Interrupciones
//******************************************************************************
void __interrupt() isr (void){
    
    // Interrupción del ADC
    
    if (PIR1bits.ADIF)          // Una vez terminada la conversión
    {
        PIR1bits.ADIF = 0;      // Bajamos la bandera de interrupción del ADC
    }
    
    //Interrupción del TMR0
    
    if (INTCONbits.T0IF)        // Cuando el TMR0 haga overflow
    {
        CONT_TMR0++;            // Incrementar en 1 la variable CONT_TMR0
        
        if (CONT_TMR0 <= V3_ADRESH)     // Si CONT_TMR0 es menor o igual a V3_ADRESH
        {
            PORTDbits.RD0 = 1;  // Encender el RD0
        }
        else
        {
            PORTDbits.RD0 = 0;  // Apagar el RD0
        }
        
        TMR0 = V_TMR0;          // Se carga V_TMR0 al TMR0
        INTCONbits.T0IF = 0;    // Bajamos la bandera de interrupción del TMR0
    }
}

//******************************************************************************
// Código Principal
//******************************************************************************
void main(void) {
    
    setup();
    setupADC();
    setupPWM();
    
    CONT_TMR0 = 0;
    
    while(1){
        
        //**********************************************************************
        //Primer Servomotor
        //**********************************************************************
                                
        ADCON0bits.CHS = 0b0000;        //Selección del canal AN0
        __delay_us(100);
        ADCON0bits.GO = 1;              //Iniciamos la conversión en el ADC
        while (ADCON0bits.GO == 1){};
        
        V1_ADRESH = ADRESH;             //Pasamos el valor de ADRESH a V1_ADRESH
        convertir(V1_ADRESH);           
        CCPR1L = SERVO;                 //Se envía el PWM al puerto
        __delay_us(100);
        
        //**********************************************************************
        //Segundo Servomotor
        //**********************************************************************
                                
        ADCON0bits.CHS = 0b0001;        //Selección del canal AN1
        __delay_us(100);
        ADCON0bits.GO = 1;              //Iniciamos la conversión en el ADC
        while (ADCON0bits.GO == 1){};
        
        V2_ADRESH = ADRESH;             //Pasamos el valor de ADRESH a V2_ADRESH
        convertir(V2_ADRESH);           
        CCPR2L = SERVO;                 //Se envía el PWM al puerto
        __delay_us(100);
        
        //**********************************************************************
        //LED
        //**********************************************************************
                                
        ADCON0bits.CHS = 0b0010;        //Selección del canal AN2
        __delay_us(100);
        ADCON0bits.GO = 1;              //Iniciamos la conversión en el ADC
        while (ADCON0bits.GO == 1){};
        
        V3_ADRESH = ADRESH;             //Pasamos el valor de ADRESH a V3_ADRESH
        __delay_us(100);
        
    }
    return;
}

//******************************************************************************
// Funciones
//******************************************************************************
void setup (void){
    
    ANSELH = 0;
    
    TRISA = 0;              //Configuración del PORTA como output
    TRISB = 0;              //Configuración del PORTB como output
    TRISC = 0;              //Configuración del PORTC como output
    TRISD = 0;              //Configuración del PORTD como output
    TRISE = 0;              //Configuración del PORTE como output
    
    PORTA = 0;              //Limpiamos el PORTA
    PORTB = 0;              //Limpiamos el PORTB
    PORTC = 0;              //Limpiamos el PORTC
    PORTD = 0;              //Limpiamos el PORTD
    PORTE = 0;              //Limpiamos el PORTD
    
    // Configuración del Oscilador Interno a 500KHz
    
    OSCCONbits.IRCF = 0b011 ;   // Selección de los 500KHz
    OSCCONbits.SCS = 1;         // Selección del Oscilador Interno
    
    // Interrupción del ADC
    
    PIE1bits.ADIE = 1;      //Habilitamos la interrupción del ADC
    PIR1bits.ADIF = 0;      //Bajamos la bandera de interrupción del ADC
    
    // Configuración del TMR0 y su Interrupción
    
    OPTION_REGbits.T0CS = 0;        // Fosc/4
    OPTION_REGbits.PSA = 0;         // Prescaler para el TMR0
    OPTION_REGbits.PS = 0b011;      // Prescaler 1:16
    TMR0 = V_TMR0;                  // Asignamos valor al TMR0 para 2ms
    
    INTCONbits.TMR0IE = 1;  //Habilitamos la interrupción del TMR0
    INTCONbits.T0IF = 0;    //Bajamos la bandera de interrupción del TMR0
    
    // Interrupciones
    
    INTCONbits.GIE = 1;     //Habilitamos las interrupciones globales (GIE)
    //INTCONbits.PEIE = 1;    //Habilitamos las interrupción del PEIE
    //INTCONbits.RBIF = 1;    //Habilitamos las interrupciones del PORTB (RBIF)
    //INTCONbits.RBIE = 0;    //Bajamos la bandera de interrupción del PORTB (RBIE)
    
}

void setupADC (void){
    
    //Paso 1: Selección del puerto de entrada
    
    TRISAbits.TRISA0 = 1;       //Configuración del RA0 como input
    ANSELbits.ANS0 = 1;         //Configuración del pin RA0 como análogo (AN0)
    
    TRISAbits.TRISA1 = 1;       //Configuración del RA1 como input
    ANSELbits.ANS1 = 1;         //Configuración del pin RA1 como análogo (AN1)
    
    TRISAbits.TRISA2 = 1;       //Configuración del RA2 como input
    ANSELbits.ANS2 = 1;         //Configuración del pin RA2 como análogo (AN2)
    
    //Paso 2: Configuración del módulo ADC
    
    ADCON0bits.ADCS0 = 1;
    ADCON0bits.ADCS1 = 0;       //Fosc/8
    
    ADCON1bits.VCFG0 = 0;       //VDD como voltaje de referencia -
    ADCON1bits.VCFG1 = 0;       //VSS como voltaje de referencia +
    
    ADCON0bits.CHS0 = 0;
    ADCON0bits.CHS1 = 0;
    ADCON0bits.CHS2 = 0;
    ADCON0bits.CHS3 = 0;        //Selección del canal análogo AN0 (Default)
    
    ADCON1bits.ADFM = 0;        //Justificado hacia la izquierda
    
    ADCON0bits.ADON = 1;        //Habilitamos el ADC
    
    __delay_us(100);            //Delay para adquirir la lectura
}

void setupPWM(void){
   
    // Paso 1
    
    TRISCbits.TRISC2 = 1;           //Configuración del RC1 como input (CCP1)
    TRISCbits.TRISC1 = 1;           //Configuración del RC2 como input (CCP2)
    
    // Paso 2
    
    PR2 = 155;                  // Establecemos un período de 20mS
    
    // Paso 3
    
    CCP1CONbits.P1M = 0b00;     //Selección del modo Single Output
    
    CCP1CONbits.CCP1M = 0b1100;     // P1A como PWM 
    CCP2CONbits.CCP2M = 0b1111;     // P2A como PWM
            
   // Paso 4
    
    CCP1CONbits.DC1B = 0b11;    
    CCP2CONbits.DC2B1 = 0b1;    
    CCP2CONbits.DC2B0 = 0b1;    // CCPxCON<5:4>
    
    CCPR1L = 11;                // CCPR1L
    CCPR2L = 11;                // CCPR2L
    
                                // Cálculo para 1.5mS de ancho de pulso
    
    // Paso 5
    
    PIR1bits.TMR2IF = 0;        // Bajamos la bandera de interrupción TMR2
    T2CONbits.T2CKPS = 0b11;    // Prescaler de 1:16
    T2CONbits.TMR2ON = 1;       // Se enciende el TMR2
    
    // Paso 6
    
    while(!PIR1bits.TMR2IF){};
    
    TRISCbits.TRISC2 = 0;       // Habilitamos la salida del PWM (RC2)
    TRISCbits.TRISC1 = 0;       // Habilitamos la salida del PWM (RC1)
    
}

void convertir(int V_ADRESH){

    SERVO = (unsigned short)(7+((float)(9)/(255))*(V_ADRESH-0));

}