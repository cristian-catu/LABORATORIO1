    /*
 * File:   PWM.c
 * Author: Cristian Catú
 * LAB 9
 *
 * Created on 27 april 2022, 19:04
 */

// CONFIG1
#pragma config FOSC = INTRC_NOCLKOUT// Oscillator Selection bits (INTOSCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF      // RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
#pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = OFF      // Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO = OFF       // Internal External Switchover bit (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
#pragma config LVP = OFF        // Low Voltage Programming Enable bit (RB3 pin has digital I/O, HV on MCLR must be used for programming)

// CONFIG2
#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits (Write protection off)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <stdint.h>
/*------------------------------------------------------------------------------
 * LIBRERIAS
 ------------------------------------------------------------------------------*/
#include "oscilador.h"
#include "adc.h"
#include "tmr0.h"
#include "pwm.h"

#define _XTAL_FREQ 4000000

/*------------------------------------------------------------------------------
 * VARIABLES
 ------------------------------------------------------------------------------*/
uint8_t contador = 0;
uint8_t bandera = 1;
uint8_t ciclo_trabajo = 0;
void setup(void);

/*------------------------------------------------------------------------------
 * INTERRUPCIONES
 ------------------------------------------------------------------------------*/
void __interrupt() isr (void){
    if(INTCONbits.T0IF){ //Interrupción TMR0?
        PORTAbits.RA0 = !PORTAbits.RA0;
        tmr0_reload();
    }
    if(PIR1bits.ADIF){ //Interrupción ADC?
        PORTD = adc_read()>>4; //Poner lectura en PUERTO D
    }
    if(PIR1bits.TMR2IF){ //Interrupción TMR2?
        contador++; //contador que cuenta hasta 57 para 1 segundo
        if (bandera){//primer estado incrementar ciclo trabajo, luz de LED aumenta
            if (contador == 57){//Llegó a 57? cambiamos de estado
                bandera = 0;
                contador = 0;
            }
            ciclo_trabajo++;
            pwm_duty_cycle(ciclo_trabajo,1);
        }
        else{//segundo estado decrementar ciclo trabajo, luz de LED disminuye
            if (contador == 57){ //Llegó a 57? cambiamos de estado
                bandera = 1;
                contador = 0;
            }
            ciclo_trabajo--;
            pwm_duty_cycle(ciclo_trabajo,1);
        }
        PIR1bits.TMR2IF = 0;//Limpiamos bandera TMR2
    }
}

/*------------------------------------------------------------------------------
 * CICLO PRINCIPAL
 ------------------------------------------------------------------------------*/
void main(void) {
    setup();
    
    while(1){
        adc_start(12); //Iniciar conversión ADC canal 12 en RB0
    }
    return;
}

/*------------------------------------------------------------------------------
 * CONFIGURACION
 ------------------------------------------------------------------------------*/
void setup(void){
    ANSEL = 0;
    TRISA = 0;
    PORTA = 0;
    TRISC = 0;
    PORTC = 0;
    TRISD = 0;
    PORTD = 0;
    init_osc_MHz(4); //freq            0 ---> 1MHz, 1 ---> 2MHz, 2 ---> 4MHz, 3 ---> 8MHz, 4 ---> 500kHz, default ---> 4MHz
    tmr0_reload();   //función de TMR0
    tmr0_init(256);  //configuración prescaler 256 TMR0 
    pwm_init(1);     //configuración PWM
    pwm_duty_cycle(4,1);
    INTCONbits.PEIE = 1;        // Habilitamos int. de perifericos
    INTCONbits.GIE = 1;         // Habilitamos int. globales
    INTCONbits.T0IE = 1;        // Habilitamos interrupcion TMR0
    
    adc_init(1,0,0);            //función ADC_INIT
    pwm_duty_cycle(1,1);        //función ciclo de trabajo
    
    PIR1bits.TMR2IF = 0;        // Limpiamos bandera de interrupcion del TMR2
    T2CONbits.T2CKPS = 0b11;    // prescaler 1:16
    T2CONbits.TMR2ON = 1;       // Encendemos TMR2
    while(!PIR1bits.TMR2IF);    // Esperar un cliclo del TMR2
    PIR1bits.TMR2IF = 0;        // Limpiamos bandera de interrupcion del TMR2 nuevamente
    T2CONbits.TOUTPS = 0b0111;  //postscaler 8
    PIE1bits.TMR2IE = 1;        //Habilitamos interrupción TMR2
    
    PIR1bits.ADIF = 0;          // Limpiamos bandera de ADC
    PIE1bits.ADIE = 1;          // Habilitamos interrupcion de ADC
    
}