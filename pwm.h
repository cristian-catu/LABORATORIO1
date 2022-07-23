/* 
 * File:   oscilador.h
 * Author: CATU
 *
 * Created on 22 de julio de 2022, 08:02 AM
 */

#ifndef PWM_H
#define	PWM_H


#include <xc.h>
#include <stdint.h>
#define VALOR_PR2 15

//Definimos funciones
void pwm_init(uint8_t channel);
void pwm_duty_cycle(uint16_t duty_cycle, uint8_t channel);
#endif	/* OSCILADOR_H */