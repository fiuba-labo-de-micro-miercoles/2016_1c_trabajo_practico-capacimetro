/*
 * Capacimetro.asm
 *
 *  Created: 29/06/2016 06:43:24 p.m.
 *   Author: MarceloFernando
 */ 


;.include "m88PAdef.inc     ;comento ésta linea porque Atmel Studio 6 incluye la librería
;----------------------------------------------------------------------
;Declaro constantes para identificar los puertos utilizados por el LCD
;----------------------------------------------------------------------
.equ		lcd_dprt = PORTB		;puerto de datos del lcd
.equ		lcd_dddr = DDRB			;puerto ddr del lcd
.equ		lcd_dpin = PINB			;puerto pin del lcd
.equ		lcd_cprt = PORTC		;puerto de comandos del lcd
.equ		lcd_cddr = DDRC			;puero de comandos ddr del lcd
.equ		lcd_cpin = PINC			;puerto de comandos pin del lcd
.equ		rs = 0					;registro rs del lcd
.equ	    rw = 1					;registro r/w del lcd
.equ	    en = 2					;registro en del lcd

;Otras constantes

.equ		offset = 184
.equ		maximo = (60 * 10 + offset)*64/85	;defino el maximo y minimo valor de capacidad a medir
.equ		minimo = (1 * 10 + offset)*64/85	;maximo = 60 nF minimo = 1 nF y los escalo.
;-----------------------------------------------------------------------------------------------
;Defino unas variables que voy a utilizar para promediar y convertir los valores leídos del adc.
;-----------------------------------------------------------------------------------------------
.def        u_mil = r18
.def		centena = r19
.def		decena = r20
.def        unidad = r21

.def		prom_high = r22
.def		prom_low = r23
 
;Se abre un segmento de datos para definir variables
.dseg 
;.org	0x100
adc_1 :		.byte 128		;vector para guardar 64 valores de 2 bytes 
					


;Segmento de código
.cseg		
rjmp main



.org int_vectors_size			;comienzo después de las interrupciones.

main:
				ldi r16,low(ramend)		; Inicializo el Stack Pointer
				out spl,r16
				ldi r16,high(ramend)
				out sph,r16  

				rcall ini_lcd			;inicio el LCD
				
				ldi zl, low(MJE0<<1)	;"Iniciando----" en pantalla
				ldi zh, high(MJE0<<1)
				rcall enviar_frase
				rcall delay_5seg

			
        		ldi r18, 80				;Comienzo la onda cuadrada con un período T=1600ms
medicion :		rcall ini_onda_cuad
				
				rcall ini_ADC

				ldi r16,0X01			;Borro la pantalla
				rcall enviar_com

				ldi zl, low(MJE1<<1)	;envío "Probando...." a la pantalla
				ldi zh, high(MJE1<<1)
				rcall enviar_frase
				rcall delay_5seg


medir :			rcall grabar_y_promediar	;el promedio me devuelve los valores en prom_high prom_low
				
				
				cpi prom_high, 2			;si el valor es mayor a 50nF = 500 en el adc envio un mje en pantalla
				brcc mayor_a_50n
				cpi prom_high, 1
				brcs comparacion_2
				cpi prom_low, 244
				brcc mayor_a_50n
				rjmp rango_ok

comparacion_2 :	cpi prom_low, minimo				;en esta parte comparo si es menor a 1nF
				brcs longitud_menor_1n				

rango_ok :		rcall escalar

				rcall convertir_a_longitud	;la subrutina toma los valores guardados en r22 r23 del promediador
											;y los convierte en decimal codigo ascii para luego enviar al lcd. 
											;el resultado queda guadado en las variables u_mil centena decena unidad
				
				ldi r16,0X01				;Borro la pantalla
				rcall enviar_com
				
				
				ldi zl, low(MJE5<<1)		;envio mje "Longitud = " a lcd
				ldi zh, high(MJE5<<1)
				rcall enviar_frase


				;si r20 > 1022 -------------> mostrar MJE8.
				;o si r20 < 20 --------> cambiar ancho de pulso a T=160ms (r18) y comenzar denuevo la medición.
				;sino convertir volts medidos en distancia del cable y mostrar en pantalla MJE5 + valor obtenido.

				ldi r16,0XC0			;Comienzo en la segunda linea
				rcall enviar_com
								
				mov r16, u_mil				;muestro en pantalla el resultado de la medición
				rcall enviar_dato
				mov r16, centena
				rcall enviar_dato
				mov r16, decena
				rcall enviar_dato
				ldi r16,'.'
				rcall enviar_dato 
				mov r16, unidad
				rcall enviar_dato

			
				ldi zl, low(MJE6<<1)		;envío " nF" a la pantalla
				ldi zh, high(MJE6<<1)
				rcall enviar_frase
			
				rjmp medir


mayor_a_50n :	ldi r16,0X01				;Borro la pantalla
				rcall enviar_com
				ldi zl, low(MJE8<<1)		;envío "mayor a 50 nF" a la pantalla
				ldi zh, high(MJE8<<1)
				rcall enviar_frase
				rjmp medir


longitud_menor_1n :
				ldi r16,0X01				;Borro la pantalla
				rcall enviar_com
				ldi zl, low(MJE9<<1)		;envío "menor a 1 nF" a la pantalla
				ldi zh, high(MJE9<<1)
				rcall enviar_frase
				rjmp medir


;#####################################################################################
;Rutinas para iniciar el LCD
;#####################################################################################

ini_lcd	:		ldi r21, 0xFF
				out lcd_dddr, r21		;configuro los puertos de datos como salida
				out lcd_cddr, r21		;configuro los puertos de comando como salida
				cbi lcd_cprt, en		;en = 0 para luego grabar
				rcall delay_10ms		;Espero a que se prenda el LCD
				;Inicio al LCD en 2 lineas, matriz de 5x7
				ldi r16,0X38 
				rcall enviar_com 
				;Prendo el Display, Prendo el cursor
				ldi r16,0X0E 
				rcall enviar_com
				;Borro la pantalla
				ldi r16,0X01
				rcall enviar_com
				;Corro el cursor hacia la derecha
 				ldi r16,0X06
				rcall enviar_com
 				ret
				
;-----------------------------------------------------------------------------------------;
;rutinas para enviar datos o comandos al LCD
;-----------------------------------------------------------------------------------------;
enviar_frase :	;ldi r16,0X01			;Borro la pantalla
				;rcall enviar_com
				/*ldi r16,0X06			;Corro el cursor hacia la derecha
				rcall enviar_com*/
				ldi r17, 0
ciclo_subr1 :	inc r17
				cpi r17, 17
				breq linea_2
				lpm r16, z+
				cpi r16,0
				breq salir_subr1
				rcall enviar_dato
				rjmp ciclo_subr1
linea_2 :		ldi r16,0XC0			;Comienzo en la segunda linea
				rcall enviar_com
				rcall ciclo_subr1
salir_subr1 :	ret


enviar_com:		cbi lcd_cprt, rs		;rs=0 para mandar comandos
				cbi lcd_cprt, rw		;rw=0 para escribir en el LCD
				sbi lcd_cprt, en		;E=1
				out lcd_dprt, r16
				rcall sdelay 
				cbi lcd_cprt, en		;E=0 es el flanco descendente (High-to-Low)
				rcall delay_2ms
				ret


enviar_dato: 	sbi lcd_cprt, rs ;rs=1 para mandar datos
				cbi lcd_cprt, rw ;rw=0 para escribir en el LCD
				sbi lcd_cprt, en ;E=1
				out lcd_dprt, r16	
				rcall sdelay	
				cbi lcd_cprt, en ;E=0  ;Es el flanco descendente (High-to-Low)
				rcall delay_2ms
				ret

				  
;#################################################################
;Delays
;################################################################

delay_40us :	push r16		;[2] + rcall [3] = 5
				ldi r16, 18		;[1] --> ciclos = 6
ciclo_40us :	dec r16			;[1]*18 --> ciclos = 24
				brne ciclo_40us	;[1/2]*18 --> ciclos = 33
				pop r16			;[2] --> ciclos = 35
				nop				;[1] --> ciclos = 36
				ret				;[4] --> ciclos = 40, para un clck de 1 Mhz delay = 40us


sdelay:			nop
				nop
				ret


delay_100us:	push r16		;[2] + rcall [3] = 5
				ldi r16,58		;[1] --> ciclos = 6
ciclo_100us :	dec r16			;[1]*58 --> ciclos = 64
				brne ciclo_100us;[1/2]*58 --> ciclos = 93
				pop r16			;[2] --> ciclos = 95
				nop             ;[1] --> ciclos = 96
				ret				;[4] --> ciclos = 100, con un clck = 1Mhz delay = 100us


delay_2ms	:  	push r16		;[2] + [3] del rcall = [5]
				push r17		;[2] --> ciclos = 7
				ldi r17,24		;[1] --> ciclos = 8
ciclo1_2ms	:	ldi r16,22		;[1]*24*33 -->ciclos = 800
ciclo2_2ms	:	dec r16			;[1]*22
				brne ciclo2_2ms	;[1/2]*22 = subtotal = 33
				dec r17			;[1]*24*33 -->ciclos = 1592
				brne ciclo1_2ms	;[1/2]*24*33 -->ciclos = 1988
				nop
				nop
				nop
				nop				;[4] -->ciclos = 1992
				pop r17			;[2] -->ciclos = 1994
				pop r16			;[2] -->ciclos = 1996
				ret				;[4] -->cilclos = 2000, para un clk = 1Mhz delay = 2,00 ms

				
delay_10ms	:  	push r16		
				push r17		
				ldi r17,22		
ciclo1_10ms	:	ldi r16,121		
ciclo2_10ms	:	dec r16			
				brne ciclo2_10ms	
				dec r17			
				brne ciclo1_10ms	
				nop
				nop				
				pop r17			
				pop r16			
				ret	
								
				
delay_100ms :   ldi r16, 20			;[1]
ciclo_100ms	:	rcall delay_2ms		;2ms*20
				dec r16				;2ms*20
				brne ciclo_100ms	;2ms*10 ---> aprox delay (20+20+10)*2ms = 100 ms 
				ret

delay_5seg :	ldi r17, 25
ciclo_dy5s:		rcall delay_100ms
				dec r17
				brne ciclo_dy5s
				ret
;#####################################################################
;Rutina para iniciar onda cuadrada
;#####################################################################
;Genera una onda cuadrada con periodo T = 2*r18.

ini_onda_cuad :	ldi r16, 0x00		;configuro el PORTD.6 como salida
				ori r16, (1<<PD6)		
				out DDRD, r16
				ldi r17, 0x00
				out PORTD, r17		;comienzo la señal con 0
				
				;para generar ciclos de 40 us
				out ocr0a, r18		;configuro el período con el valor guardado en r18 

				;modo CTC(toogle) + fast PWM
				in r17, tccr0a
				ori r17, ((1<<COM0A0)|(1<<WGM01)|(1<<WGM00))
				andi r17, ~(1<<COM0A1)
				out tccr0a, r17				; tccr0a = 0b01xxxx11

				;modo fast PWM, sin preescaler
				in r17, tccr0b
				ori r17, ((1<<WGM02)|(1<<CS00))
				andi r17, ~((1<<CS02)|(1<<CS01))
				out tccr0b, r17				; tccr0b = xxxx1010

				ret


;########################################################################################
;Rutina para iniciar el conversor ADC en modo Free Runing, sin habilitar interrupciones.
;########################################################################################

ini_ADC :		;ADMUX - ADC Multiplexer Selection Register
				;Ref externa, registros guardados a derecha, selecciono ADC3 = pin26 del micro
				lds r16, admux
				ori r16, (1<<MUX1|1<<MUX0)
				andi r16, ~((1<<REFS1)|(1<<REFS0)|(1<<ADLAR)|(1<<MUX3)|(1<<MUX2))
				sts admux, r16	; admux = 00000011

				;ADCSRA - ADC Control and Status Register A
				;ADC on, conversión on, Trigger on, Interrupción ADC off,  
				lds r17, adcsra
				ori r17, ((1<<ADEN)|(1<<ADSC)|(1<<ADATE)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0))
				andi r17, ~((1<<ADIF)|(1<<ADIE))
				sts adcsra, r17  ;adcsra = 11100000

				;ADCSRB - ADC Control and Status Register B
				;Configuro el ADC en modo "Free Runing", aunque ya este por default
				ldi r16, 0x00
				sts adcsrb, r16	 ;adcsrb = 00000000	
				
				;DIDR0 - Digital Input Disable Register 0
				;desactivo los Buffers de los pines ADC que no utilizo para ahorrar energía, dejo solo el ADC4
				;lds r17, didr0
				;ori r17, ((1<<ADC5D)|(1<<ADC3D)|(1<<ADC2D)|(1<<ADC1D)|(1<<ADC0D))
				;andi r17, ~(1<<ADC4D)
				;sts didr0, r17

				ret

/*################################################################################################
Rutina para guardar las tensiones obtenidas por el ADC y promediarlas
#################################################################################################*/

;La rutina graba 64 tensiones en la variable adc_1 obtenidas del adc, el mismo tiene que estar configurado como "Free Runing".
;Luego las promedia y vuelve a repetir el proceso. Se comparan entonces los 2 promedios obtenidos. Si son iguales sale de la 
;subrutina y devuelve el resultado en los registros definidos como prom_high y prom_low.


grabar_y_promediar :
				ldi r20, 0x00
				ldi r21, 0x00
comenzar_muestreo :
				ldi xl, low(adc_1)		;Apunto la variable x e y al vector adc_1
				ldi xh, high(adc_1)			
				ldi r16, 64		
							
grabar_adc1 :	lds r17, ADCL			;guardo los valores obtenidos por el ADC en vector
				lds r18, ADCH				
				st x+, r17					
				st x+, r18					
				
				rcall delay_2ms			;espero 4 ms para volver a guardar un valor
				rcall delay_2ms
				dec r16						
				brne grabar_adc1			
				
promedio_1 :	ldi xl, low(adc_1)			;Apunto la variable x al vector adc_1			
				ldi xh, high(adc_1)			; para sumar los datos almacenados				
				ldi r22, 0x00				;inicializo las variables 						
				ldi r23, 0x00				;q voy a utilizar para sumar los valores		
				
				ldi r16, 64				;sumo los 64 valores guardados en vector adc_1  
suma :			ld r17, x+					;r17 = ADCL										
				ld r18, x+					;r18 = ADCH											
				add r23, r17				;r22 r23 = val_1ADC + val_2ADC +.....+val_32_ADC
				adc r22, r18																		
				dec r16																		
				brne suma
																					
				ldi r16, 6					;divido por 32 la suma almacenada en r19 r20	
division :		lsr r22																		
				ror r23																			
				dec r16																		
				brne division				;el resultado del 1° promedio queda guardado en r22 r23 
				
				cpi	r22, 3																	
				brne promedio_2				;si el valor es menor a 1020 sigo con el 2° promedio	
				cpi r23, 251																
				brcs salir					;si el valor es mayor o igual 1020 = 1200 mts salgo del promedio 
				;con un clk = 1Mhz se tarda aprox  0,27 ms en promediar los datos
				
							
promedio_2 :	cp r20, r22					;comparo el valor almacenado con el obtenido con el adc
				brne copiar_dato			;si no es igual copio el dato en r20 r21 y vuelo a muestrear
				cp r21, r23
				breq salir					;si son iguales salgo de la subrutina

copiar_dato :	mov r20, r22
				mov r21, r23
				rjmp comenzar_muestreo		;si los valores obtenidos no son iguales promediar otra vez			
				
salir :			ret							;al salir los valores promediados quedan almacenados en r22 r23


;----------------------------------------------------------------------------
;Factor de escala
;----------------------------------------------------------------------------
;Como la entrada del ADC tiene un offset debo escalar la lectura para que me arroje el resultado correcto
;El calculo es el siguiente: Cx = m * adc - offset. Donde Cx es mi capacidad incognita ym es la pendiente
;y adc es la lectura que tomo del pin. m = 1,362 ofsett = 164.

escalar :		push r16
				push r17
											;multiplico por 87/64 aprox 1,36
				ldi r16, 87					;multiplico por 85  lo que está guardado en prom_low
				mul prom_low, r16
				mov r17, r1
				mov prom_low, r0

				mul prom_high, r16
				mov prom_high, r0
				add prom_high, r17

				ldi r16, 6
divido_x64 :	lsr prom_high																		
				ror prom_low																		
				dec r16																		
				brne divido_x64

				cpi prom_low, offset + 1 ; si la resta es negativa salto a la resta 2
				brcs resta_2
				
				subi prom_low, offset
				rjmp salir_escalar

resta_2 :		subi prom_high, 1
				ldi r17, 256 - offset
				add prom_low, r17

salir_escalar : pop r17
				pop r16

				ret
;######################################################################################################
;Rutina para convertir los datos del adc en numeros decimales ascii para representar en el lcd.
;######################################################################################################
;Esta rutina recibe valores guardados en las variables prom_high prom_low y las convierte en
;decimales en codigo ascii. Los resultados quedan almacenados en u_mil centena decena unidad.

convertir_a_longitud :
				ldi u_mil, '0'		;inicio las variables en '0' escrito en ascii
				ldi centena, '0'
				ldi decena, '0'
				ldi unidad, '0'

				cpi prom_high, 3		;comparo con el registro de promedio para saber si es menor que mil
				brne menor_a_mil
				cpi prom_low, 232
				brcs menor_a_mil
				
				ldi u_mil, '1'			;si es mayor a mil grabo 1 y 0 en u_mil y centena respec.
				ldi centena, '0'
				subi prom_low, 232		;y le resto 232 al adc_low

				rjmp sumar_decenas		;como la medicion es hasta 20 no hace falta sumar las centenas


menor_a_mil :	sbrs prom_high, 0
				rjmp bit_1				;salto si hay un 0 en el bit 0 del registro alto de promedio
				
				ldi r16, 2				;sumo 256 si el bit está en 1
				add centena, r16
				ldi r16, 5
				add decena, r16
				ldi r16, 6
				add unidad, r16

bit_1 :			sbrs prom_high, 1
				rjmp sumar_centenas		;salto si hay un 0 en el bit 1 del registro alto de promedio
				
				ldi r16, 5				;sumo 512 si el bit está en 1
				add centena, r16
				ldi r16, 1
				add decena, r16
				ldi r16, 2
				add unidad, r16


sumar_centenas :
				cpi prom_low, 100
				brcs sumar_decenas		;si el resultado en el registro bajo es menor a 100 (b = 1 sigo con las decenas
				
				subi prom_low, 100		;sino le resto 100 al registro e incremento la centena y vuelvo a comparar con 100
				inc centena
				rjmp sumar_centenas


sumar_decenas :
				cpi prom_low, 10
				brcs sumar_unidades		;si el resultado en el registro bajo es menor a 10 sigo con las unidades
				
				subi prom_low, 10		;sino le resto 10 al registro e incremento la decena y vuelvo a comparar con 10
				inc decena
				rjmp sumar_decenas


sumar_unidades : 
				add unidad, prom_low

				cpi unidad, '0' + 10			;comparo la unidad con el nro 10 en ascii
				brcs verificar_decena		;si el resultado es negativo (C = 1) verifico las decenas
				
				subi unidad, 10				;sino le resto los 10 sobrantes y le sumo uno a la decena
				inc decena

verificar_decena :
				cpi decena, '0'+ 10			;comparo la decena con el nro 10 en ascii
				brcs salir_conversor		;si el resultado es negativo (C=1) salgo de la rutina
				
				subi decena, 10				;sino le resto los 10 sobrantes y le sumo uno a la centena
				inc centena

salir_conversor:							;al salir los resultados quedan almacenados en u_mil centena decena unidad
				ret							;que son respect los registros					r18	   r19	   r20	  r21


;-----------------------------------------------------------------------------
;Constantes en memoria Flash para utilizar en pantalla LCD
;-----------------------------------------------------------------------------
MJE0 : .db "Iniciando...",0
MJE1 : .db "Probando.....",0
MJE2 : .db "MENU PRINCIPAL",0
MJE3 : .db "Probar línea",0
MJE4 : .db "Medir capacidad",0
MJE5 : .db "capacidad = ",0
MJE6 : .db " nF",0
MJE7 : .db " pF",0
MJE8 : .db "capacidad mayor a 50 nF",0
MJE9 : .db "capacidad menor a 1 nF",0




