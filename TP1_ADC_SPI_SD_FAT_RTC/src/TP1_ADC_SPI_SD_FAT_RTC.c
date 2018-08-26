/* Copyleft 2018, Leonardo Urrego @trecetp
 *    Descripción:
 *    Guarda un log de datos de mediciones tomadas por el ADC, en un único archivo.
 *    Además incluye en cada muestra un time-stamp de cuando la misma fue obtenida utilizando el periférico RTC.
 *    El archivo obtenido guarda las líneas con el siguiente formato:
 *
 *    CH1;CH2;CH3;YYYY/MM/DD_hh:mm:ss;
 *
 */

/*==================[inclusions]=============================================*/

#include "sapi.h"        // <= sAPI header
#include "ff.h"       // <= Biblioteca FAT FS
#include <string.h>

/*==================[macros and definitions]=================================*/

#define FILENAME 			"logADC.txt"
#define	_TIME_MUESTREO_		1000

/*==================[internal data declaration]==============================*/

/* Estructura RTC */
rtc_t rtc;

static FATFS fs;           // <-- FatFs work area needed for each volume
static FIL fp;             // <-- File object needed for each open file


/*==================[internal functions declaration]=========================*/

/*==================[internal data definition]===============================*/

char string2log[100];
char bufferTmp[20];

/*==================[external data definition]===============================*/

DEBUG_PRINT_ENABLE;

/*==================[internal functions definition]==========================*/


/*==================[external functions definition]==========================*/


/**
 * C++ version 0.4 char* style "itoa":
 * Written by Lukás Chmela
 * Released under GPLv3.

 */
char* itoa(int value, char* result, int base) {
	// check that the base if valid
	if (base < 2 || base > 36) { *result = '\0'; return result; }

	char* ptr = result, *ptr1 = result, tmp_char;
	int tmp_value;

	do {
		tmp_value = value;
		value /= base;
		*ptr++ = "zyxwvutsrqponmlkjihgfedcba9876543210123456789abcdefghijklmnopqrstuvwxyz" [35 + (tmp_value - value * base)];
	} while ( value );

	// Apply negative sign
	if (tmp_value < 0) *ptr++ = '-';
	*ptr-- = '\0';
	while(ptr1 < ptr) {
		tmp_char = *ptr;
		*ptr--= *ptr1;
		*ptr1++ = tmp_char;
	}
	return result;
}

void addTimeStamp( rtc_t * rtc )	// Formato: YYYY/MM/DD_hh:mm:ss;
{
	/* Conversion de entero a ascii con base decimal */
	itoa( (int) (rtc->year) , (char*)bufferTmp , 10 ); 	/* 10 significa decimal */
	strcat( string2log , bufferTmp );					/* Guarda el año */
	strcat( string2log , "/" );

	itoa( (int) (rtc->month) , (char*)bufferTmp , 10 );
	if( (rtc->month)<10 )
		strcat( string2log , "0" );
	strcat( string2log , bufferTmp );					/* Guarda el mes */
	strcat( string2log , "/" );

	itoa( (int) (rtc->mday) , (char*)bufferTmp , 10 );
	if( (rtc->mday)<10 )
		strcat( string2log , "0" );
	strcat( string2log , bufferTmp );					/* Guarda el dia */
	strcat( string2log , "_" );

	itoa( (int) (rtc->hour) , (char*)bufferTmp , 10 );
	if( (rtc->hour)<10 )
		strcat( string2log , "0" );
	strcat( string2log , bufferTmp );					/* Guarda la hora */
	strcat( string2log , ":" );

	itoa( (int) (rtc->min) , (char*)bufferTmp , 10 );
	if( (rtc->min)<10 )
		strcat( string2log , "0" );
	strcat( string2log , bufferTmp );					/* Guarda los minutos */
	strcat( string2log , ":" );

	itoa( (int) (rtc->sec) , (char*)bufferTmp , 10 );
	if( (rtc->sec)<10 )
		strcat( string2log , "0" );
	strcat( string2log , bufferTmp );					/* Guarda los segundos */
	strcat( string2log , ";" );

	strcat( string2log , "\r\n" );
}

void saveLog2SD()
{
	// ------ PROGRAMA QUE ESCRIBE EN LA SD -------
	gpioWrite( LEDB, ON );
	delay(100);
	gpioWrite( LEDB, OFF );

	UINT nbytes;

	// Give a work area to the default drive
	if( f_mount( &fs, "", 0 ) != FR_OK ){
		// If this fails, it means that the function could
		// not register a file system object.
		// Check whether the SD card is correctly connected
	}

	// Create/open a file, then write a string and close it
	if( f_open( &fp, FILENAME, FA_WRITE | FA_OPEN_APPEND ) == FR_OK )
	{
		f_write( &fp , string2log , strlen( string2log ) , &nbytes );

		f_close(&fp);

		if( strlen( string2log ) == nbytes )
		{
			// Turn ON LED2 if the write operation was successful
			gpioWrite( LEDG, ON );
			delay(100);
			gpioWrite( LEDG, OFF );
		}
	}
	else
	{
		// Turn ON LEDR if the write operation was fail
		gpioWrite( LEDR, ON );
		delay(100);
		gpioWrite( LEDR, OFF );
	}
}

/* FUNCION PRINCIPAL, PUNTO DE ENTRADA AL PROGRAMA LUEGO DE RESET. */
int main(void){

	/* ------------- INICIALIZACIONES ------------- */

	/* Inicializar la placa */
	boardConfig();

	/* Inicializar UART_USB a 115200 baudios */
	uartConfig( UART_USB, 115200 );
	uartWriteString( UART_USB , "TP1_ADC_SPI_SD_FAT_RTC\r\n" );

	/* Inicializar AnalogIO */
	adcConfig( ADC_ENABLE ); /* ADC */

	// SPI configuration
	spiConfig( SPI0 );

	rtc.year = 2018;
	rtc.month = 06;
	rtc.mday = 29;
	rtc.wday = 1;
	rtc.hour = 20;
	rtc.min = 30;
	rtc.sec= 0;

	/* Inicializar RTC */
	rtcConfig( &rtc );

	delay(2000); // El RTC tarda en setear la hora, por eso el delay

	/* Variable para almacenar el valor leido del ADC CH1 */
	uint16_t muestra_CH1 = 0;
	uint16_t muestra_CH2 = 0;
	uint16_t muestra_CH3 = 0;

	/* Variables de delays no bloqueantes */
	delay_t delay1;

	/* Inicializar Retardo no bloqueante con tiempo en ms */
	delayConfig( &delay1, _TIME_MUESTREO_ );

	/* ------------- REPETIR POR SIEMPRE ------------- */
	while(1) {

		/* delayRead retorna TRUE cuando se cumple el tiempo de retardo */
		if ( delayRead( &delay1 ) )
		{
			string2log[0] 	= '\0';
			bufferTmp[0]	= '\0';
			rtcRead( &rtc );
			/* Leo la Entrada Analogica AI0 - ADC0 CH1 */
			muestra_CH1 = adcRead( CH1 );
			muestra_CH2 = adcRead( CH2 );
			muestra_CH3 = adcRead( CH3 );

			// uartWriteString( UART_USB, "ADC CH1 value: " );
			itoa( muestra_CH1  , bufferTmp , 10 );				/* Conversión de muestra entera a ascii con base decimal */
			strcat( string2log , bufferTmp );
			strcat( string2log , ";" );
			// /* Enviar muestra y Enter */
			// uartWriteString( UART_USB, bufferTmp );
			// uartWriteString( UART_USB, ";\r\n" );

			// uartWriteString( UART_USB, "ADC CH2 value: " );
			itoa( muestra_CH2  , bufferTmp , 10 );				/* Conversión de muestra entera a ascii con base decimal */
			strcat( string2log , bufferTmp );
			strcat( string2log , ";" );
			// /* Enviar muestra y Enter */
			// uartWriteString( UART_USB, bufferTmp );
			// uartWriteString( UART_USB, ";\r\n" );

			// uartWriteString( UART_USB, "ADC CH3 value: " );
			itoa( muestra_CH3  , bufferTmp , 10 );				/* Conversión de muestra entera a ascii con base decimal */
			strcat( string2log , bufferTmp );
			strcat( string2log , ";" );
			// /* Enviar muestra y Enter */
			// uartWriteString( UART_USB, bufferTmp );
			// uartWriteString( UART_USB, ";\r\n" );

			addTimeStamp( &rtc );

			uartWriteString( UART_USB , string2log );			// Muestra el resultado por la UART

			saveLog2SD();
		}

	}

	/* NO DEBE LLEGAR NUNCA AQUI, debido a que a este programa no es llamado
		por ningun S.O. */
	return 0 ;
}

/*==================[end of file]============================================*/
