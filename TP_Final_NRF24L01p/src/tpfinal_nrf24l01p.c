/**
 * Proyecto:	TP Final de PCSE CESE
 * Autor:		Carlos Leonardo Urrego
 * Fecha:		Agosto de 2018
 * Descripción:	Primera implementación del driver del radio NRF24L01+ sobre la EDU-CIAA
 *
 */

/*==================[inlcusiones]============================================*/

#include "chip.h"
#include "sapi.h"                // <= sAPI header
#include "tpfinal_nrf24l01p.h"   // <= own header (optional)


/*==================[definiciones y macros]==================================*/

#define _GPIO_NRF24_CE_		GPIO0
#define _GPIO_NRF24_CSN_	GPIO1
#define _SPI_BITRATE_		1000000
#define _BUFFER_SIZE_		40

#define __NRF24_CE_OFF()	gpioWrite( _GPIO_NRF24_CE_, OFF );
#define __NRF24_CE_ON()		gpioWrite( _GPIO_NRF24_CE_, ON );
#define __NRF24_CS_OFF()	gpioWrite( _GPIO_NRF24_CSN_, ON );
#define __NRF24_CS_ON()		gpioWrite( _GPIO_NRF24_CSN_, OFF );

/*==================[definiciones de datos internos]=========================*/

// const uint64_t pipe = 0xE7E7E7E7E7;
// byte address[][5] = { 0xCC,0xCE,0xCC,0xCE,0xCC , 0xCE,0xCC,0xCE,0xCC,0xCE};
uint8_t address[] = { 0xE7,0xE7,0xE7,0xE7,0xE7};		// Dirección de prueba

// Struct que maneja los datos del SPI
static Chip_SSP_DATA_SETUP_T spiData;

/* Buffers */
static char uartBuff[ 32 ];
static uint8_t spiTx_Buff[ _BUFFER_SIZE_ ];
static uint8_t spiRx_Buff[ _BUFFER_SIZE_ ];
/*==================[definiciones de datos externos]=========================*/

//================================================[SPI Management]========================================================
void SPI_init(uint32_t bitrateconfig)
{
	// Configure SSP SSP1 pins
	Chip_SCU_PinMuxSet(0xf, 4, (SCU_MODE_PULLUP | SCU_MODE_FUNC0)); // CLK0
	Chip_SCU_PinMuxSet(0x1, 3, (SCU_MODE_PULLUP | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS | SCU_MODE_FUNC5)); // MISO1
	Chip_SCU_PinMuxSet(0x1, 4, (SCU_MODE_PULLUP | SCU_MODE_FUNC5)); // MOSI1

	// Chip_SCU_PinMuxSet(0xF, 4, (SCU_PINIO_FAST | SCU_MODE_FUNC0));  /* PF.4 => SCK1 */
	// Chip_SCU_PinMuxSet(0x1, 3, (SCU_MODE_INACT | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS | SCU_MODE_FUNC5)); /* P1.3 => MISO1 */
	// Chip_SCU_PinMuxSet(0x1, 4, (SCU_MODE_INACT | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS | SCU_MODE_FUNC5)); /* P1.4 => MOSI1 */

	// Chip_SCU_PinMuxSet(0x6, 1, (SCU_MODE_PULLUP | SCU_MODE_FUNC0)); // CS1 configured as GPIO
	// Chip_GPIO_SetPinDIROutput(LPC_GPIO_PORT, 3, 0);

	// Initialize SSP Peripheral
	Chip_SSP_Init( LPC_SSP1 );
	// Chip_Clock_Enable( Chip_SSP_GetClockIndex( LPC_SSP1 ) );
	// Chip_Clock_Enable( Chip_SSP_GetPeriphClockIndex( LPC_SSP1 ) );

	Chip_SSP_Set_Mode( LPC_SSP1 , SSP_MODE_MASTER );
	Chip_SSP_SetFormat( LPC_SSP1 , SSP_BITS_8 , SSP_FRAMEFORMAT_SPI , SSP_CLOCK_CPHA0_CPOL0 );
	Chip_SSP_SetBitRate( LPC_SSP1 , bitrateconfig );

	Chip_SSP_Enable( LPC_SSP1 );
}

void Board_SSP_Init(void)
{
	Chip_SCU_PinMuxSet(0xF, 4, (SCU_PINIO_FAST | SCU_MODE_FUNC0));  /* PF.4 => SCK1 */
	Chip_SCU_PinMuxSet(0x1, 4, (SCU_MODE_INACT | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS | SCU_MODE_FUNC5)); /* P1.4 => MOSI1 */
	Chip_SCU_PinMuxSet(0x1, 3, (SCU_MODE_INACT | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS | SCU_MODE_FUNC5)); /* P1.3 => MISO1 */
}

void Board_SSP_config(uint8_t bits, uint8_t clockMode, uint32_t bitrate)
{
	uint32_t bitsConfig= SSP_BITS_8;
	switch(bits)
	{
		case 4: bitsConfig=SSP_BITS_4;break;
		case 8: bitsConfig=SSP_BITS_8;break;
		case 16: bitsConfig=SSP_BITS_16;break;
	}
	uint32_t clockModeConfig = SSP_CLOCK_CPHA0_CPOL0;
	switch(clockMode)
	{
		case 0: clockModeConfig = SSP_CLOCK_CPHA0_CPOL0;break;
		case 1: clockModeConfig = SSP_CLOCK_CPHA1_CPOL0;break;
		case 2: clockModeConfig = SSP_CLOCK_CPHA0_CPOL1;break;
		case 3: clockModeConfig = SSP_CLOCK_CPHA1_CPOL1;break;
	}
        Chip_SSP_Init(LPC_SSP1);
        Chip_SSP_SetFormat(LPC_SSP1, bitsConfig, SSP_FRAMEFORMAT_SPI, clockModeConfig);
        Chip_SSP_SetBitRate(LPC_SSP1, bitrate);
        Chip_SSP_Enable(LPC_SSP1);
}

uint32_t Board_SSP_writeBuffer(const uint8_t *buffer, uint32_t bufferLen)
{
	return Chip_SSP_WriteFrames_Blocking(LPC_SSP1, buffer, bufferLen);
}

uint32_t Board_SSP_readBuffer(uint8_t *buffer, uint32_t bufferLen)
{
	return Chip_SSP_ReadFrames_Blocking(LPC_SSP1,buffer, bufferLen);
}

uint32_t Board_SSP_transfer(uint8_t *bufferTx, uint8_t *bufferRx, uint32_t bufferLen)
{
	Chip_SSP_DATA_SETUP_T spiSetup;
	spiSetup.tx_data = bufferTx;
	spiSetup.rx_data = bufferRx;
	spiSetup.tx_cnt=0;
	spiSetup.rx_cnt=0;
	spiSetup.length=bufferLen;

	return Chip_SSP_RWFrames_Blocking(LPC_SSP1, &spiSetup);
}

/*==================[declaraciones de funciones internas]====================*/

// FUNCION que se ejecuta cada vezque ocurre un Tick
void diskTickHook( void *ptr );

/*==================[declaraciones de funciones externas]====================*/
/**
 * C++ version 0.4 char* style "itoa":
 * Written by Lukás Chmela
 * Released under GPLv3.

 */
char* itoa(int value, char* result, int base)
{
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

void uartWriteNum( uartMap_t uart , int num , int base)
{
	static char tmpBuff[10];
	itoa( num , tmpBuff , base);
	uartWriteString( uart, tmpBuff );
	// uartWriteByte( uart, 0x20 );
}

void uartWriteNumln( uartMap_t uart , int num , int base)
{
	static char tmpBuff[10];
	itoa( num , tmpBuff , base);
	uartWriteString( uart, tmpBuff );
	uartWriteString( UART_USB, "\r\n" ); /* Enviar un Enter */
	// uartWriteByte( uart, 0x20 );
}

void uartWriteHEXarr( uartMap_t uart , uint8_t* arr , uint8_t num )
{
	static char tmpBuff[5];
	for(uint8_t i = 0 ; i < num ; i++)
	{
		itoa( arr[ i ] , tmpBuff , 16);
		uartWriteString( uart, tmpBuff );
		uartWriteByte( uart, 0x20 );
	}
	uartWriteString( UART_USB, "\r\n" ); /* Enviar un Enter */
}

void clearRxTxBuff()
{
	for( uint8_t i = 0 ; i < _BUFFER_SIZE_ ; i++)
		spiTx_Buff[ i ] = spiRx_Buff[ i ] = 0;
}

void RF24TrasferLog( )
{
	uartWriteString( UART_USB, "Datos Transferidos:\r\n" );

	uartWriteString( UART_USB, "tx_cnt: " );
	uartWriteNumln( UART_USB , spiData.tx_cnt , 10 );
	uartWriteHEXarr( UART_USB , spiTx_Buff , _BUFFER_SIZE_);

	uartWriteString( UART_USB, "rx_cnt: " );
	uartWriteNumln( UART_USB , spiData.rx_cnt , 10 );
	uartWriteHEXarr( UART_USB , spiRx_Buff , _BUFFER_SIZE_);
	uartWriteString( UART_USB, "\r\n" ); /* Enviar un Enter */
}

uint8_t RF24transfer(uint8_t num)
{
	spiData.rx_cnt = spiData.tx_cnt = 0;
	spiData.length = num;
	__NRF24_CS_ON();
	Chip_SSP_RWFrames_Blocking(LPC_SSP1, &spiData);
	__NRF24_CS_OFF();
	RF24TrasferLog();
	return spiTx_Buff[0]; // status is 1st byte of receive buffer
}

uint8_t RF24write_register(uint8_t reg, uint8_t value)
{
	uint8_t status;
	spiTx_Buff[0] = ( W_REGISTER | ( REGISTER_MASK & reg ) );
	spiTx_Buff[1] = value ;

	RF24transfer( 2 );
	status = spiTx_Buff[0]; // status is 1st byte of receive buffer
	return status;
}

uint8_t RF24write_registerarr(uint8_t reg, const uint8_t* buf, uint8_t len)
{
	uint8_t status;
	uint8_t size = len + 1; // Add register value to transmit buffer

	uint8_t * ptx = spiTx_Buff;

	*ptx++ = ( W_REGISTER | ( REGISTER_MASK & reg ) );
	while ( len-- )
		*ptx++ = *buf++;

	RF24transfer( size );
	status = spiTx_Buff[0]; // status is 1st byte of receive buffer
	return status;
}

uint8_t RF24read_register(uint8_t reg)
{
	uint8_t result;

	spiTx_Buff[0] = ( R_REGISTER | ( REGISTER_MASK & reg ) );
	spiTx_Buff[1] = RF24_NOP ; // Dummy operation, just for reading

	RF24transfer( 2 );
	result = spiRx_Buff[1];   // result is 2nd byte of receive buffer
	return result;
}

void RF24setRetries(uint8_t delay, uint8_t count)
{
	// RF24write_register(SETUP_RETR,(delay&0xf)<<ARD | (count&0xf)<<ARC);
}

bool_t RF24setDataRate(rf24_datarate_e speed)
{
	bool result = false;
	uint8_t setup = RF24read_register(RF_SETUP) ;

	// HIGH and LOW '00' is 1Mbs - our default
	setup &= ~(_BV(RF_DR_LOW) | _BV(RF_DR_HIGH)) ;

	if( speed == RF24_250KBPS )
	{
		// Must set the RF_DR_LOW to 1; RF_DR_HIGH (used to be RF_DR) is already 0
		// Making it '10'.
		setup |= _BV( RF_DR_LOW ) ;
	}
	else
	{
		// Set 2Mbs, RF_DR (RF_DR_HIGH) is set 1
		// Making it '01'
		if ( speed == RF24_2MBPS )
		{
			setup |= _BV(RF_DR_HIGH);
		}
	}

	RF24write_register(RF_SETUP,setup);

	// Verify our result
	if ( RF24read_register(RF_SETUP) == setup )
	{
		result = true;
	}

	return result;
}

void RF24setCRCLength(rf24_crclength_e length)
{
	uint8_t config = RF24read_register(NRF_CONFIG) & ~( _BV(CRCO) | _BV(EN_CRC)) ;

	if ( length == RF24_CRC_DISABLED )
	{
		// Do nothing, we turned it off above.
	}
	else if ( length == RF24_CRC_8 )
	{
		config |= _BV(EN_CRC);
	}
	else
	{
		config |= _BV(EN_CRC);
		config |= _BV( CRCO );
	}

	RF24write_register( NRF_CONFIG, config ) ;
}

void RF24setChannel(uint8_t channel)
{
  const uint8_t max_channel = 125;
  RF24write_register(RF_CH,rf24_min(channel,max_channel));
}

void RF24flush_rx(void)
{
	spiTx_Buff[ 0 ] = FLUSH_RX;
	RF24transfer( 2 );
}

void RF24flush_tx(void)
{
	spiTx_Buff[ 0 ] = FLUSH_TX;
	RF24transfer( 2 );
}

uint8_t RF24get_status(void)
{
	spiTx_Buff[ 0 ] = RF24_NOP;
	return RF24transfer( 1 );
}

void RF24powerDown(void)
{
  __NRF24_CE_OFF(); // Guarantee CE is low on powerDown
  RF24write_register(NRF_CONFIG, RF24read_register(NRF_CONFIG) & ~_BV(PWR_UP));
}

/****************************************************************************/

//Power up now. Radio will not power down unless instructed by MCU for config changes etc.
void RF24powerUp(void)
{
   uint8_t cfg = RF24read_register(NRF_CONFIG);

   // if not powered up then power up and wait for the radio to initialize
   if (!(cfg & _BV(PWR_UP))){
      RF24write_register(NRF_CONFIG, cfg | _BV(PWR_UP));

      // For nRF24L01+ to go from power down mode to TX or RX mode it must first pass through stand-by mode.
	  // There must be a delay of Tpd2stby (see Table 16.) after the nRF24L01+ leaves power down mode before
	  // the CEis set high. - Tpd2stby can be up to 5ms per the 1.0 datasheet
      delay(5);
   }
}

void RF24openWritingPipe(const uint8_t *address)
{
  // Note that AVR 8-bit uC's store this LSB first, and the NRF24L01(+)
  // expects it LSB first too, so we're good.

  RF24write_registerarr(RX_ADDR_P0,address, 5);
  RF24write_registerarr(TX_ADDR, address, 5);

  // const uint8_t max_payload_size = 32;
  // RF24write_register(RX_PW_P0,rf24_min(payload_size,max_payload_size));
  // RF24write_register(RX_PW_P0,payload_size);
}

uint8_t RF24write_payload(const void* buf, uint8_t data_len, const uint8_t writeType)
{
	uint8_t status;
	uint8_t payload_size = 32;
	const uint8_t* current = (const uint8_t*)(buf);

	data_len = rf24_min(data_len, payload_size);
	uint8_t blank_len = payload_size - data_len;

	uint8_t * ptx = spiTx_Buff;
	uint8_t size;
	size = data_len + blank_len + 1 ; // Add register value to transmit buffer

	*ptx++ =  writeType;
	while ( data_len-- )
		*ptx++ =  *current++;
	while ( blank_len-- )
		*ptx++ =  0;

	RF24transfer( size );
	status = spiRx_Buff[ 0 ]; // status is 1st byte of receive buffer

	return status;
}

//Per the documentation, we want to set PTX Mode when not listening. Then all we do is write data and set CE high
//In this mode, if we can keep the FIFO buffers loaded, packets will transmit immediately (no 130us delay)
//Otherwise we enter Standby-II mode, which is still faster than standby mode
//Also, we remove the need to keep writing the config register over and over and delaying for 150 us each time if sending a stream of data

void RF24startFastWrite( const void* buf, uint8_t len, const bool multicast, bool startTx)
{
	//write_payload( buf,len);
	RF24write_payload( buf , len , multicast ? W_TX_PAYLOAD_NO_ACK : W_TX_PAYLOAD ) ;
	// if(startTx){
	// 	__NRF24_CE_ON();	// ce(HIGH);
	// }
}

bool RF24write( const void* buf, uint8_t len )
{
	//Start Writing
	// RF24startFastWrite(buf,len,0,1);
	RF24write_payload( buf , len , W_TX_PAYLOAD ) ;

	__NRF24_CE_ON();	// ce(HIGH);
	delay(1);
	__NRF24_CE_OFF(); //ce(LOW);

	uint8_t status = RF24write_register(NRF_STATUS,_BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT) );

	//Max retries exceeded
	if( status & _BV(MAX_RT)){
		RF24flush_tx(); //Only going to be 1 packet int the FIFO at a time using this method, so just flush
		return 0;
	}
	//TX OK 1 or 0
	return 1;
}

bool_t RF24ini( gpioMap_t cepin , gpioMap_t csnpin )
{
 	uint8_t setup=0;

	// Configura pines de control
	gpioConfig( cepin  , GPIO_OUTPUT );
	gpioConfig( csnpin , GPIO_OUTPUT );
	__NRF24_CE_OFF();
	__NRF24_CS_OFF();

	clearRxTxBuff();

	spiData.length = _BUFFER_SIZE_;
	spiData.tx_data = spiTx_Buff;
	spiData.rx_data = spiRx_Buff;
	spiData.rx_cnt = spiData.tx_cnt = 0;

	delay( 5 ) ;

	// Reset NRF_CONFIG and enable 16-bit CRC.
	RF24write_register( NRF_CONFIG, 0x0C ) ;

	// Set 1500uS (minimum for 32B payload in ESB@250KBPS) timeouts, to make testing a little easier
	// RF24setRetries(5,15);

	// Reset value is MAX
	//setPALevel( RF24_PA_MAX ) ;

	setup = RF24read_register(RF_SETUP);

	// Then set the data rate to the slowest (and most reliable) speed supported by all
	// hardware.
	RF24setDataRate( RF24_1MBPS ) ;

	// Initialize CRC and request 2-byte (16bit) CRC
	// RF24setCRCLength( RF24_CRC_16 ) ;

	// Disable dynamic payloads, to match dynamic_payloads_enabled setting - Reset value is 0
	// toggle_features();
	// write_register(FEATURE,0 );
	// write_register(DYNPD,0);
	// dynamic_payloads_enabled = false;

	// Reset current status
	// Notice reset and flush is the last thing we do
	// write_register(NRF_STATUS,_BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT) );

	// Set up default configuration.  Callers can always change it later.
	// This channel should be universally safe and not bleed over into adjacent
	// spectrum.
	RF24setChannel(76);

	// Flush buffers
	RF24flush_rx();
	RF24flush_tx();

	RF24powerUp(); //Power up by default when begin() is called

	// Enable PTX, do not write CE high so radio will remain in standby I mode ( 130us max to transition to RX or TX instead of 1500us from powerUp )
	// PTX should use only 22uA of power
	// RF24write_register(NRF_CONFIG, ( RF24read_register(NRF_CONFIG) & ~_BV(PRIM_RX) ) );

	// if setup is 0 or ff then there was no response from module
	return ( setup != 0 && setup != 0xff );
}


/*==================[funcion principal]======================================*/

// FUNCION PRINCIPAL, PUNTO DE ENTRADA AL PROGRAMA LUEGO DE ENCENDIDO O RESET.
int main( void )
{
	// ---------- CONFIGURACIONES ------------------------------
	// Inicializar y configurar la plataforma
	boardConfig();
	uartConfig( UART_USB, 115200 );
	SPI_init( _SPI_BITRATE_ );
	RF24ini( _GPIO_NRF24_CE_ , _GPIO_NRF24_CSN_ );

	uartWriteString( UART_USB, "TEST RF24L01p EDU-CIAA\r\n" );
	// Inicializar el conteo de Ticks con resolucion de 10ms,
	// con tickHook diskTickHook
	tickConfig( 50 );
	tickCallbackSet( diskTickHook, NULL );

	// RF24openWritingPipe( address );
	__NRF24_CE_ON();

	bool_t tec1Value = ON;
	gpioWrite( LEDR, tec1Value );

	// ------ TEST -------
	// uartWriteString( UART_USB, "0x04: " );
	// uartWriteNumln( UART_USB , RF24read_register( 0x04 ) , 16 );

	// uartWriteString( UART_USB, "0x0A: " );
	// uartWriteNumln( UART_USB , RF24read_register( 0x0A ) , 16 );
	// spiTx_Buff[ 0 ] = 0x05;
	// RF24transfer( _BUFFER_SIZE_ );

	// delay(10);

	// spiTx_Buff[ 0 ] = 0x02;
	// RF24transfer( _BUFFER_SIZE_ );

	// itoa( 0 , uartBuff , 16);
	// uartWriteString( UART_USB, uartBuff );
	// uartWriteString( UART_USB, "\r\n" ); /* Enviar un Enter */

	// itoa( 1 , uartBuff , 16);
	// uartWriteString( UART_USB, uartBuff );
	// uartWriteString( UART_USB, "\r\n" ); /* Enviar un Enter */

	// itoa( 0x0F , uartBuff , 16);
	// uartWriteString( UART_USB, uartBuff );
	// uartWriteString( UART_USB, "\r\n" ); /* Enviar un Enter */

	// itoa( 0xFEAA , uartBuff , 16);
	// uartWriteString( UART_USB, uartBuff );
	// uartWriteString( UART_USB, "\r\n" ); /* Enviar un Enter */

	// uartWriteHEXarr( UART_USB , address , 5 );

	// ---------- REPETIR POR SIEMPRE --------------------------
	while( TRUE )
	{
		bool_t tec1ValueNew = gpioRead( TEC1 );

		if ( tec1ValueNew != tec1Value )
		{
			tec1Value = tec1ValueNew;
			RF24write( &tec1Value , sizeof(tec1Value) );
			gpioWrite( LEDR, tec1Value );
			while( tec1ValueNew == gpioRead( TEC1 ));
		}

		/* Prendo el led azul */
		// gpioWrite( _GPIO_NRF24_CE_, OFF );
		// gpioWrite( _GPIO_NRF24_CSN_, ON );
		// delay(500);

		/* Apago el led azul */
		// gpioWrite( _GPIO_NRF24_CE_, ON );
		// gpioWrite( _GPIO_NRF24_CSN_, OFF );
		// delay(500);
		sleepUntilNextInterrupt();
	}

	// NO DEBE LLEGAR NUNCA AQUI, debido a que a este programa se ejecuta
	// directamenteno sobre un microcontroladore y no es llamado/ por ningun
	// Sistema Operativo, como en el caso de un programa para PC.
	return 0;
}

/*==================[definiciones de funciones internas]=====================*/

/*==================[definiciones de funciones externas]=====================*/

// FUNCION que se ejecuta cada vezque ocurre un Tick
void diskTickHook( void *ptr ){
	disk_timerproc();   // Disk timer process
}


/*==================[fin del archivo]========================================*/
