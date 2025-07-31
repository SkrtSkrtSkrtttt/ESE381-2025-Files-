/*
 * usart3_avr128_cir_buff.c
 * Added our Usart Int function from Task 1
 * Created: 4/9/2025 7:38:57 PM
 * Author : Naafiul Hossain
 */

/*
*
* circular_buffer_avr128_usart3.c
*
* Created: 3/1/2024 6:26:29 PM
* Author : kshort
*/


#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>
#define F_CPU 4000000 // CPU clock in Hz
#define USART3_BAUD_RATE(BAUD_RATE) ((float)(4000000 * 64 / (16 * (float)BAUD_RATE)) + 0.5)

/* UART Buffer Defines */
#define USART_RX_BUFFER_SIZE 16     /* 2,4,8,16,32,64,128 or 256 bytes */
#define USART_TX_BUFFER_SIZE 16     /* 2,4,8,16,32,64,128 or 256 bytes */
#define USART_RX_BUFFER_MASK ( USART_RX_BUFFER_SIZE - 1 )
#define USART_TX_BUFFER_MASK ( USART_TX_BUFFER_SIZE - 1 )

#if ( USART_RX_BUFFER_SIZE & USART_RX_BUFFER_MASK )
#error RX buffer size is not a power of 2
#endif
#if ( USART_TX_BUFFER_SIZE & USART_TX_BUFFER_MASK )
#error TX buffer size is not a power of 2
#endif

//#define RX_BUFFER_MARGIN ((uint8_t)(0.1 * USART_RX_BUFFER_SIZE + 0.5))

/* Static Variables */
static unsigned char USART_RxBuf[USART_RX_BUFFER_SIZE];
static  uint8_t USART_RxHead;  //orig. declared volatile - kls
static  uint8_t USART_RxTail;  //orig. declared volatile - kls
static unsigned char USART_TxBuf[USART_TX_BUFFER_SIZE];
static  uint8_t USART_TxHead;  //orig. declared volatile - kls
static  uint8_t USART_TxTail;  //orig. declared volatile - kls


volatile uint8_t cntrlcBM ;

//uint8_t counter = 0;

/* Function Prototypes */
void USART3_Init(uint16_t baud, uint8_t data_bits, unsigned char parity);
uint8_t USART3_Receive( void );
void USART3_Transmit( uint8_t data );
uint8_t DataInReceiveBuffer(void);


/* Main - a simple test program*/
int main( void )
{
USART_RxTail = 0x00; //clear buffer indexes, not really necessary
USART_RxHead = 0x00; //because they are automatically cleared since
USART_TxTail = 0x00; //declared as global uninitialized variables
USART_TxHead = 0x00;

// SW0 pin an input, must be pressed to transfer data from Rx to Tx buffer
PORTB.DIR &= ~PIN2_bm; // SW0 pin pushbutton input
PORTB.PIN2CTRL = 0x08; //enable pull up
//USART3_Init();   // Initialize USART3
USART3_Init( 9600,8,'D');   // Initialize USART3
USART3.CTRLA |= USART_RXCIE_bm; /* Receive Complete Interrupt must be enabled */
sei();           // Enable global interrupts => enable USART interrupts
for( ; ; )        // Forever
{
//Uncomment next statement to have operation independent of SW0
//USART3_Transmit( USART3_Receive() );

//Uncomment next statement have operation dependent on SW0
if ( (!(VPORTB_IN & PIN2_bm)) && DataInReceiveBuffer() )
USART3_Transmit( USART3_Receive() );
}
return 0;
}



// Function to initialize USART3
void USART3_Init(uint16_t baud, uint8_t data_bits, unsigned char parity){

PORTB.DIR &= ~PIN1_bm; // RXD pin input
PORTB.DIR |= PIN0_bm; // TXD pin output
PORTB.OUT |= PIN0_bm; // TXD output value = 1
USART3.BAUD =  (uint16_t)USART3_BAUD_RATE(baud);      //baud rate

USART3.CTRLB =  0b11000000;    //transmitter and receiver enabling  as output


cntrlcBM =  0x00 ;    //frame format
//data bits format:

switch(data_bits) {
case 5:
cntrlcBM |= 0x00;
break;
case 6:
cntrlcBM |= 0x01;
break;
case 7:
cntrlcBM |= 0x02;
break;
case 8:
cntrlcBM |= 0x03;
break;
default:
cntrlcBM |= 0x00; //not valid choice
break;
}

//stop bit mode:

cntrlcBM |= 0x00; //1 stop bit
//cntrlcBM |= 0x04; // 2 stop bits

//parity format:


switch(parity) {
case 'D':
cntrlcBM |= 0x00;
break;
case 'E':
cntrlcBM |= 0x20;
break;
case 'O':
cntrlcBM |= 0x30;
break;
default:
cntrlcBM |= 0x00; //not valid choice
break;
}
//Enable Tx and Rx for operation and Tx and Rx interrupts
USART3.CTRLB |= (USART_TXEN_bm | USART_RXEN_bm);
USART3.CTRLA |= (USART_RXCIE_bm); //Enable RXCIE
}


/* Interrupt handlers */

ISR (USART3_RXC_vect) //Receive complete interrupt
{
uint8_t data;

//The following variable is not necessary if you are not going to take any action
//for an overflow that requires keeping the old index. Instead just use
//USART_RxHead instead of tmphead.
uint8_t tmphead;

cli(); // Clear global interrupt flag

/* Read the received data */
data = USART3.RXDATAL;

/* Calculate buffer index, increment and possibly roll over index */
tmphead = ( USART_RxHead + 1 ) & USART_RX_BUFFER_MASK;


//************************************************************************
/*
The following condition could be changed to
if ( (tmphead >= (USART_RxTail + RX_BUFFER_MARGIN)) || (USART_RxTail >= (tmphead + RX_BUFFER_MARGIN)));
{
//Use flow control to stop flow of characters:
(a) hardware unasserts CTS
(b) software send XOFF
}
*/
//************************************************************************

if ( tmphead == USART_RxTail )
{
// ERROR! Receive buffer overflow
}

USART_RxBuf[tmphead] = data; // Store received data in buffer
//Alternate position B for USART_RxHead = tmphead;
USART_RxHead = tmphead;      // Store new index (was prev. in position A)
sei(); // re enable global interrupts
}


ISR (USART3_DRE_vect)
{
uint8_t tmptail;
cli(); // Clear global interrupts

/* Check if all data is transmitted */
if ( USART_TxHead != USART_TxTail )
{
// Calculate buffer index
tmptail = ( USART_TxTail + 1 ) & USART_TX_BUFFER_MASK;
USART_TxTail = tmptail;      // Store new index

USART3.TXDATAL = USART_TxBuf[tmptail];  // Start transmission
}
else
{
USART3.CTRLA &= ~(USART_DREIE_bm);     // Disable UDRE interrupt
}
sei();
}


/* Read function */
unsigned char USART3_Receive( void )
{
uint8_t tmptail;

while ( USART_RxHead == USART_RxTail );  /* Wait for incomming data */
tmptail = ( USART_RxTail + 1 ) & USART_RX_BUFFER_MASK;/* Calculate buffer index */
USART_RxTail = tmptail;                /* Store new index */
return USART_RxBuf[tmptail];           /* Return data */
}

/* Write function */
void USART3_Transmit( uint8_t data )
{
uint8_t tmphead;
/* Calculate buffer index */
tmphead = ( USART_TxHead + 1 ) & USART_TX_BUFFER_MASK; /* Wait for free space in buffer */
while ( tmphead == USART_TxTail );
USART_TxBuf[tmphead] = data;           /* Store data in buffer */
USART_TxHead = tmphead;                /* Store new index */

USART3.CTRLA |= USART_DREIE_bm;                    /* Enable UDRE interrupt */
}

uint8_t DataInReceiveBuffer( void )
{
return ( USART_RxHead != USART_RxTail ); /* Return 0 (FALSE) if the receive buffer is empty */
}
