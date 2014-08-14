////////////////////////////////////////////////////////////////////////////////////////////////////////
// ICOM UX-14 redesign by DK4UG 08/2014, version 0.2
// code snippets of CI-4/CI-5 command conversion are based on ...
// adaption/improvements
// - code completely rewritten
//
// - HW 74ls244/373 removed & CI-4 HW/SW interface regarding adapted, one additional line needed,
//   \SQR_LEDCI4 combined with CI-4 LED
// - added boot loader to change firmware via terminal program
//
// (- added macro's to compile independently of any avar lib's for porting e.g. towards  mikroC)
////////////////////////////////////////////////////////////////////////////////////////////////////////
// HW will be based on Arduino Nano with ATmega328p at 16MHz:
// features:
// +voltage regulator
// +USART->USB interface
// +FLASH 32kByte
// +RAM 2KByte
// +EEPROM 1KByte
// +Arduino boot loader
// 
// DB0-DB3 = PC0-PC3, input/output
// DB4-DB7 = PB0-PB3, input/output
// RXD = PD0, input
// TXD = PD1, output
// \SQR_LEDCI4 = PD2 = service request/CI-4 activity LED, output
// \DAV = PD3 = data valid, input 
// \I = P? = read data, CPU \RD & A15, input
// \C = P? = write data, CPU \WR & A15, input
//
// WP = PD4, write request, output
// RP = PD5, read request, output
// 
// PC4 = CI-5 activity LED
//
// for future extensions
// 18 - PB4 - A=B
// 19 - PB5 - A/B
// 13 - PD7 - "1" VFO_B 
//////////////////////////////////////////////////////////////////////////////////////////////////////
// CT-4 interface procedure based on Ham Radio 04/1986
/*
WRITE DATA / only one command
0. set/configure port on input or tristate
1. Set \SQR_LEDCI4 on "0" to get attention
(2. put D0-D7 on the data bus)
3. set WP on "1", request to write
(3.1 wait on read_data (\I) set on "0" or interrupt on edge (H->L) and set port on output)
(3.2 wait on read_data (\I)set on "1" or interrupt on edge (L->H) and set port on input)
4. wait on \DAV set on "0", ACK - data received
5. set WP back on "0" ACK the ACK
6. wait on \DAV set on "1"
7. go to 2. and repeat step 2-6. until finished
8. set \SQR_LEDCI4 on "1" to finish transmission
READ DATA
0. set port on input
1. Set \SQR_LEDCI4 on "0" to get attention
2. set RP on "1", request to read
(2.1 read data bus if signal \C is on "0" or interrupt on edge H->L)
3. wait on \DAV set on "0", ACK - data ready 
(4. read data bus D0-D7)
5. set RP back on "0" ACK - data received
6. wait on \DAV set on "1"
7. go to 2. and repeat step 2-6. until finished
8. set \SQR_LEDCI4 on "1" to finish transmission
*/
//////////////////////////////////////////////////////////////////////////////////////////////////////

#include <avr/io.h>

//#include	<avr/interrupt.h>
//#include	<avr/boot.h>
//#include	<avr/pgmspace.h>

//#include <avr/sfr_defs.h>
//#include <avr/cpufunc.h>
//#include <setjmp.h>
//#include baudrate.h>
//#include <util/twi.h>
//#include <avr/sleep.h>
//#include <avr/interrupt.h>
//#include <avr/wdt.h>

//////////////////////////////////////////////////////////////////////////////////////////////////////

//#define F_CPU 4000000UL // clock 4MHz
//#define F_CPU 8000000UL // clock 8MHz
#define F_CPU 16000000UL // clock 16MHz

//#define BAUD 1200UL
//#define BAUD 2400UL
//#define BAUD 4800UL
#define BAUD 9600UL 
//#define BAUD 14400UL
//#define BAUD 19200UL
//#define BAUD 38400UL

// #include <util/setbaud.h>
#include <util/delay.h>

//#define RADIO_ADDRESS 0x1C // IC-751A transceiver address
#define RADIO_ADDRESS 0x5E // to work with most new programs set on  IC-781 transceiver address

// HW related definitions
#define D0 PC0 // port for data bus bit0-3
#define D1 PC1
#define D2 PC2
#define D3 PC3
#define D4 PB0 // port for data bus bit4-7
#define D5 PB1
#define D6 PB2
#define D7 PB3

#define READ_DATA PD1 // I - bus signal
#define WRITE_DATA PC5 /// C - bus signal

#define SQR_LEDCI4 PD2
#define DAV PD3
#define WP PD4
#define RP PD5
#define LED_CI5 PC4

// error return values
/* #define CONVERT_ERROR 0xE1
#define CI5_TX_ERROR 0xE2
#define CI5_RX_ERROR 0xE3
#define CI4_TX_ERROR 0xE4
#define CI4_RX_ERROR 0xE5
#define CI5_TX_END 0x00
#define CI5_RX_END 0x00
#define CI4_TX_END 0x00
#define CI4_RX_END 0x00
#define STATUS_OK 0x00
*/
#define BUFFER_SIZE 32

//////////////////////////////////////////////////////////////////////////////////////////////////////

// macro definitions
#define BIT_CLEAR(sfr, bit) !(sfr & (1 << bit))
#define BIT_SET(sfr, bit) sfr & (1 << bit)

#define SET_BIT(reg, bit)   {reg |= (1 << (bit));}
#define CLEAR_BIT(reg, bit) {reg &= ~(1 << (bit));}


#define SQR_LEDCI4_DEACTIVE SET_BIT(PORTD, SQR_LEDCI4) // set \SQR_LEDCI4 "1"
#define SQR_LEDCI4_ACTIVE CLEAR_BIT(PORTD, SQR_LEDCI4) // set \SQR_LEDCI4 "0"

#define RP_ACTIVE SET_BIT(PORTD, RP) // set RP "1"
#define RP_DEACTIVE CLEAR_BIT(PORTD, RP) // set RP "0"

#define WP_ACTIVE SET_BIT(PORTD, WP) // set WP "1"
#define WP_DEACTIVE CLEAR_BIT(PORTD, WP) // set WP "0"

#define LED_CI5_ON SET_BIT(PORTC, LED_CI5) // LED RS-232 ACTIVITY on
#define LED_CI5_OFF CLEAR_BIT(PORTC, LED_CI5) // LED RS-232 ACTIVITY off

#define TIMEOUT_TICK {_delay_us(80); timeout++;}
#define TIMEOUT_RESET timeout = 1;

// set data ports on output
#define SET_PORTCB_OUPUT {DDRC |= (1<<D0)|(1<<D1)|(1<<D2)|(1<<D3); DDRB |= (1<<D4)|(1<<D5)|(1<<D6)|(1<<D7);}
// set data ports back to input, consider propagation delays!
#define SET_PORTCB_INPUT {DDRC &= ~((1<<D0)|(1<<D1)|(1<<D2)|(1<<D3)); DDRB &= ~((1<<D4)|(1<<D5)|(1<<D6)|(1<<D7));}

//////////////////////////////////////////////////////////////////////////////////////////////////////

// typ definitions
typedef unsigned char uint8;

// global variable declarations
static uint8 ci5_data[BUFFER_SIZE]; // RS-232/CI-5 buffer
static uint8 ci4_data[BUFFER_SIZE]; // CI-4 buffer
static uint8 num; // number of valid bytes to transmit/receive
enum ERROR_STATUS {
	CONVERT_ERROR,
	CI5_TX_ERROR,
	CI5_RX_ERROR,
	CI4_TX_ERROR,
	CI4_RX_ERROR,
	CI5_TX_END,
	CI5_RX_END,
	CI4_TX_END,
	CI4_RX_END,
	STATUS_OK
} status ; // error status 

// global function declarations
static uint8 ci5_ng (void); // CI-5 NG message, command not done 
static uint8 ci5_ok (void); // CI-5 OK message, command done
static void error (uint8 status_flag); // error handling  
static uint8 ci5_tx (uint8); // transmit CI-5 command via RS232
static uint8 ci5_rx (uint8); // receive CI-5 command via RS232
static uint8 ci4_to_ci5 (void);
static uint8 ci5_to_ci4 (void);
static uint8 ci4_tx (uint8); // transmit CI-4 command
static uint8 ci4_rx (uint8); // receive CI-4 command

//////////////////////////////////////////////////////////////////////////////////////////////////////
// start main program
//////////////////////////////////////////////////////////////////////////////////////////////////////

int main(void) {
 
         // configuration and initialization

        // initialize port's
        SET_PORTCB_INPUT;
        // setup pull-up's, also for unused pin's?
        PORTB =  PORTC = PORTD = 0xFF; // pull-up's on

        // SQR_LEDCI4_DEACTIVE;
        SET_BIT (DDRD, SQR_LEDCI4); // set as output
        
        LED_CI5_OFF;
        SET_BIT (DDRC, LED_CI5); // set as output

        WP_DEACTIVE;
        SET_BIT (DDRD, WP); // set as output
        
        RP_DEACTIVE;
        SET_BIT (DDRD, RP);        // set as output

        // initialize USART
        UBRR0 = (F_CPU / 16 / BAUD)-1; //set baud rateU2X = 0
        // UBRR = (F_CPU / 8 / BAUD)-1; //set baud rateU2X = 1

        UCSR0B =  (1<<RXEN0) | (1<<TXEN0); // enable receiver and transmitter
        UCSR0C =  (1<<USBS0) | (3<<UCSZ00); // set frame format: 8data, 2stop bit
        
        // watchdog start

//////////////////////////////////////////////////////////////////////////////////////////////////////
// main loop - start
//////////////////////////////////////////////////////////////////////////////////////////////////////

        while (1) {
                
                 status = STATUS_OK;
                num = 0;

                // wait on CI-5 Rx activity and read data
                status = ci5_rx (num);
                // check error status
                error (status);

                // convert CI-5 to CI-4 command and send to transceiver
                num = ci5_to_ci4();
                status = num;
                error (status);

                if (ci5_data[4] == 0x19) {
                        // request to send transceiver ID
                        //command ID 0x19, where come from?
                        status = ci5_tx(num);
                } 
                else {
                        status = ci4_tx(num);        
                }
                error (status);
                
                // protocol of expected command response on CI-5 commands
                // ??? complete?!
                switch (ci5_data[4]) {
                        case (0x00): // no response needed to CI-5 controller?!
                                continue;
                        case (0x01):
                                continue;
                        case (0x02): // get answer from transceiver and send to CI-5 controller
                                num = 16;
                                break; 
                        case (0x03):
                                num = 10;
                                break; 
                        case (0x04):
                                num = 3;
                                break;
                        default:
                                status = ci5_ok(); // direct ok response to CI-5 controller?!
                                // error (status);
                                continue;
                }

                //continue to get response from transceiver and send to CI-5 controller
                status = ci4_rx (num); // get CI-4 command
                error (status);
                
                num = ci4_to_ci5(); // convert command
                status = num;
                error (status);
                
                status = ci5_tx (num); // transmiT CI-5 command via RS-232
                error (status);
        }
        return (0);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
// error handling
//////////////////////////////////////////////////////////////////////////////////////////////////////

void error (uint8 status_flag) {
        if (status_flag != STATUS_OK) {
                // switch on/off LED and repeat x-times
                // switch on/off buzzer and repeat x-times

                switch (status_flag) {
                        
                        // case (CI4_RX_ERROR):
                        //         ci5_ng();
                        //         break;
                        // case (CI4_TX_ERROR):
                        //         ci5_ng();
                        //         break;
                        
                        default:
                                ci5_ng();
                                break;
                }
                 // main();
                goto *&main;
        }
}        

//////////////////////////////////////////////////////////////////////////////////////////////////////
// debug mode, echo commands to TWI-> 2. controller/ USART
//////////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////////////////
// watchdog handling
//////////////////////////////////////////////////////////////////////////////////////////////////////
/*
void wdt_reset (void) {
// something
}
*/
//////////////////////////////////////////////////////////////////////////////////////////////////////
// sleep handling
//////////////////////////////////////////////////////////////////////////////////////////////////////

// start timer2 for wake up
// start sleep mode

//////////////////////////////////////////////////////////////////////////////////////////////////////
// send CI-5 NG command
//////////////////////////////////////////////////////////////////////////////////////////////////////
 
uint8 ci5_ng (void) {
         ci5_data[0] = 0xFE; 
        ci5_data[1] = 0xFE;
        ci5_data[2] = 0xE0;
        ci5_data[3] = RADIO_ADDRESS;
        ci5_data[4] = 0xFA; // "NG"
        ci5_data[5] = 0xFD;
        return (ci5_tx(6));
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
// send CI-5 OK command
//////////////////////////////////////////////////////////////////////////////////////////////////////

uint8 ci5_ok (void) {
         ci5_data[0] = 0xFE; 
        ci5_data[1] = 0xFE;
        ci5_data[2] = 0xE0;
        ci5_data[3] = RADIO_ADDRESS;
        ci5_data[4] = 0xFB; // "OK"
        return (ci5_tx(6));
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
// receive CI-5 commands via RS232
//////////////////////////////////////////////////////////////////////////////////////////////////////

uint8 ci5_rx (uint8 index) {
        
        uint8 timeout;

        TIMEOUT_RESET;

        while (BIT_CLEAR(UCSR0A, RXC0)) {
                // wait on CI-5 Rx activity 
                if (!timeout) {
                        return (CI5_RX_ERROR);
                }
                TIMEOUT_TICK; 
        }
        TIMEOUT_RESET;
        LED_CI5_ON;
        ci5_data [0] = UDR0; // first received byte
                
        // error handling
        if (ci5_data[0] != 0xFE) {
                // no valid CI-5 command
                LED_CI5_OFF;
                return (CI5_RX_ERROR);
        }
        // receive all bytes from CI-5 interface
        for (index = 0; index < BUFFER_SIZE; index++) {
                while (BIT_CLEAR(UCSR0A, RXC0)) {
                        // wait on CI-5 Rx activity        
                        if (!timeout) {
                                return (CI5_RX_ERROR);
                        }
                        TIMEOUT_TICK;  
                }
                TIMEOUT_RESET;                
                ci5_data [index] = UDR0;
                if (ci5_data[index] == 0xFD) {
                        // end of command?
                        break;
                }                
        }

        LED_CI5_OFF;

        // error handling
        if ((ci5_data[1] != 0xFE) || (ci5_data[2] != 0x5E) || (ci5_data[num] != 0xFD)) {
                // no valid CI-5 command        
                return (CI5_RX_ERROR);
        }
        return (CI5_RX_END);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
// send CI-5 commands via RS232
//////////////////////////////////////////////////////////////////////////////////////////////////////

uint8 ci5_tx (uint8 num) {
   
        uint8 index;
        // uint8 timeout;

        // TIMEOUT_RESET;
        LED_CI5_ON;

        for (index = 0; index < num; index++) {
                while (BIT_CLEAR(UCSR0A, UDRE0)) {
                        // wait to send out all bytes of tx buffer                 
                        // if (!timeout) {
                        //                // ??? tx error status flags?!
                        //         return (CI5_TX_ERROR);
                        // }
                        // TIMEOUT_TICK;
                }
                // TIMEOUT_RESET;
                UDR0 = ci5_data [index];
        }

        LED_CI5_OFF;
        return (CI5_TX_END);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
// send CI-4 commands via PCx and PBx lines
//////////////////////////////////////////////////////////////////////////////////////////////////////

uint8 ci4_tx(uint8 num) {

        uint8 index;
        uint8 timeout;

        //data port configured as input
        
        SQR_LEDCI4_ACTIVE;
        
        for (index=0; index < num; index++) {
                // prepare data ports
                PORTC &= 0xF0 + (ci4_data [index] & 0x0F); // DB0-DB3
                PORTB &= 0xF0 + (ci4_data [index] >> 4); // DB4-DB7

                TIMEOUT_RESET;
                WP_ACTIVE;

                while (BIT_SET(PIND, READ_DATA)) {
                                // wait on signal \I = "0"
                                if (!timeout) {
                                        WP_DEACTIVE;
                                        SQR_LEDCI4_DEACTIVE;                                
                                        return (CI4_TX_ERROR);
                                }
                                TIMEOUT_TICK;
                }
                // put data on the bus
                SET_PORTCB_OUPUT; // set data ports as output

                while (BIT_CLEAR(PIND, READ_DATA)) {
                                // wait on signal \I = "1"
                                if (!timeout) {
                                        WP_DEACTIVE;
                                        SQR_LEDCI4_DEACTIVE;                                
                                        return (CI4_TX_ERROR);
                                }
                                TIMEOUT_TICK;
                }
                
                SET_PORTCB_INPUT; // set data ports back to input

                while(BIT_SET(PIND, DAV)) {
                        // wait on \DAV = "0"
                        if (!timeout) {
                                WP_DEACTIVE;
                                SQR_LEDCI4_DEACTIVE;
                                return(CI4_TX_ERROR);
                        }
                        TIMEOUT_TICK;
                }

                TIMEOUT_RESET;
                WP_DEACTIVE;

                while (BIT_CLEAR(PIND, DAV)) {
                        // wait on \DAV = "1"
                        if (!timeout) {
                                SQR_LEDCI4_DEACTIVE;
                                return (CI4_TX_ERROR);
                        }
                        TIMEOUT_TICK;
                }
        }
        SQR_LEDCI4_DEACTIVE;
        return (CI4_TX_END);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
// receive CI-4 commands via PCx and PBx lines
//////////////////////////////////////////////////////////////////////////////////////////////////////

uint8 ci4_rx (uint8 num) {
        
        uint8 index;
        uint8 timeout;

        //data port configured as input

        SQR_LEDCI4_ACTIVE;

        for (index = 0; index < num; index++) {
                TIMEOUT_RESET;
                RP_ACTIVE;
                
                while (BIT_SET(PIND, WRITE_DATA)) {
                                // wait on signal \C = "0"
                                if (!timeout) {
                                        RP_DEACTIVE;
                                        SQR_LEDCI4_DEACTIVE;                                
                                        return (CI4_RX_ERROR);
                                }
                                TIMEOUT_TICK;
                }
                // read valid data bus
                ci4_data [index] = (PINB << 4) + (PINC & 0x0F); // read CI-4 data
                TIMEOUT_RESET;

                while (BIT_SET(PIND, DAV)) {
                        // wait on \DAV = "0"
                        if (!timeout) {
                                RP_DEACTIVE;
                                SQR_LEDCI4_DEACTIVE;
                                return (CI4_RX_ERROR);
                        }
                        TIMEOUT_TICK;
                }

                RP_DEACTIVE;
                TIMEOUT_RESET;

                while (BIT_CLEAR(PIND, DAV)) {
                        // wait on \DAV = "1"
                        if (!timeout) {
                                SQR_LEDCI4_DEACTIVE;
                                return (CI4_RX_ERROR);
                        }
                        TIMEOUT_TICK;
                }        
        }
        SQR_LEDCI4_DEACTIVE;
        return (CI4_RX_END);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
// converts CI-5 to CI-4 commands
//////////////////////////////////////////////////////////////////////////////////////////////////////

uint8 ci5_to_ci4 (void) {

        uint8 num;
        
        switch (ci5_data[4]) {
                case (0x00): // set operating frequency
                        ci4_data[0] = 0x21;
                        ci4_data[1] = 0x2D;
                        ci4_data[2] = 0x20; // 1GHz
                        ci4_data[3] = 0x20; // 100MHz
                        ci4_data[4] = (ci5_data[8] >> 4) + 0x20; // 10MHz
                        ci4_data[5] = (ci5_data[8] & 0x0F) + 0x20; // 1MHz
                        ci4_data[6] = (ci5_data[7] >> 4) + 0x20; // 100kHz
                        ci4_data[7] = (ci5_data[7] & 0x0F) + 0x20; // 10kHz
                        ci4_data[8] = (ci5_data[6] >> 4) + 0x20; // 1kHz
                        ci4_data[9] = (ci5_data[6] & 0x0F) + 0x20; // 100Hz
                        ci4_data[10] = (ci5_data[5] >> 4) + 0x20; // 10Hz
                        ci4_data[11] = (ci5_data[5] & 0x0F) + 0x20; // 1Hz
                        ci4_data[12] = 0x2E;
                        num = 13;
                        break;
                case(0x01): // set operating mode
                        ci4_data[0] = 0x31;
                        ci4_data[1] = 0x3D;

                          switch (ci5_data[5]) {
                                case (0x00):
                                        ci4_data[2] = 0x30; // LSB
                                        break;
                                case (0x01):
                                        ci4_data[2] = 0x31; // USB
                                        break;
                                case (0x02):
                                        ci4_data[2] = 0x32; // AM
                                         break;
                                case (0x03):
                                        ci4_data[2] = 0x33; // CW
                                         break;
                                case (0x04):
                                         ci4_data[2] = 0x34; // RTTY
                                         break;
                                case (0x05):
                                        ci4_data[2] = 0x35; // FM
                                        break;
                        }

                        ci4_data[3] = 0x3E;
                        num = 4;
                        break;
                case (0x02): // read lower/upper edge frequency
                        ci4_data[0] = 0x11;
                        num = 1;
                        break;
                case (0x03): // read operating frequency
                        ci4_data[0] = 0x21;
                        num = 1;
                        break;  
                case (0x04): // read operating mode and IF passband width
                        ci4_data[0] = 0x31;
                        num=1;
                        break;
                case (0x05): // write operating FREQUENCY into VFO or memory channel
                        ci4_data[0] = 0x21;
                        ci4_data[1] = 0x2D;
                        ci4_data[2] = 0x20; // 1GHz
                        ci4_data[3] = 0x20; // 100 MHz
                        ci4_data[4] = (ci5_data[8] >> 4) + 0x20; // 10 MHz
                        ci4_data[5] = (ci5_data[8] & 0x0F) + 0x20; // 1 MHz
                        ci4_data[6] = (ci5_data[7] >> 4) + 0x20; // 100 kHz
                        ci4_data[7] = (ci5_data[7] & 0x0F) + 0x20; // 10 kHz
                        ci4_data[8] = (ci5_data[6] >> 4) + 0x20; // 1 kHz
                        ci4_data[9] = (ci5_data[6] & 0x0F) + 0x20; // 100 Hz
                        ci4_data[10] = (ci5_data[5] >> 4) + 0x20; // 10 Hz
                        ci4_data[11] = (ci5_data[5] & 0x0F) + 0x20; // 1 Hz
                        ci4_data[12] = 0x2E;
                        num = 13;
                        break;  
                case (0x06): // writes operating MODE data into VFO or memory channel
                        ci4_data[0] = 0x31;
                        ci4_data[1] = 0x3D;

                        switch (ci5_data[5]) {
                                case (0x00):
                                        ci4_data[2] = 0x30;
                                        break;
                                case (0x01):
                                        ci4_data[2] = 0x31;
                                        break;
                                case (0x02):
                                        ci4_data[2] = 0x32;
                                        break;
                                case (0x03):
                                        ci4_data[2] = 0x33;
                                        break;
                                case (0x04):
                                        ci4_data[2] = 0x34;
                                        break;
                                case (0x05):
                                        ci4_data[2] = 0x35;
                                        break;
                        }

                        ci4_data[3] = 0x3E;
                        num = 4;
                        break;
                case (0x07): // selects VFO mode, VFO-A or VFO-B
                        switch (ci5_data[5]) {
                        case (0xFD):
                                ci4_data[0] = 0x51;
                                ci4_data[1] = 0x5D;
                                ci4_data[2] = 0x50;
                                ci4_data[3] = 0x50;
                                ci4_data[4] = 0x5E;
                                num = 5;  
                                break;
                        /* // for future extension
                        case (0x00):
                                ci4_data[0] = 0x51;
                                ci4_data[1] = 0x5D;
                                ci4_data[2] = 0x50;
                                ci4_data[3] = 0x50;
                                ci4_data[4] = 0x5E;
                                num = 5;  
                                break;
                        
                        case (0x01):
                                ci4_data[0] = 0x51;
                                ci4_data[1] = 0x5D;
                                ci4_data[2] = 0x50;
                                ci4_data[3] = 0x50;
                                ci4_data[4] = 0x5E;
                                num = 5;  
                                break;
                                
                        case (0xA0):
                                num = 0;        
                                break;
                                
                        case (0xB0):
                                num = 0;        
                                break;
                        */
                        }
                        break;
                case (0x08): // MR, select memory channel
                        ci4_data[0] = 0x51;
                        ci4_data[1] = 0x5D;

                        switch (ci5_data[5]) {
                                case (0xFD): ci4_data[2] = 0x50; ci4_data[3] = 0x51; break;
                                case (0x00): ci4_data[2] = 0x50; ci4_data[3] = 0x50; break;
                                case (0x01): ci4_data[2] = 0x50; ci4_data[3] = 0x51; break;
                                case (0x02): ci4_data[2] = 0x50; ci4_data[3] = 0x52; break;
                                case (0x03): ci4_data[2] = 0x50; ci4_data[3] = 0x53; break;
                                case (0x04): ci4_data[2] = 0x50; ci4_data[3] = 0x54; break;
                                case (0x05): ci4_data[2] = 0x50; ci4_data[3] = 0x55; break;
                                case (0x06): ci4_data[2] = 0x50; ci4_data[3] = 0x56; break;
                                case (0x07): ci4_data[2] = 0x50; ci4_data[3] = 0x57; break;
                                case (0x08): ci4_data[2] = 0x50; ci4_data[3] = 0x58; break;
                                case (0x09): ci4_data[2] = 0x50; ci4_data[3] = 0x59; break;
                                case (0x10): ci4_data[2] = 0x50; ci4_data[3] = 0x5A; break;
                                case (0x11): ci4_data[2] = 0x50; ci4_data[3] = 0x5B; break;
                                case (0x12): ci4_data[2] = 0x50; ci4_data[3] = 0x5C; break;
                                case (0x13): ci4_data[2] = 0x50; ci4_data[3] = 0x5D; break;
                                case (0x14): ci4_data[2] = 0x50; ci4_data[3] = 0x5E; break;
                                case (0x15): ci4_data[2] = 0x50; ci4_data[3] = 0x5F; break;
                                case (0x16): ci4_data[2] = 0x51; ci4_data[3] = 0x50; break;
                                case (0x17): ci4_data[2] = 0x51; ci4_data[3] = 0x51; break;
                                case (0x18): ci4_data[2] = 0x51; ci4_data[3] = 0x52; break;
                                case (0x19): ci4_data[2] = 0x51; ci4_data[3] = 0x53; break;
                                case (0x20): ci4_data[2] = 0x51; ci4_data[3] = 0x54; break;
                                case (0x21): ci4_data[2] = 0x51; ci4_data[3] = 0x55; break;
                                case (0x22): ci4_data[2] = 0x51; ci4_data[3] = 0x56; break;
                                case (0x23): ci4_data[2] = 0x51; ci4_data[3] = 0x57; break;
                                case (0x24): ci4_data[2] = 0x51; ci4_data[3] = 0x58; break;
                                case (0x25): ci4_data[2] = 0x51; ci4_data[3] = 0x59; break;
                                case (0x26): ci4_data[2] = 0x51; ci4_data[3] = 0x5A; break;
                                case (0x27): ci4_data[2] = 0x51; ci4_data[3] = 0x5B; break;
                                case (0x28): ci4_data[2] = 0x51; ci4_data[3] = 0x5C; break;
                                case (0x29): ci4_data[2] = 0x51; ci4_data[3] = 0x5D; break;
                                case (0x30): ci4_data[2] = 0x51; ci4_data[3] = 0x5E; break;
                                case (0x31): ci4_data[2] = 0x51; ci4_data[3] = 0x5F; break;
                                case (0x32): ci4_data[2] = 0x52; ci4_data[3] = 0x50; break;
                        }
                
                        ci4_data[4] = 0x5E;
                        num = 5;
                        break;
                case (0x09): // M WRITE, write displayed content into selected memory channel
                        ci4_data[0] = 0x61;
                        ci4_data[1] = 0x6D;
                        ci4_data[2] = 0x61;
                        ci4_data[3] = 0x6E;
                        num = 4;
                        break;          
                case (0x0A): // M-> VFO
                        ci4_data[0] = 0x61;
                        ci4_data[1] = 0x6D;
                        ci4_data[2] = 0x62;
                        ci4_data[3] = 0x6E;
                        num = 4;
                        break;
                /* // for future extension
                        case (0x0B): // M CLEAR
                        ci4_data[0] = 0x61;
                        ci4_data[1] = 0x6D;
                        ci4_data[2] = 0x62;
                        ci4_data[3] = 0x6E;
                        num = 4;
                        break;
                        case (0x0C): // read offset frequency
                        ci4_data[0] = 0x61;
                        ci4_data[1] = 0x6D;
                        ci4_data[2] = 0x62;
                        ci4_data[3] = 0x6E;
                        num = 4;
                        break;
                        case (0x0D): // write offset frequency
                        ci4_data[0] = 0x61;
                        ci4_data[1] = 0x6D;
                        ci4_data[2] = 0x62;
                        ci4_data[3] = 0x6E;
                        num = 4;
                        break;
                case (0x0E): // start and stop scan, no CI-4 support
                        // to be realized via remote key matrix control
                        break;
                        case (0x0F): // SPLIT, DUP, select split simplex, +- duplex
                        ci4_data[0] = 0x61;
                        ci4_data[1] = 0x6D;
                        ci4_data[2] = 0x62;
                        ci4_data[3] = 0x6E;
                        num = 4;
                        break;
                        case (0x10): // TS select tuning step
                        ci4_data[0] = 0x61;
                        ci4_data[1] = 0x6D;
                        ci4_data[2] = 0x62;
                        ci4_data[3] = 0x6E;
                        num = 4;
                        break;
                        case (0x11): // ATT, select attenuator level
                        ci4_data[0] = 0x61;
                        ci4_data[1] = 0x6D;
                        ci4_data[2] = 0x62;
                        ci4_data[3] = 0x6E;
                        num = 4;
                        break;
                        case (0x15): // read squelch and signal strength
                        ci4_data[0] = 0x61;
                        ci4_data[1] = 0x6D;
                        ci4_data[2] = 0x62;
                        ci4_data[3] = 0x6E;
                        num = 4;
                        break;
                */
                case (0x19): // ID
                        ci5_data[0] = 0xFE;
                        ci5_data[1] = 0xFE;
                        ci5_data[2] = 0xE0;
                        ci5_data[3] = RADIO_ADDRESS;
                        ci5_data[4] = 0x19;
                        ci5_data[5] = 0x00;
                        ci5_data[6] = 0x5E;
                        ci5_data[7] = 0xFD;
                        num = 8;
                        break;                
                default:
                        num = CONVERT_ERROR; // error handling, something invalid
        }
        return (num);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
// convert CI-4 to CI-5 commands
//////////////////////////////////////////////////////////////////////////////////////////////////////

uint8 ci4_to_ci5 (void) {

        uint8 num;

        // ??? could be deleted
        ci5_data[0] = 0xFE; // command preamble
        ci5_data[1] = 0xFE;
        ci5_data[2] = 0xE0; // controller address
        ci5_data[3] = RADIO_ADDRESS; // transceiver address
        //ci5_data[4] = 0x00; // ??? still valid

        switch (ci4_data[4]) {
                case (0x02): // ???
                        ci5_data[5] = 0x00;
                        ci5_data[6] = 0x00;
                        ci5_data[7] = (ci4_data[5] << 4) + (ci4_data[6] & 0x0F);
                        ci5_data[8] = (ci4_data[3] << 4) + (ci4_data[4] & 0x0F);
                        ci5_data[9] = 0x00;
                        ci5_data[10] = 0x2D;
                        ci5_data[11] = 0x00;
                        ci5_data[12] = 0x00;
                        ci5_data[13] = (ci4_data[13] << 4) + (ci4_data[14] & 0x0F);
                        ci5_data[14] = (ci4_data[11] << 4) + (ci4_data[12] & 0x0F);
                        ci5_data[15] = 0x00;
                        ci5_data[16] = 0xFD;
                        num = 17;
                        break;
  
                case (0x03): // ???
                        ci5_data[5] = (ci4_data[9] << 4);
                        ci5_data[6] = (ci4_data[7] << 4) + (ci4_data[8] & 0x0F);
                        ci5_data[7] = (ci4_data[5] << 4) + (ci4_data[6] & 0x0F);
                        ci5_data[8] = (ci4_data[3] << 4) + (ci4_data[4] & 0x0F);
                        ci5_data[9] = 0x00;
                        ci5_data[10] = 0xFD;
                        num = 11;
                        break;  
  
                case (0x04): // ???
                        switch (ci4_data[1]) {
                                case (0x36):
                                        ci5_data[5] = 0x03;
                                        break;
                                case (0x37):
                                        ci5_data[5] = 0x04;
                                        break;
                                case (0x38):
                                         ci5_data[5] = 0x00;
                                         break;
                                case (0x39):
                                        ci5_data[5] = 0x01;
                                        break;
                                case (0x3A):
                                        ci5_data[5] = 0x02;
                                        break;
                                case (0x3B):
                                        ci5_data[5] = 0x03;
                                        break;
                                case (0x3C):
                                        ci5_data[5] = 0x04;
                                        break;
                                case (0x3D):
                                        ci5_data[5] = 0x05;
                                        break;
                                default:
                                        ci5_data[5] = ci4_data[1] & 0x0F;
                                        break;
                        }
                        ci5_data[6] = 0xFD;
                        num = 7;
                        break;

                default:
                        num = CONVERT_ERROR; // error handling, something invalid
        }
        return (num);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
// TWI/I2C bus to interface matrix- and RAM-controller
//////////////////////////////////////////////////////////////////////////////////////////////////////
/*
The TWI can operate in one of four major modes. These are named Master Transmitter (MT), Master Receiver (MR),
Slave Transmitter (ST) and Slave Receiver (SR).

S:                START condition
Rs:        REPEATED START condition
R:                Read bit (high level at SDA)
W:                Write bit (low level at SDA)
A:                Acknowledge bit (low level at SDA)
A:                Not acknowledge bit (high level at SDA)
Data: 8-bit data byte
P:                STOP condition
SLA:        Slave Address

// 1. Send START condition.
        TWCR = (1<<TWINT) | (1<<TWSTA)|(1<<TWEN);

// 2 Wait for TWINT Flag set. This indicates that the START condition has been transmitted.
        while (!(TWCR & (1<<TWINT)));

// 3. Check value of TWI Status Register. Mask prescaler bits. If status different from START go to ERROR.
        if ((TWSR & 0xF8) != START) ERROR();
// Load SLA_W into TWDR Register. Clear TWINT bit in TWCR to start transmission of address.
        TWDR = SLA_W;
        TWCR = (1<<TWINT) | (1<<TWEN);

// 4. wait for TWINT Flag set. This indicates that the SLA+W has been transmitted, and ACK/NACK has been received.
        while (!(TWCR & (1<<TWINT)));

// 5. Check value of TWI Status Register. Mask prescaler bits. If status different from MT_SLA_ACK go to ERROR.
        if ((TWSR & 0xF8) != MT_SLA_ACK) ERROR();
// Load DATA into TWDR Register. Clear TWINT bit in TWCR to start transmission of data.
        TWDR = DATA;
        TWCR = (1<<TWINT) | (1<<TWEN);

// 6. Wait for TWINT Flag set. This indicates that the DATA has been transmitted, and ACK/NACK has been received.
        while (!(TWCR & (1<<TWINT)));

// 7. Check value of TWI Status Register. Mask prescaler bits. If status different from MT_DATA_ACK go to ERROR.
        if ((TWSR & 0xF8) != MT_DATA_ACK) ERROR();
// Transmit STOP condition
        TWCR = (1<<TWINT) | (1<<TWEN)|(1<<TWSTO);
*/

//////////////////////////////////////////////////////////////////////////////////////////////////////
// ADC handling to support signal strength command
//////////////////////////////////////////////////////////////////////////////////////////////////////
