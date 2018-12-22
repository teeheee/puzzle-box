/* Name: main.c
 * Project: hid-mouse, a very simple HID example
 * Author: Christian Starkjohann
 * Creation Date: 2008-04-07
 * Tabsize: 4
 * Copyright: (c) 2008 by OBJECTIVE DEVELOPMENT Software GmbH
 * License: GNU GPL v2 (see License.txt), GNU GPL v3 or proprietary (CommercialLicense.txt)
 */

/*
This example should run on most AVRs with only little changes. No special
hardware resources except INT0 are used. You may have to change usbconfig.h for
different I/O pins for USB. Please note that USB D+ must be the INT0 pin, or
at least be connected to INT0 as well.

We use VID/PID 0x046D/0xC00E which is taken from a Logitech mouse. Don't
publish any hardware using these IDs! This is for demonstration only!
*/
#define F_CPU 12000000UL

#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>  /* for sei() */
#include <util/delay.h>     /* for _delay_ms() */

#include <avr/pgmspace.h>   /* required by usbdrv.h */
#include "usbdrv.h"
#include "oddebug.h"        /* This is also an example for using debug macros */
#include "i2c_master.h"
#include <stdlib.h>



/* ------------------------------------------------------------------------- */
/* ----------------------------- USB interface ----------------------------- */
/* ------------------------------------------------------------------------- */

PROGMEM const char usbHidReportDescriptor[52] = { /* USB report descriptor, size must match usbconfig.h */
    0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)
    0x09, 0x02,                    // USAGE (Mouse)
    0xa1, 0x01,                    // COLLECTION (Application)
    0x09, 0x01,                    //   USAGE (Pointer)
    0xA1, 0x00,                    //   COLLECTION (Physical)
    0x05, 0x09,                    //     USAGE_PAGE (Button)
    0x19, 0x01,                    //     USAGE_MINIMUM
    0x29, 0x03,                    //     USAGE_MAXIMUM
    0x15, 0x00,                    //     LOGICAL_MINIMUM (0)
    0x25, 0x01,                    //     LOGICAL_MAXIMUM (1)
    0x95, 0x03,                    //     REPORT_COUNT (3)
    0x75, 0x01,                    //     REPORT_SIZE (1)
    0x81, 0x02,                    //     INPUT (Data,Var,Abs)
    0x95, 0x01,                    //     REPORT_COUNT (1)
    0x75, 0x05,                    //     REPORT_SIZE (5)
    0x81, 0x03,                    //     INPUT (Const,Var,Abs)
    0x05, 0x01,                    //     USAGE_PAGE (Generic Desktop)
    0x09, 0x30,                    //     USAGE (X)
    0x09, 0x31,                    //     USAGE (Y)
    0x09, 0x38,                    //     USAGE (Wheel)
    0x15, 0x81,                    //     LOGICAL_MINIMUM (-127)
    0x25, 0x7F,                    //     LOGICAL_MAXIMUM (127)
    0x75, 0x08,                    //     REPORT_SIZE (8)
    0x95, 0x03,                    //     REPORT_COUNT (3)
    0x81, 0x06,                    //     INPUT (Data,Var,Rel)
    0xC0,                          //   END_COLLECTION
    0xC0,                          // END COLLECTION
};
/* This is the same report descriptor as seen in a Logitech mouse. The data
 * described by this descriptor consists of 4 bytes:
 *      .  .  .  .  . B2 B1 B0 .... one byte with mouse button states
 *     X7 X6 X5 X4 X3 X2 X1 X0 .... 8 bit signed relative coordinate x
 *     Y7 Y6 Y5 Y4 Y3 Y2 Y1 Y0 .... 8 bit signed relative coordinate y
 *     W7 W6 W5 W4 W3 W2 W1 W0 .... 8 bit signed relative coordinate wheel
 */
typedef struct{
    uchar   buttonMask;
    char    dx;
    char    dy;
    char    dWheel;
}report_t;

static report_t reportBuffer;
static uchar    idleRate;   /* repeat rate for keyboards, never used for mice */


/* The following function advances sin/cos by a fixed angle
 * and stores the difference to the previous coordinates in the report
 * descriptor.
 * The algorithm is the simulation of a second order differential equation.
 */


void init_uart(uint16_t baudrate){

	uint16_t UBRR_val = (F_CPU/16)/(baudrate-1);

	UBRRH = UBRR_val >> 8;
	UBRRL = UBRR_val;

	UCSRB |= (1<<TXEN) | (1<<RXEN); // UART TX (Transmit - senden) einschalten
	UCSRC |= (1<<URSEL)|(1<<USBS) | (3<<UCSZ0); //Modus Asynchron 8N1 (8 Datenbits, No Parity, 1 Stopbit)
}

void uart_putc(unsigned char c){

	while(!(UCSRA & (1<<UDRE))); // wait until sending is possible
	UDR = c; // output character saved in c
}

void uart_puts(char *s){
	while(*s){
		uart_putc(*s);
		s++;
	}
}



#define ACCEL 0b11010110

int acc_x,acc_y,acc_z;
int num;

void initsensor()
{
	num = 0;
	i2c_init();
	if(i2c_write_Reg_Byte(ACCEL,0x10,0x80))
		return;
	if(i2c_write_Reg_Byte(ACCEL,0x11,0x80))
		return;
	if(i2c_write_Reg_Byte(ACCEL,0x12,0x04))
		return;
}

int abs(int a)
{
	return a>0?a:-a;
}

void readsensor()
{
	int16_t buffer[3];
	i2c_readReg(ACCEL,0x28,buffer,6);
	acc_x=buffer[0];
	acc_y=buffer[1];
	acc_z=buffer[2];
}

void close()
{
	DDRD |= (1<<PD7);
	for(int x = 0; x < 10; x++)
	{
		
		PORTD |= (1<<PD7);
		_delay_us(3000);
		PORTD &= ~(1<<PD7);
		_delay_ms(50);
	}
}

void open()
{
	DDRD |= (1<<PD7);
	for(int x = 0; x < 10; x++)
	{
		
		PORTD |= (1<<PD7);
		_delay_us(1500);
		PORTD &= ~(1<<PD7);
		_delay_ms(50);
	}
}

#define threshold 20000

#define passlength 5
//const int pass[passlength] = {1,7,1,0,1,3,1,9,2,3,2}; 
const int pass[passlength] = {1,2,1,2,1}; 



/*
1710 1319 23 2

1 3
2 0
3 2
4 1
5 7
6 9



  1
3 5 4
  2
  6

*/

int passcount = 0;
void processsensor()
{
	char textbuffer[10];
	uart_puts("\033[2J\033[1;1H");
	uart_puts("num: ");
	uart_puts(itoa(num,textbuffer,10));
	uart_puts("\r\npassc: ");
	uart_puts(itoa(passcount,textbuffer,10));


	int x = abs(acc_x);
	int y = abs(acc_y);
	int z = abs(acc_z);
	if(x>y)
	{
		if(x-y>threshold)
			num = 1;
		else
			num = 0;
		
	}
	else if(y>x)
	{
		if(y-x>threshold)
			num = 2;
		else
			num = 0;
	}
	if(passcount==passlength)
	{
		open();
		passcount=0;
		//return;
	}
	if(pass[passcount]==num)
	{
		passcount++;
		_delay_ms(500);
		return;	
	}
	if(num == 0)
		return;
	close();
	passcount = 0;
}


#define lengthhappy 8
char happy[] = {2,0, -1,0, 0,1, -1,0, 2,0, -1,0, 0,-1, -1,0};
		    



		    //length //radius //direction //offset //type 0 circle
		    //x 	//y			   //type 1 line
		    //x         //y			   //type 2 dot
int gallifreyan[] = {360,20,1,5,0,
		     390,5,-1,15,0,
		     85,18,-1,5,0,
		     360,5,1,270,0,
		     85,18,-1,90,0,
		     360,5,1,180,0,
		     80,18,-1,180,0,
		     320,5,1,100,0,
		     140,5,-1,-55,0,
		     -20,-100,0,0,1,
		     5,0,0,0,1,
		     52,35,0,0,1,
		     0,5,0,0,1,
		     -40,50,0,0,1,
		     145,5,1,275,0,
		     80,18,-1,275,0,
		     0,10,0,0,1,
		     32,10,0,0,1,
		     0,0,0,0,2,
		     0,12,0,0,1,
		     0,0,0,0,2};

#define gallisize 21


static void advanceCircleByFixedAngle(void)
{
	


static int delay = 0;
if(delay<100)
{
	delay++;
	return;
}
static double c = 0;
static int i = 0;
if(c<=gallifreyan[0+i] && gallifreyan[4+i]==0)
{
	reportBuffer.dx = sin((c+gallifreyan[3+i])*3.14d/180)*gallifreyan[1+i];
	reportBuffer.dy = gallifreyan[2+i]*cos((c+gallifreyan[3+i])*3.14d/180)*gallifreyan[1+i];
	c+=10;
}
else if(gallifreyan[4+i]==1)
{
	reportBuffer.dx = gallifreyan[0+i];
	reportBuffer.dy = gallifreyan[1+i];
	gallifreyan[4+i]=-1;
}
else if(gallifreyan[4+i]==2 && c<4)
{
int e = c;
	switch(e)
	{
		case 0:
	reportBuffer.dx = 2;
	reportBuffer.dy = 0;
		break; case 1:
	reportBuffer.dx = 0;
	reportBuffer.dy = 2;
		break; case 2:
	reportBuffer.dx = -2;
	reportBuffer.dy = 0;
		break; case 3:
	reportBuffer.dx = 0;
	reportBuffer.dy = -2;
		break;
	}
c++;
}
else
{
	if(i/5<gallisize-1)
	{
		c=0;
		i+=5;
	}
	reportBuffer.dy = 0;	
	reportBuffer.dx = 0;
}


//char    d;
/*	static int c = 0;
	reportBuffer.dx = happy[c*2]*5;
	reportBuffer.dy = happy[c*2+1]*5;
	c=(c+1)%lengthhappy;
*/
/*#define DIVIDE_BY_64(val)  (val + (val > 0 ? 32 : -32)) >> 6    
    reportBuffer.dx = d = DIVIDE_BY_64(cosinus);
    sinus += d;
    reportBuffer.dy = d = DIVIDE_BY_64(sinus);
    cosinus -= d;*/
}

/* ------------------------------------------------------------------------- */

usbMsgLen_t usbFunctionSetup(uchar data[8])
{
usbRequest_t    *rq = (void *)data;

    /* The following requests are never used. But since they are required by
     * the specification, we implement them in this example.
     */
    if((rq->bmRequestType & USBRQ_TYPE_MASK) == USBRQ_TYPE_CLASS){    /* class request type */
        DBG1(0x50, &rq->bRequest, 1);   /* debug output: print our request */
        if(rq->bRequest == USBRQ_HID_GET_REPORT){  /* wValue: ReportType (highbyte), ReportID (lowbyte) */
            /* we only have one report type, so don't look at wValue */
            usbMsgPtr = (void *)&reportBuffer;
            return sizeof(reportBuffer);
        }else if(rq->bRequest == USBRQ_HID_GET_IDLE){
            usbMsgPtr = &idleRate;
            return 1;
        }else if(rq->bRequest == USBRQ_HID_SET_IDLE){
            idleRate = rq->wValue.bytes[1];
        }
    }else{
        /* no vendor specific requests implemented */
    }
    return 0;   /* default for not implemented requests: return no data back to host */
}

/* ------------------------------------------------------------------------- */

int __attribute__((noreturn)) main(void)
{
 	close();
	uchar   i;
	initsensor();
	init_uart(9600);

    wdt_enable(WDTO_1S);
    /* If you don't use the watchdog, replace the call above with a wdt_disable().
     * On newer devices, the status of the watchdog (on/off, period) is PRESERVED
     * OVER RESET!
     */
    /* RESET status: all port bits are inputs without pull-up.
     * That's the way we need D+ and D-. Therefore we don't need any
     * additional hardware initialization.
     */
    odDebugInit();
    DBG1(0x00, 0, 0);       /* debug output: main starts */
    usbInit();
    usbDeviceDisconnect();  /* enforce re-enumeration, do this while interrupts are disabled! */
    i = 0;
    while(--i){             /* fake USB disconnect for > 250 ms */
        wdt_reset();
        _delay_ms(1);
    }
    usbDeviceConnect();
    sei();
    DBG1(0x01, 0, 0);       /* debug output: main loop starts */
    for(;;){                /* main event loop */
        DBG1(0x02, 0, 0);   /* debug output: main loop iterates */
	readsensor();
	processsensor();
        wdt_reset();
        usbPoll();
        if(usbInterruptIsReady()){
            /* called after every poll of the interrupt endpoint */
            //advanceCircleByFixedAngle();
            if(num == 1)
            {
	            reportBuffer.dx = 10*passcount;
	            reportBuffer.dy = 0;
            }
            if(num == 2)
            {
	            reportBuffer.dx = 0;
	            reportBuffer.dy = 10*passcount;
            }
            if(num == 0)
            {
	            reportBuffer.dx = 0;
	            reportBuffer.dy = 0;
            }
            DBG1(0x03, 0, 0);   /* debug output: interrupt report prepared */
            usbSetInterrupt((void *)&reportBuffer, sizeof(reportBuffer));
        }
    }
}



/* ------------------------------------------------------------------------- */
