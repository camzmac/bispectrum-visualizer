
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
//#include "driverlib/gpio.h"
#include "inc/hw_ints.h"

//Print messages on UART0 for debugging purposes
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "utils/uartstdio.c"

//Permit calls to vendor supplied functions in ROM
#define TARGET_IS_TM4C123_RB2
#include "driverlib/rom.h"

// RGB LED driver
#include "drivers/rgb.h"
#include "drivers/rgb.c"

//ADC
#include "driverlib/adc.h"
#include "inc/hw_adc.h"
//Timer driver
#include "driverlib/timer.h"
//uDMA
#include "driverlib/udma.h"
//FPU
#include "driverlib/fpu.h"



//Constant declarations

//44.1 kHz
#define SAMPLING_PERIOD 0.01

//Window size is the number of samples per window
// and must be: a power of 2 and >=16
#define WINDOW_SIZE 1024


//Define states for the half_window module
#define HALFWINDOW_READYFORREAD_WITHA    0
#define HALFWINDOW_LOADING_FROMA         2
#define HALFWINDOW_DONELOADING_FROMA     3
#define HALFWINDOW_READYFORREAD_WITHB    4
#define HALFWINDOW_LOADING_FROMB         6
#define HALFWINDOW_DONELOADING_FROMB     7



//*****************************************************************************
//
// The control table used by the uDMA controller.  This table must be aligned
// to a 1024 byte boundary.
//
//*****************************************************************************
#if defined(ewarm)
#pragma data_alignment=1024
uint8_t ui8ControlTable[1024];
#elif defined(ccs)
#pragma DATA_ALIGN(ui8ControlTable, 1024)
uint8_t ui8ControlTable[1024];
#else
uint8_t ui8ControlTable[1024] __attribute__ ((aligned(1024)));
#endif


//*****************************************************************************
// The error routine that is called if the driver library encounters an error.
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif

// uDMA ping-pong buffers for ADC
//WINDOW_SIZE is divided by two to facilitate 50% window overlap
static uint16_t ADC_BufA[WINDOW_SIZE/2];
static uint16_t ADC_BufB[WINDOW_SIZE/2];

//Lagging half of the signal segment to be windowed
static uint16_t half_window[WINDOW_SIZE/2];

volatile uint8_t SystemState;


//*****************************************************************************
//
// The interrupt handler for uDMA errors.  This interrupt will occur if the
// uDMA encounters a bus error while trying to perform a transfer.  This
// handler just increments a counter if an error occurs.
//
//*****************************************************************************
void
uDMAErrorHandler(void)
{
/*    uint32_t ui32Status;

    //
    // Check for uDMA error bit
    //
    ui32Status = ROM_uDMAErrorStatusGet();

    //
    // If there is a uDMA error, then clear the error and increment
    // the error counter.
    //
    if(ui32Status)
    {
        ROM_uDMAErrorStatusClear();
        g_ui32uDMAErrCount++;
    }*/
    UARTprintf("uDMA Error\n");
}

//Called when uDMA controller completes a software transfer
void uDMAIntHandler(void)
{
    //UARTprintf("uDMAIntHandler!\n");

    switch(SystemState){

    case HALFWINDOW_LOADING_FROMA:
        SystemState=HALFWINDOW_DONELOADING_FROMA;
        break;

    case HALFWINDOW_LOADING_FROMB:
        SystemState=HALFWINDOW_DONELOADING_FROMB;
        break;

    }

}


//Called when a uDMA transfer is complete.
//Purpose is to update module state and switch between the ping-ping buffers.
void ADCSeq3IntHandler(void)
{
    //Warning: Calling UARTprintf in this handler seems to cause some sampling artifacts at the end of buffer B

    uint32_t ui32Mode;
    ui32Mode = ROM_uDMAChannelModeGet(UDMA_CHANNEL_ADC3 | UDMA_PRI_SELECT);
    if(ui32Mode == UDMA_MODE_STOP)
    {
        //Data in buffer A is ready

        //Set up the next transfer for buffer A so that it is ready when B is full
        ROM_uDMAChannelTransferSet(UDMA_CHANNEL_ADC3 | UDMA_PRI_SELECT, UDMA_MODE_PINGPONG, (void *)(ADC0_BASE + ADC_O_SSFIFO3), ADC_BufA, WINDOW_SIZE/2);


        if(SystemState != HALFWINDOW_DONELOADING_FROMB)
        {
            //Data wasn't processed quickly enough, so "drop" this half-window's worth of data.
            //Can increment a counter or set a flag to indicate this condition if need be.
        }

        SystemState = HALFWINDOW_READYFORREAD_WITHA;

    }

    ui32Mode = ROM_uDMAChannelModeGet(UDMA_CHANNEL_ADC3 | UDMA_ALT_SELECT);
    if(ui32Mode == UDMA_MODE_STOP)
    {
        //Data in buffer B is ready

        //Set up the next transfer for buffer B so that it is ready when A is full
        ROM_uDMAChannelTransferSet(UDMA_CHANNEL_ADC3 | UDMA_ALT_SELECT, UDMA_MODE_PINGPONG, (void *)(ADC0_BASE + ADC_O_SSFIFO3), ADC_BufB, WINDOW_SIZE/2);

        if(SystemState != HALFWINDOW_DONELOADING_FROMA)
        {
            //Data wasn't processed quickly enough, so "drop" this half-window's worth of data.
            //Can increment a counter or set a flag to indicate this condition if need be.
        }

        SystemState = HALFWINDOW_READYFORREAD_WITHB;

    }

}


//*****************************************************************************
//
// Configure the UART and its pins.  This must be called before UARTprintf().
//
//*****************************************************************************
void
ConfigureUART(void)
{
    //
    // Enable the GPIO Peripheral used by the UART.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    //
    // Enable UART0
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    ROM_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_UART0);

    //
    // Configure GPIO Pins for UART mode.
    //
    ROM_GPIOPinConfigure(GPIO_PA0_U0RX);
    ROM_GPIOPinConfigure(GPIO_PA1_U0TX);
    ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Use the internal 16MHz oscillator as the UART clock source.
    //
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

    //
    // Initialize the UART for console I/O.
    //
    UARTStdioConfig(0, 115200, 16000000);
}

void print_buffer_contents(uint16_t *buffer)
{
    int i;
    for(i=0 ; i<WINDOW_SIZE/2 ; i++){
        UARTprintf("%d, ",buffer[i]);
        //ROM_SysCtlDelay(ROM_SysCtlClockGet() / (3 * 40));
    }
    UARTprintf("\n");
}



int main(void)
{
    //Clock CPU at 80 MHz
    //Note that with PLL enabled (SYSCTL_USE_PLL), PLL is 400 MHz and is pre-divided down to
    //200 MHz. After that, SYSCTL_SYSDIV_* divides the clock further.
    //In this case _2_5 indicates clock division of 2.5, resulting in 80 MHz.
    ROM_SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);

    //Enable FPU for use in the main program thread but not in any of the interrupts
    ROM_FPUEnable();

    ConfigureUART(); //Configures UART0 for command-line use (for debug purposes only)


    UARTprintf("Bispectrum visualizer\n");


    //Set global system state
    SystemState=HALFWINDOW_READYFORREAD_WITHB;


    //Set up uDMA
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UDMA);
    while(!ROM_SysCtlPeripheralReady(SYSCTL_PERIPH_UDMA)){}
    ROM_IntEnable(INT_UDMAERR); //Enable uDMA error interrupt
    ROM_uDMAEnable(); //Enable uDMA controller
    ROM_uDMAControlBaseSet(ui8ControlTable); //Pass pointer to base of uDMA channel control structure


    //***Set up ADC0 to use AIN1 (analog in 1) on PE2 (port E, pin 2)
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE); //PE
    while(!ROM_SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0)){}
    while(!ROM_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOE)){}
    ROM_GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_2); //PE2

    //Attempt to prevent neighbouring pins from coupling signal to the ADC input pin
    ROM_GPIOPinTypeGPIOInput(GPIO_PORTE_BASE, GPIO_PIN_1 | GPIO_PIN_3);


    //Set up sequencer 3 to be triggered by on-chip timer (this will control sample rate)
    ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_TIMER, 0);
    ROM_ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_IE | ADC_CTL_END | ADC_CTL_CH1); //Input channel 1 which corresponds to AIN1
    ROM_ADCSequenceEnable(ADC0_BASE, 3);
    ADCSequenceDMAEnable(ADC0_BASE, 3);


    //Initialize uDMA memory-to-memory transfers
    ROM_IntEnable(INT_UDMA); //Enable uDMA software channel interrupt
    ROM_uDMAChannelAttributeDisable(UDMA_CHANNEL_SW, UDMA_ATTR_USEBURST | UDMA_ATTR_ALTSELECT | (UDMA_ATTR_HIGH_PRIORITY | UDMA_ATTR_REQMASK)); //Put channel attributes in known state

    //Two samples are stored in 32 bits of memory. Therefore (2 samples) x (arbitration size of 4) = 8 samples per transfer, so WINDOW_SIZE/2 must >= 8
    ROM_uDMAChannelControlSet(UDMA_CHANNEL_SW | UDMA_PRI_SELECT, UDMA_SIZE_32 | UDMA_SRC_INC_32 | UDMA_DST_INC_32 | UDMA_ARB_4);


    //Initialize uDMA ADC-to-memory transfers
    //uDMA controller will interrupt as if it came from sequencer 3 of ADC0, so allow NVIC to pass those interrupts to the CPU.
    //(Note that peripheral SEQ3/ADC0 itself wasn't told to turn on interrupts.)
    ROM_IntEnable(INT_ADC0SS3);
    ROM_uDMAChannelAttributeDisable(UDMA_CHANNEL_ADC3, UDMA_ATTR_ALTSELECT | UDMA_ATTR_USEBURST | UDMA_ATTR_HIGH_PRIORITY | UDMA_ATTR_REQMASK);

    //Want arbitration set to 1 otherwise underflow occurs on sample sequencer (because sequencer is configured to only sample once per trigger)
    ROM_uDMAChannelControlSet(UDMA_CHANNEL_ADC3 | UDMA_PRI_SELECT, UDMA_SIZE_16 | UDMA_SRC_INC_NONE | UDMA_DST_INC_16 | UDMA_ARB_1);
    ROM_uDMAChannelControlSet(UDMA_CHANNEL_ADC3 | UDMA_ALT_SELECT, UDMA_SIZE_16 | UDMA_SRC_INC_NONE | UDMA_DST_INC_16 | UDMA_ARB_1);

    //Last argument is number of items to transfer, NOT number of bytes (udma_demo.c gives impression that it is number of bytes by use of sizeof()).
    ROM_uDMAChannelTransferSet(UDMA_CHANNEL_ADC3 | UDMA_PRI_SELECT, UDMA_MODE_PINGPONG, (void *)(ADC0_BASE + ADC_O_SSFIFO3), ADC_BufA, WINDOW_SIZE/2);
    ROM_uDMAChannelTransferSet(UDMA_CHANNEL_ADC3 | UDMA_ALT_SELECT, UDMA_MODE_PINGPONG, (void *)(ADC0_BASE + ADC_O_SSFIFO3), ADC_BufB, WINDOW_SIZE/2);
    ROM_uDMAChannelAttributeEnable(UDMA_CHANNEL_ADC3, UDMA_ATTR_HIGH_PRIORITY);
    ROM_uDMAChannelEnable(UDMA_CHANNEL_ADC3);



    //Set up timer to trigger ADC
    //Note: using wide timer 0 because there appears to a bug in the api
    //that prevents the non-wide timers from counting beyond 16 bits.
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER0);
    while(!ROM_SysCtlPeripheralReady(SYSCTL_PERIPH_WTIMER0)){}
    ROM_TimerConfigure(WTIMER0_BASE, TIMER_CFG_PERIODIC);
    ROM_TimerLoadSet(WTIMER0_BASE, TIMER_A, SAMPLING_PERIOD * ROM_SysCtlClockGet()); //Set the timer interval in # of system clock cycles
    ROM_TimerControlTrigger(WTIMER0_BASE, TIMER_A, true); //This is what permits timer to trigger ADC
    ROM_TimerEnable(WTIMER0_BASE, TIMER_A); //Enables the timer


    //RGB LED driver
    RGBInit(0);
    RGBIntensitySet(0.1f);
    uint32_t ulColors[3];
    ulColors[BLUE] = 0xFFFF;
    ulColors[RED] = 0xFFFF;
    ulColors[GREEN] = 0xFFFF;
    RGBColorSet(ulColors);
    RGBEnable();



    while(1){

        if(SystemState == HALFWINDOW_READYFORREAD_WITHB){

            print_buffer_contents(half_window);
            print_buffer_contents(ADC_BufB);

            //Advance state and start shifting immediately after done reading from buffer
            SystemState=HALFWINDOW_LOADING_FROMB;

            //Set up transfer of second half of signal_segment[WINDOW_SIZE] to its first half
            //(Two samples are being transferred as a single unit.)
            ROM_uDMAChannelTransferSet(UDMA_CHANNEL_SW | UDMA_PRI_SELECT, UDMA_MODE_AUTO, ADC_BufB, half_window, WINDOW_SIZE/4);
            ROM_uDMAChannelEnable(UDMA_CHANNEL_SW);
            ROM_uDMAChannelRequest(UDMA_CHANNEL_SW);

            }

        if(SystemState == HALFWINDOW_READYFORREAD_WITHA){

            print_buffer_contents(half_window);
            print_buffer_contents(ADC_BufA);

            SystemState=HALFWINDOW_LOADING_FROMA;

            ROM_uDMAChannelTransferSet(UDMA_CHANNEL_SW | UDMA_PRI_SELECT, UDMA_MODE_AUTO, ADC_BufA, half_window, WINDOW_SIZE/4);
            ROM_uDMAChannelEnable(UDMA_CHANNEL_SW);
            ROM_uDMAChannelRequest(UDMA_CHANNEL_SW);

        }

    }

}
