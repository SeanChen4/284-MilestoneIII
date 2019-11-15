

// Standard includes
#include <string.h>
#include <stdio.h>
#include <stdint.h>
// Driverlib includes
#include "hw_types.h"
#include "hw_memmap.h"
#include "hw_common_reg.h"
#include "hw_ints.h"
#include "spi.h"
#include "rom.h"
#include "rom_map.h"
#include "utils.h"
#include "prcm.h"
#include "uart.h"
#include "interrupt.h"
#include "hw_apps_rcm.h"
// Common interface includes
#include "uart_if.h"
#include "pin_mux_config.h"
#include "timer.h"
#include "gpio_if.h"
#include "timer_if.h"
#include "systick.h"
#define APPLICATION_VERSION     "1.4.0"
//*****************************************************************************
//
// Application Master/Slave mode selector macro
//
// MASTER_MODE = 1 : Application in master mode
// MASTER_MODE = 0 : Application in slave mode
//
//*****************************************************************************
#define MASTER_MODE      1

#define SPI_IF_BIT_RATE  400000
#define TR_BUFF_SIZE     100

#define MASTER_MSG       "This is CC3200 SPI Master Application\n\r"
#define SLAVE_MSG        "This is CC3200 SPI Slave Application\n\r"

//*****************************************************************************
//                 GLOBAL VARIABLES -- Start
//*****************************************************************************
static unsigned char g_ucTxBuff[TR_BUFF_SIZE];
static unsigned char g_ucRxBuff[TR_BUFF_SIZE];
static unsigned char ucTxBuffNdx;
static unsigned char ucRxBuffNdx;
unsigned long timer = 5000;
unsigned short counter;
int currentSampling;
int currentProcessing; 
int counters = 0;

#if defined(ccs)
extern void (* const g_pfnVectors[])(void);
#endif
#if defined(ewarm)
extern uVectorEntry __vector_table;
#endif
//*****************************************************************************
//                 GLOBAL VARIABLES -- End
//*****************************************************************************

static void TimerA0IntHandler(void)
{
    unsigned long ulStatus;
    ulStatus = MAP_TimerIntStatus(TIMERA0_BASE, true);
    MAP_TimerIntClear(TIMERA0_BASE, ulStatus);

    if (counter < 16)
    {
        currentSampling = 1;
        counter++;
    }
    else if (counter == 16)
    {
        currentProcessing = 1;
    }
}

static void TimerA1IntHandler(void)
{
    unsigned long ulStatus;
    ulStatus = MAP_TimerIntStatus(TIMERA1_BASE, true);
    MAP_TimerIntClear(TIMERA1_BASE, ulStatus);
}

static void BoardInit(void);
static void BoardInit(void) {
    /* In case of TI-RTOS vector table is initialize by OS itself */
    #ifndef USE_TIRTOS
      //
      // Set vector table base
      //
    #if defined(ccs)
        MAP_IntVTableBaseSet((unsigned long)&g_pfnVectors[0]);
    #endif
    #if defined(ewarm)
        MAP_IntVTableBaseSet((unsigned long)&__vector_table);
    #endif
    #endif

    // Enable Processor
    //
    MAP_IntMasterEnable();
    MAP_IntEnable(FAULT_SYSTICK);

    PRCMCC3200MCUInit();
}

unsigned short readADC(void)
{
    unsigned char data0;
    unsigned char data1;
    GPIOPinWrite(GPIOA1_BASE, 0x1, 0x00);
    MAP_SPITransfer(GSPI_BASE, 0, &data0, 0x1, SPI_CS_ENABLE);
    MAP_SPITransfer(GSPI_BASE, 0, &data1, 0x1, SPI_CS_DISABLE);
    GPIOPinWrite(GPIOA1_BASE, 0x1, 0x10);
    unsigned short data = 0x1f & data0;
    data = (data << 5) | ((0xf8 & data1) >> 3);
    return data;
}

//*****************************************************************************
//
//! SPI Slave Interrupt handler
//!
//! This function is invoked when SPI slave has its receive register full or
//! transmit register empty.
//!
//! \return None.
//
//*****************************************************************************


//*****************************************************************************
//
//! SPI Master mode main loop
//!
//! This function configures SPI modelue as master and enables the channel for
//! communication
//!
//! \return None.
//
//*****************************************************************************

void main()
{
    //
    // Initialize Board configurations
    //
    BoardInit();

    //
    // Muxing UART and SPI lines.
    //
    PinMuxConfig();

    //
    // Enable the SPI module clock
    //
    MAP_PRCMPeripheralClkEnable(PRCM_GSPI,PRCM_RUN_MODE_CLK);
	MAP_PRCMPeripheralReset(PRCM_GSPI);
    MAP_SPIReset(GSPI_BASE);
    MAP_SPIConfigSetExpClk(GSPI_BASE,MAP_PRCMPeripheralClockGet(PRCM_GSPI),
                           SPI_IF_BIT_RATE,SPI_MODE_MASTER,SPI_SUB_MODE_0,
                           (SPI_SW_CTRL_CS |
                           SPI_4PIN_MODE |
                           SPI_TURBO_OFF |
                           SPI_CS_ACTIVELOW |
                           SPI_WL_8));

    MAP_SPIEnable(GSPI_BASE);

    unsigned long status;
    unsigned long ulStatus;
    PRCMPeripheralClkEnable(PRCM_TIMERA0, PRCM_RUN_MODE_CLK);
    PRCMPeripheralClkEnable(PRCM_TIMERA1, PRCM_RUN_MODE_CLK);
    PRCMPeripheralReset(PRCM_TIMERA0);
    PRCMPeripheralReset(PRCM_TIMERA1);
    TimerIntRegister(TIMERA0_BASE, TIMER_A, TimerA0IntHandler);
    TimerIntRegister(TIMERA1_BASE, TIMER_A, TimerA1IntHandler);
    TimerConfigure(TIMERA0_BASE, TIMER_CFG_PERIODIC);
    TimerConfigure(TIMERA1_BASE, TIMER_CFG_ONE_SHOT);
    TimerLoadSet(TIMERA0_BASE, TIMER_A, timer);
    TimerLoadSet(TIMERA1_BASE, TIMER_A, 80000000);
    status = TimerIntStatus(TIMERA0_BASE, false);
    status = TimerIntStatus(TIMERA1_BASE, false);
    TimerIntClear(TIMERA0_BASE, status);
    TimerIntClear(TIMERA1_BASE, status);

    MAP_UARTConfigSetExpClk(UARTA1_BASE,MAP_PRCMPeripheralClockGet(PRCM_UARTA1),
                          115200, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                           UART_CONFIG_PAR_NONE));

    ulStatus = MAP_UARTIntStatus(TIMERA1_BASE, true);
    MAP_UARTIntClear(TIMERA1_BASE, ulStatus);
    MAP_UARTIntEnable(UARTA1_BASE,UART_INT_RX|UART_INT_RT);
    MAP_UARTEnable(UARTA1_BASE);
    TimerEnable(TIMERA0_BASE, TIMER_A);
    TimerIntEnable(TIMERA0_BASE, TIMER_TIMA_TIMEOUT);
    MAP_TimerIntEnable(TIMERA1_BASE, TIMER_A);
    MAP_TimerEnable(TIMERA1_BASE, TIMER_TIMA_TIMEOUT);
    //
    // Initialising the Terminal.
    //
    InitTerm();

    //
    // Clearing the Terminal.
    //
    ClearTerm();
	
¡¢
    while (1)
    {

        Report("%d  ",readADC());
          
            }
    }
    //
    // Display the Banner
    //
   

    //
    // Reset the peripheral
    //
    MAP_PRCMPeripheralReset(PRCM_GSPI);



}

