/* Teensyduino Core Library
 * http://www.pjrc.com/teensy/
 * Copyright (c) 2013 PJRC.COM, LLC.
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * 1. The above copyright notice and this permission notice shall be 
 * included in all copies or substantial portions of the Software.
 *
 * 2. If the Software is incorporated into a build system that allows 
 * selection among a list of target devices, then similar target
 * devices manufactured by PJRC.COM must be included in the list of
 * target devices and selectable in the same manner.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "kinetis.h"
#include "core_pins.h" // testing only
#include "ser_print.h" // testing only


// Flash Security Setting. On Teensy 3.2, you can lock the MK20 chip to prevent
// anyone from reading your code.  You CAN still reprogram your Teensy while
// security is set, but the bootloader will be unable to respond to auto-reboot
// requests from Arduino. Pressing the program button will cause a full chip
// erase to gain access, because the bootloader chip is locked out.  Normally,
// erase occurs when uploading begins, so if you press the Program button
// accidentally, simply power cycling will run your program again.  When
// security is locked, any Program button press causes immediate full erase.
// Special care must be used with the Program button, because it must be made
// accessible to initiate reprogramming, but it must not be accidentally
// pressed when Teensy Loader is not being used to reprogram.  To set lock the
// security change this to 0xDC.  Teensy 3.0 and 3.1 do not support security lock.
#define FSEC 0xDE

// Flash Options
#define FOPT 0xF9


extern unsigned long _stext;
extern unsigned long _etext;
extern unsigned long _sdata;
extern unsigned long _edata;
extern unsigned long _sbss;
extern unsigned long _ebss;
extern unsigned long _estack;
//extern void __init_array_start(void);
//extern void __init_array_end(void);



extern int main (void);
void ResetHandler(void);
void _init_Teensyduino_internal_(void);
void __libc_init_array(void);


void fault_isr(void)
{
#if 0
	uint32_t addr;

	SIM_SCGC4 |= 0x00000400;
	UART0_BDH = 0;
	UART0_BDL = 26; // 115200 at 48 MHz
	UART0_C2 = UART_C2_TE;
	PORTB_PCR17 = PORT_PCR_MUX(3);
	ser_print("\nfault: \n??: ");
        asm("ldr %0, [sp, #52]" : "=r" (addr) ::);
        ser_print_hex32(addr);
        ser_print("\n??: ");
        asm("ldr %0, [sp, #48]" : "=r" (addr) ::);
        ser_print_hex32(addr);
        ser_print("\n??: ");
        asm("ldr %0, [sp, #44]" : "=r" (addr) ::);
        ser_print_hex32(addr);
        ser_print("\npsr:");
        asm("ldr %0, [sp, #40]" : "=r" (addr) ::);
        ser_print_hex32(addr);
        ser_print("\nadr:");
        asm("ldr %0, [sp, #36]" : "=r" (addr) ::);
        ser_print_hex32(addr);
        ser_print("\nlr: ");
        asm("ldr %0, [sp, #32]" : "=r" (addr) ::);
        ser_print_hex32(addr);
        ser_print("\nr12:");
        asm("ldr %0, [sp, #28]" : "=r" (addr) ::);
        ser_print_hex32(addr);
        ser_print("\nr3: ");
        asm("ldr %0, [sp, #24]" : "=r" (addr) ::);
        ser_print_hex32(addr);
        ser_print("\nr2: ");
        asm("ldr %0, [sp, #20]" : "=r" (addr) ::);
        ser_print_hex32(addr);
        ser_print("\nr1: ");
        asm("ldr %0, [sp, #16]" : "=r" (addr) ::);
        ser_print_hex32(addr);
        ser_print("\nr0: ");
        asm("ldr %0, [sp, #12]" : "=r" (addr) ::);
        ser_print_hex32(addr);
        ser_print("\nr4: ");
        asm("ldr %0, [sp, #8]" : "=r" (addr) ::);
        ser_print_hex32(addr);
        ser_print("\nlr: ");
        asm("ldr %0, [sp, #4]" : "=r" (addr) ::);
        ser_print_hex32(addr);
        ser_print("\n");
        asm("ldr %0, [sp, #0]" : "=r" (addr) ::);
#endif
	while (1) {
		// keep polling some communication while in fault
		// mode, so we don't completely die.
		if (SIM_SCGC4 & SIM_SCGC4_USBOTG) usb_isr();
		if (SIM_SCGC4 & SIM_SCGC4_UART0) uart0_status_isr();
		if (SIM_SCGC4 & SIM_SCGC4_UART1) uart1_status_isr();
		if (SIM_SCGC4 & SIM_SCGC4_UART2) uart2_status_isr();
	}
}

void unused_isr(void)
{
	fault_isr();
}

extern volatile uint32_t systick_millis_count;
void systick_default_isr(void)
{
	systick_millis_count++;
}

void nmi_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void hard_fault_isr(void)	__attribute__ ((weak, alias("fault_isr")));
void memmanage_fault_isr(void)	__attribute__ ((weak, alias("fault_isr")));
void bus_fault_isr(void)	__attribute__ ((weak, alias("fault_isr")));
void usage_fault_isr(void)	__attribute__ ((weak, alias("fault_isr")));
void svcall_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void debugmonitor_isr(void)	__attribute__ ((weak, alias("unused_isr")));
void pendablesrvreq_isr(void)	__attribute__ ((weak, alias("unused_isr")));
void systick_isr(void)		__attribute__ ((weak, alias("systick_default_isr")));

void dma_ch0_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void dma_ch1_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void dma_ch2_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void dma_ch3_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void dma_ch4_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void dma_ch5_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void dma_ch6_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void dma_ch7_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void dma_ch8_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void dma_ch9_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void dma_ch10_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void dma_ch11_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void dma_ch12_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void dma_ch13_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void dma_ch14_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void dma_ch15_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void dma_error_isr(void)	__attribute__ ((weak, alias("unused_isr")));
void mcm_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void randnum_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void flash_cmd_isr(void)	__attribute__ ((weak, alias("unused_isr")));
void flash_error_isr(void)	__attribute__ ((weak, alias("unused_isr")));
void low_voltage_isr(void)	__attribute__ ((weak, alias("unused_isr")));
void wakeup_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void watchdog_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void i2c0_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void i2c1_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void i2c2_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void i2c3_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void spi0_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void spi1_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void spi2_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void sdhc_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void enet_timer_isr(void)	__attribute__ ((weak, alias("unused_isr")));
void enet_tx_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void enet_rx_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void enet_error_isr(void)	__attribute__ ((weak, alias("unused_isr")));
void can0_message_isr(void)	__attribute__ ((weak, alias("unused_isr")));
void can0_bus_off_isr(void)	__attribute__ ((weak, alias("unused_isr")));
void can0_error_isr(void)	__attribute__ ((weak, alias("unused_isr")));
void can0_tx_warn_isr(void)	__attribute__ ((weak, alias("unused_isr")));
void can0_rx_warn_isr(void)	__attribute__ ((weak, alias("unused_isr")));
void can0_wakeup_isr(void)	__attribute__ ((weak, alias("unused_isr")));
void can1_message_isr(void)	__attribute__ ((weak, alias("unused_isr")));
void can1_bus_off_isr(void)	__attribute__ ((weak, alias("unused_isr")));
void can1_error_isr(void)	__attribute__ ((weak, alias("unused_isr")));
void can1_tx_warn_isr(void)	__attribute__ ((weak, alias("unused_isr")));
void can1_rx_warn_isr(void)	__attribute__ ((weak, alias("unused_isr")));
void can1_wakeup_isr(void)	__attribute__ ((weak, alias("unused_isr")));
void i2s0_tx_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void i2s0_rx_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void i2s0_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void uart0_lon_isr(void)	__attribute__ ((weak, alias("unused_isr")));
void uart0_status_isr(void)	__attribute__ ((weak, alias("unused_isr")));
void uart0_error_isr(void)	__attribute__ ((weak, alias("unused_isr")));
void uart1_status_isr(void)	__attribute__ ((weak, alias("unused_isr")));
void uart1_error_isr(void)	__attribute__ ((weak, alias("unused_isr")));
void uart2_status_isr(void)	__attribute__ ((weak, alias("unused_isr")));
void uart2_error_isr(void)	__attribute__ ((weak, alias("unused_isr")));
void uart3_status_isr(void)	__attribute__ ((weak, alias("unused_isr")));
void uart3_error_isr(void)	__attribute__ ((weak, alias("unused_isr")));
void uart4_status_isr(void)	__attribute__ ((weak, alias("unused_isr")));
void uart4_error_isr(void)	__attribute__ ((weak, alias("unused_isr")));
void uart5_status_isr(void)	__attribute__ ((weak, alias("unused_isr")));
void uart5_error_isr(void)	__attribute__ ((weak, alias("unused_isr")));
void lpuart0_status_isr(void)	__attribute__ ((weak, alias("unused_isr")));
void adc0_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void adc1_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void cmp0_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void cmp1_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void cmp2_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void cmp3_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void ftm0_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void ftm1_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void ftm2_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void ftm3_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void tpm0_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void tpm1_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void tpm2_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void cmt_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void rtc_alarm_isr(void)	__attribute__ ((weak, alias("unused_isr")));
void rtc_seconds_isr(void)	__attribute__ ((weak, alias("unused_isr")));
void pit_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void pit0_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void pit1_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void pit2_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void pit3_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void pdb_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void usb_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void usb_charge_isr(void)	__attribute__ ((weak, alias("unused_isr")));
void usbhs_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void usbhs_phy_isr(void)	__attribute__ ((weak, alias("unused_isr")));
void dac0_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void dac1_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void tsi0_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void mcg_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void lptmr_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void porta_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void portb_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void portc_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void portd_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void porte_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void portcd_isr(void)		__attribute__ ((weak, alias("unused_isr")));
void software_isr(void)		__attribute__ ((weak, alias("unused_isr")));

#if defined(__MK20DX128__)
__attribute__ ((section(".dmabuffers"), used, aligned(256)))
#elif defined(__MK20DX256__)
__attribute__ ((section(".dmabuffers"), used, aligned(512)))
#elif defined(__MKL26Z64__)
__attribute__ ((section(".dmabuffers"), used, aligned(256)))
#elif defined(__MK64FX512__) || defined(__FRDM_K64F__)
__attribute__ ((section(".dmabuffers"), used, aligned(512)))
#elif defined(__MK66FX1M0__)
__attribute__ ((section(".dmabuffers"), used, aligned(512)))
#endif
void (* _VectorsRam[NVIC_NUM_INTERRUPTS+16])(void);

__attribute__ ((section(".vectors"), used))
void (* const _VectorsFlash[NVIC_NUM_INTERRUPTS+16])(void) =
{
	(void (*)(void))((unsigned long)&_estack),	//  0 ARM: Initial Stack Pointer
	ResetHandler,					//  1 ARM: Initial Program Counter
	nmi_isr,					//  2 ARM: Non-maskable Interrupt (NMI)
	hard_fault_isr,					//  3 ARM: Hard Fault
	memmanage_fault_isr,				//  4 ARM: MemManage Fault
	bus_fault_isr,					//  5 ARM: Bus Fault
	usage_fault_isr,				//  6 ARM: Usage Fault
	fault_isr,					//  7 --
	fault_isr,					//  8 --
	fault_isr,					//  9 --
	fault_isr,					// 10 --
	svcall_isr,					// 11 ARM: Supervisor call (SVCall)
	debugmonitor_isr,				// 12 ARM: Debug Monitor
	fault_isr,					// 13 --
	pendablesrvreq_isr,				// 14 ARM: Pendable req serv(PendableSrvReq)
	systick_isr,					// 15 ARM: System tick timer (SysTick)
#if defined(__MK20DX128__)
	dma_ch0_isr,					// 16 DMA channel 0 transfer complete
	dma_ch1_isr,					// 17 DMA channel 1 transfer complete
	dma_ch2_isr,					// 18 DMA channel 2 transfer complete
	dma_ch3_isr,					// 19 DMA channel 3 transfer complete
	dma_error_isr,					// 20 DMA error interrupt channel
	unused_isr,					// 21 DMA --
	flash_cmd_isr,					// 22 Flash Memory Command complete
	flash_error_isr,				// 23 Flash Read collision
	low_voltage_isr,				// 24 Low-voltage detect/warning
	wakeup_isr,					// 25 Low Leakage Wakeup
	watchdog_isr,					// 26 Both EWM and WDOG interrupt
	i2c0_isr,					// 27 I2C0
	spi0_isr,					// 28 SPI0
	i2s0_tx_isr,					// 29 I2S0 Transmit
	i2s0_rx_isr,					// 30 I2S0 Receive
	uart0_lon_isr,					// 31 UART0 CEA709.1-B (LON) status
	uart0_status_isr,				// 32 UART0 status
	uart0_error_isr,				// 33 UART0 error
	uart1_status_isr,				// 34 UART1 status
	uart1_error_isr,				// 35 UART1 error
	uart2_status_isr,				// 36 UART2 status
	uart2_error_isr,				// 37 UART2 error
	adc0_isr,					// 38 ADC0
	cmp0_isr,					// 39 CMP0
	cmp1_isr,					// 40 CMP1
	ftm0_isr,					// 41 FTM0
	ftm1_isr,					// 42 FTM1
	cmt_isr,					// 43 CMT
	rtc_alarm_isr,					// 44 RTC Alarm interrupt
	rtc_seconds_isr,				// 45 RTC Seconds interrupt
	pit0_isr,					// 46 PIT Channel 0
	pit1_isr,					// 47 PIT Channel 1
	pit2_isr,					// 48 PIT Channel 2
	pit3_isr,					// 49 PIT Channel 3
	pdb_isr,					// 50 PDB Programmable Delay Block
	usb_isr,					// 51 USB OTG
	usb_charge_isr,					// 52 USB Charger Detect
	tsi0_isr,					// 53 TSI0
	mcg_isr,					// 54 MCG
	lptmr_isr,					// 55 Low Power Timer
	porta_isr,					// 56 Pin detect (Port A)
	portb_isr,					// 57 Pin detect (Port B)
	portc_isr,					// 58 Pin detect (Port C)
	portd_isr,					// 59 Pin detect (Port D)
	porte_isr,					// 60 Pin detect (Port E)
	software_isr,					// 61 Software interrupt
#elif defined(__MK20DX256__)
	dma_ch0_isr,					// 16 DMA channel 0 transfer complete
	dma_ch1_isr,					// 17 DMA channel 1 transfer complete
	dma_ch2_isr,					// 18 DMA channel 2 transfer complete
	dma_ch3_isr,					// 19 DMA channel 3 transfer complete
	dma_ch4_isr,					// 20 DMA channel 4 transfer complete
	dma_ch5_isr,					// 21 DMA channel 5 transfer complete
	dma_ch6_isr,					// 22 DMA channel 6 transfer complete
	dma_ch7_isr,					// 23 DMA channel 7 transfer complete
	dma_ch8_isr,					// 24 DMA channel 8 transfer complete
	dma_ch9_isr,					// 25 DMA channel 9 transfer complete
	dma_ch10_isr,					// 26 DMA channel 10 transfer complete
	dma_ch11_isr,					// 27 DMA channel 11 transfer complete
	dma_ch12_isr,					// 28 DMA channel 12 transfer complete
	dma_ch13_isr,					// 29 DMA channel 13 transfer complete
	dma_ch14_isr,					// 30 DMA channel 14 transfer complete
	dma_ch15_isr,					// 31 DMA channel 15 transfer complete
	dma_error_isr,					// 32 DMA error interrupt channel
	unused_isr,					// 33 --
	flash_cmd_isr,					// 34 Flash Memory Command complete
	flash_error_isr,				// 35 Flash Read collision
	low_voltage_isr,				// 36 Low-voltage detect/warning
	wakeup_isr,					// 37 Low Leakage Wakeup
	watchdog_isr,					// 38 Both EWM and WDOG interrupt
	unused_isr,					// 39 --
	i2c0_isr,					// 40 I2C0
	i2c1_isr,					// 41 I2C1
	spi0_isr,					// 42 SPI0
	spi1_isr,					// 43 SPI1
	unused_isr,					// 44 --
	can0_message_isr,				// 45 CAN OR'ed Message buffer (0-15)
	can0_bus_off_isr,				// 46 CAN Bus Off
	can0_error_isr,					// 47 CAN Error
	can0_tx_warn_isr,				// 48 CAN Transmit Warning
	can0_rx_warn_isr,				// 49 CAN Receive Warning
	can0_wakeup_isr,				// 50 CAN Wake Up
	i2s0_tx_isr,					// 51 I2S0 Transmit
	i2s0_rx_isr,					// 52 I2S0 Receive
	unused_isr,					// 53 --
	unused_isr,					// 54 --
	unused_isr,					// 55 --
	unused_isr,					// 56 --
	unused_isr,					// 57 --
	unused_isr,					// 58 --
	unused_isr,					// 59 --
	uart0_lon_isr,					// 60 UART0 CEA709.1-B (LON) status
	uart0_status_isr,				// 61 UART0 status
	uart0_error_isr,				// 62 UART0 error
	uart1_status_isr,				// 63 UART1 status
	uart1_error_isr,				// 64 UART1 error
	uart2_status_isr,				// 65 UART2 status
	uart2_error_isr,				// 66 UART2 error
	unused_isr,					// 67 --
	unused_isr,					// 68 --
	unused_isr,					// 69 --
	unused_isr,					// 70 --
	unused_isr,					// 71 --
	unused_isr,					// 72 --
	adc0_isr,					// 73 ADC0
	adc1_isr,					// 74 ADC1
	cmp0_isr,					// 75 CMP0
	cmp1_isr,					// 76 CMP1
	cmp2_isr,					// 77 CMP2
	ftm0_isr,					// 78 FTM0
	ftm1_isr,					// 79 FTM1
	ftm2_isr,					// 80 FTM2
	cmt_isr,					// 81 CMT
	rtc_alarm_isr,					// 82 RTC Alarm interrupt
	rtc_seconds_isr,				// 83 RTC Seconds interrupt
	pit0_isr,					// 84 PIT Channel 0
	pit1_isr,					// 85 PIT Channel 1
	pit2_isr,					// 86 PIT Channel 2
	pit3_isr,					// 87 PIT Channel 3
	pdb_isr,					// 88 PDB Programmable Delay Block
	usb_isr,					// 89 USB OTG
	usb_charge_isr,					// 90 USB Charger Detect
	unused_isr,					// 91 --
	unused_isr,					// 92 --
	unused_isr,					// 93 --
	unused_isr,					// 94 --
	unused_isr,					// 95 --
	unused_isr,					// 96 --
	dac0_isr,					// 97 DAC0
	unused_isr,					// 98 --
	tsi0_isr,					// 99 TSI0
	mcg_isr,					// 100 MCG
	lptmr_isr,					// 101 Low Power Timer
	unused_isr,					// 102 --
	porta_isr,					// 103 Pin detect (Port A)
	portb_isr,					// 104 Pin detect (Port B)
	portc_isr,					// 105 Pin detect (Port C)
	portd_isr,					// 106 Pin detect (Port D)
	porte_isr,					// 107 Pin detect (Port E)
	unused_isr,					// 108 --
	unused_isr,					// 109 --
	software_isr,					// 110 Software interrupt
#elif defined(__MKL26Z64__)
	dma_ch0_isr,					// 16 DMA channel 0 transfer complete
	dma_ch1_isr,					// 17 DMA channel 1 transfer complete
	dma_ch2_isr,					// 18 DMA channel 2 transfer complete
	dma_ch3_isr,					// 19 DMA channel 3 transfer complete
	unused_isr,					// 20 --
	flash_cmd_isr,					// 21 Flash Memory Command complete
	low_voltage_isr,				// 22 Low-voltage detect/warning
	wakeup_isr,					// 23 Low Leakage Wakeup
	i2c0_isr,					// 24 I2C0
	i2c1_isr,					// 25 I2C1
	spi0_isr,					// 26 SPI0
	spi1_isr,					// 27 SPI1
	uart0_status_isr,				// 28 UART0 status & error
	uart1_status_isr,				// 29 UART1 status & error
	uart2_status_isr,				// 30 UART2 status & error
	adc0_isr,					// 31 ADC0
	cmp0_isr,					// 32 CMP0
	ftm0_isr,					// 33 FTM0
	ftm1_isr,					// 34 FTM1
	ftm2_isr,					// 35 FTM2
	rtc_alarm_isr,					// 36 RTC Alarm interrupt
	rtc_seconds_isr,				// 37 RTC Seconds interrupt
	pit_isr,					// 38 PIT Both Channels
	i2s0_isr,					// 39 I2S0 Transmit & Receive
	usb_isr,					// 40 USB OTG
	dac0_isr,					// 41 DAC0
	tsi0_isr,					// 42 TSI0
	mcg_isr,					// 43 MCG
	lptmr_isr,					// 44 Low Power Timer
	software_isr,					// 45 Software interrupt
	porta_isr,					// 46 Pin detect (Port A)
	portcd_isr,					// 47 Pin detect (Port C and D)
#elif defined(__MK64FX512__) || defined(__FRDM_K64F__)
	dma_ch0_isr,					// 16 DMA channel 0 transfer complete
	dma_ch1_isr,					// 17 DMA channel 1 transfer complete
	dma_ch2_isr,					// 18 DMA channel 2 transfer complete
	dma_ch3_isr,					// 19 DMA channel 3 transfer complete
	dma_ch4_isr,					// 20 DMA channel 4 transfer complete
	dma_ch5_isr,					// 21 DMA channel 5 transfer complete
	dma_ch6_isr,					// 22 DMA channel 6 transfer complete
	dma_ch7_isr,					// 23 DMA channel 7 transfer complete
	dma_ch8_isr,					// 24 DMA channel 8 transfer complete
	dma_ch9_isr,					// 25 DMA channel 9 transfer complete
	dma_ch10_isr,					// 26 DMA channel 10 transfer complete
	dma_ch11_isr,					// 27 DMA channel 11 transfer complete
	dma_ch12_isr,					// 28 DMA channel 12 transfer complete
	dma_ch13_isr,					// 29 DMA channel 13 transfer complete
	dma_ch14_isr,					// 30 DMA channel 14 transfer complete
	dma_ch15_isr,					// 31 DMA channel 15 transfer complete
	dma_error_isr,					// 32 DMA error interrupt channel
	mcm_isr,					// 33 MCM
	flash_cmd_isr,					// 34 Flash Memory Command complete
	flash_error_isr,				// 35 Flash Read collision
	low_voltage_isr,				// 36 Low-voltage detect/warning
	wakeup_isr,					// 37 Low Leakage Wakeup
	watchdog_isr,					// 38 Both EWM and WDOG interrupt
	randnum_isr,					// 39 Random Number Generator
	i2c0_isr,					// 40 I2C0
	i2c1_isr,					// 41 I2C1
	spi0_isr,					// 42 SPI0
	spi1_isr,					// 43 SPI1
	i2s0_tx_isr,					// 44 I2S0 Transmit
	i2s0_rx_isr,					// 45 I2S0 Receive
	unused_isr,					// 46 --
	uart0_status_isr,				// 47 UART0 status
	uart0_error_isr,				// 48 UART0 error
	uart1_status_isr,				// 49 UART1 status
	uart1_error_isr,				// 50 UART1 error
	uart2_status_isr,				// 51 UART2 status
	uart2_error_isr,				// 52 UART2 error
	uart3_status_isr,				// 53 UART3 status
	uart3_error_isr,				// 54 UART3 error
	adc0_isr,					// 55 ADC0
	cmp0_isr,					// 56 CMP0
	cmp1_isr,					// 57 CMP1
	ftm0_isr,					// 58 FTM0
	ftm1_isr,					// 59 FTM1
	ftm2_isr,					// 60 FTM2
	cmt_isr,					// 61 CMT
	rtc_alarm_isr,					// 62 RTC Alarm interrupt
	rtc_seconds_isr,				// 63 RTC Seconds interrupt
	pit0_isr,					// 64 PIT Channel 0
	pit1_isr,					// 65 PIT Channel 1
	pit2_isr,					// 66 PIT Channel 2
	pit3_isr,					// 67 PIT Channel 3
	pdb_isr,					// 68 PDB Programmable Delay Block
	usb_isr,					// 69 USB OTG
	usb_charge_isr,					// 70 USB Charger Detect
	unused_isr,					// 71 --
	dac0_isr,					// 72 DAC0
	mcg_isr,					// 73 MCG
	lptmr_isr,					// 74 Low Power Timer
	porta_isr,					// 75 Pin detect (Port A)
	portb_isr,					// 76 Pin detect (Port B)
	portc_isr,					// 77 Pin detect (Port C)
	portd_isr,					// 78 Pin detect (Port D)
	porte_isr,					// 79 Pin detect (Port E)
	software_isr,					// 80 Software interrupt
	spi2_isr,					// 81 SPI2
	uart4_status_isr,				// 82 UART4 status
	uart4_error_isr,				// 83 UART4 error
	uart5_status_isr,				// 84 UART4 status
	uart5_error_isr,				// 85 UART4 error
	cmp2_isr,					// 86 CMP2
	ftm3_isr,					// 87 FTM3
	dac1_isr,					// 88 DAC1
	adc1_isr,					// 89 ADC1
	i2c2_isr,					// 90 I2C2
	can0_message_isr,				// 91 CAN OR'ed Message buffer (0-15)
	can0_bus_off_isr,				// 92 CAN Bus Off
	can0_error_isr,					// 93 CAN Error
	can0_tx_warn_isr,				// 94 CAN Transmit Warning
	can0_rx_warn_isr,				// 95 CAN Receive Warning
	can0_wakeup_isr,				// 96 CAN Wake Up
	sdhc_isr,					// 97 SDHC
	enet_timer_isr,					// 98 Ethernet IEEE1588 Timers
	enet_tx_isr,					// 99 Ethernet Transmit
	enet_rx_isr,					// 100 Ethernet Receive
	enet_error_isr,					// 101 Ethernet Error
#elif defined(__MK66FX1M0__)
	dma_ch0_isr,					// 16 DMA channel 0 transfer complete
	dma_ch1_isr,					// 17 DMA channel 1 transfer complete
	dma_ch2_isr,					// 18 DMA channel 2 transfer complete
	dma_ch3_isr,					// 19 DMA channel 3 transfer complete
	dma_ch4_isr,					// 20 DMA channel 4 transfer complete
	dma_ch5_isr,					// 21 DMA channel 5 transfer complete
	dma_ch6_isr,					// 22 DMA channel 6 transfer complete
	dma_ch7_isr,					// 23 DMA channel 7 transfer complete
	dma_ch8_isr,					// 24 DMA channel 8 transfer complete
	dma_ch9_isr,					// 25 DMA channel 9 transfer complete
	dma_ch10_isr,					// 26 DMA channel 10 transfer complete
	dma_ch11_isr,					// 27 DMA channel 11 transfer complete
	dma_ch12_isr,					// 28 DMA channel 12 transfer complete
	dma_ch13_isr,					// 29 DMA channel 13 transfer complete
	dma_ch14_isr,					// 30 DMA channel 14 transfer complete
	dma_ch15_isr,					// 31 DMA channel 15 transfer complete
	dma_error_isr,					// 32 DMA error interrupt channel
	mcm_isr,					// 33 MCM
	flash_cmd_isr,					// 34 Flash Memory Command complete
	flash_error_isr,				// 35 Flash Read collision
	low_voltage_isr,				// 36 Low-voltage detect/warning
	wakeup_isr,					// 37 Low Leakage Wakeup
	watchdog_isr,					// 38 Both EWM and WDOG interrupt
	randnum_isr,					// 39 Random Number Generator
	i2c0_isr,					// 40 I2C0
	i2c1_isr,					// 41 I2C1
	spi0_isr,					// 42 SPI0
	spi1_isr,					// 43 SPI1
	i2s0_tx_isr,					// 44 I2S0 Transmit
	i2s0_rx_isr,					// 45 I2S0 Receive
	unused_isr,					// 46 --
	uart0_status_isr,				// 47 UART0 status
	uart0_error_isr,				// 48 UART0 error
	uart1_status_isr,				// 49 UART1 status
	uart1_error_isr,				// 50 UART1 error
	uart2_status_isr,				// 51 UART2 status
	uart2_error_isr,				// 52 UART2 error
	uart3_status_isr,				// 53 UART3 status
	uart3_error_isr,				// 54 UART3 error
	adc0_isr,					// 55 ADC0
	cmp0_isr,					// 56 CMP0
	cmp1_isr,					// 57 CMP1
	ftm0_isr,					// 58 FTM0
	ftm1_isr,					// 59 FTM1
	ftm2_isr,					// 60 FTM2
	cmt_isr,					// 61 CMT
	rtc_alarm_isr,					// 62 RTC Alarm interrupt
	rtc_seconds_isr,				// 63 RTC Seconds interrupt
	pit0_isr,					// 64 PIT Channel 0
	pit1_isr,					// 65 PIT Channel 1
	pit2_isr,					// 66 PIT Channel 2
	pit3_isr,					// 67 PIT Channel 3
	pdb_isr,					// 68 PDB Programmable Delay Block
	usb_isr,					// 69 USB OTG
	usb_charge_isr,					// 70 USB Charger Detect
	unused_isr,					// 71 --
	dac0_isr,					// 72 DAC0
	mcg_isr,					// 73 MCG
	lptmr_isr,					// 74 Low Power Timer
	porta_isr,					// 75 Pin detect (Port A)
	portb_isr,					// 76 Pin detect (Port B)
	portc_isr,					// 77 Pin detect (Port C)
	portd_isr,					// 78 Pin detect (Port D)
	porte_isr,					// 79 Pin detect (Port E)
	software_isr,					// 80 Software interrupt
	spi2_isr,					// 81 SPI2
	uart4_status_isr,				// 82 UART4 status
	uart4_error_isr,				// 83 UART4 error
	unused_isr,					// 84 --
	unused_isr,					// 85 --
	cmp2_isr,					// 86 CMP2
	ftm3_isr,					// 87 FTM3
	dac1_isr,					// 88 DAC1
	adc1_isr,					// 89 ADC1
	i2c2_isr,					// 90 I2C2
	can0_message_isr,				// 91 CAN OR'ed Message buffer (0-15)
	can0_bus_off_isr,				// 92 CAN Bus Off
	can0_error_isr,					// 93 CAN Error
	can0_tx_warn_isr,				// 94 CAN Transmit Warning
	can0_rx_warn_isr,				// 95 CAN Receive Warning
	can0_wakeup_isr,				// 96 CAN Wake Up
	sdhc_isr,					// 97 SDHC
	enet_timer_isr,					// 98 Ethernet IEEE1588 Timers
	enet_tx_isr,					// 99 Ethernet Transmit
	enet_rx_isr,					// 100 Ethernet Receive
	enet_error_isr,					// 101 Ethernet Error
	lpuart0_status_isr,				// 102 LPUART
	tsi0_isr,					// 103 TSI0
	tpm1_isr,					// 104 FTM1
	tpm2_isr,					// 105 FTM2
	usbhs_phy_isr,					// 106 USB-HS Phy
	i2c3_isr,					// 107 I2C3
	cmp3_isr,					// 108 CMP3
	usbhs_isr,					// 109 USB-HS
	can1_message_isr,				// 110 CAN OR'ed Message buffer (0-15)
	can1_bus_off_isr,				// 111 CAN Bus Off
	can1_error_isr,					// 112 CAN Error
	can1_tx_warn_isr,				// 113 CAN Transmit Warning
	can1_rx_warn_isr,				// 114 CAN Receive Warning
	can1_wakeup_isr,				// 115 CAN Wake Up
#endif
};


__attribute__ ((section(".flashconfig"), used))
const uint8_t flashconfigbytes[16] = {
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, FSEC, FOPT, 0xFF, 0xFF
};


// Automatically initialize the RTC.  When the build defines the compile
// time, and the user has added a crystal, the RTC will automatically
// begin at the time of the first upload.
#ifndef TIME_T
#define TIME_T 1349049600 // default 1 Oct 2012 (never used, Arduino sets this)
#endif
extern void *__rtc_localtime; // Arduino build process sets this
extern void rtc_set(unsigned long t);


static void startup_default_early_hook(void) {
#if defined(KINETISK)
	WDOG_STCTRLH = WDOG_STCTRLH_ALLOWUPDATE;
#elif defined(KINETISL)
	SIM_COPC = 0;  // disable the watchdog
#endif
}
static void startup_default_late_hook(void) {}
void startup_early_hook(void)		__attribute__ ((weak, alias("startup_default_early_hook")));
void startup_late_hook(void)		__attribute__ ((weak, alias("startup_default_late_hook")));


#ifdef __clang__
// Clang seems to generate slightly larger code with Os than gcc
__attribute__ ((optimize("-Os")))
#else
__attribute__ ((section(".startup"),optimize("-Os")))
#endif
void ResetHandler(void)
{
	uint32_t *src = &_etext;
	uint32_t *dest = &_sdata;
	unsigned int i;
#if F_CPU <= 2000000
	volatile int n;
#endif
	//volatile int count;

#ifdef KINETISK
	WDOG_UNLOCK = WDOG_UNLOCK_SEQ1;
	WDOG_UNLOCK = WDOG_UNLOCK_SEQ2;
	__asm__ volatile ("nop");
	__asm__ volatile ("nop");
#endif
	// programs using the watchdog timer or needing to initialize hardware as
	// early as possible can implement startup_early_hook()
	startup_early_hook();

	// enable clocks to always-used peripherals
#if defined(__MK20DX128__)
	SIM_SCGC5 = 0x00043F82;		// clocks active to all GPIO
	SIM_SCGC6 = SIM_SCGC6_RTC | SIM_SCGC6_FTM0 | SIM_SCGC6_FTM1 | SIM_SCGC6_ADC0 | SIM_SCGC6_FTFL;
#elif defined(__MK20DX256__)
	SIM_SCGC3 = SIM_SCGC3_ADC1 | SIM_SCGC3_FTM2;
	SIM_SCGC5 = 0x00043F82;		// clocks active to all GPIO
	SIM_SCGC6 = SIM_SCGC6_RTC | SIM_SCGC6_FTM0 | SIM_SCGC6_FTM1 | SIM_SCGC6_ADC0 | SIM_SCGC6_FTFL;
#elif defined(__MK64FX512__) || defined(__MK66FX1M0__)
	SIM_SCGC3 = SIM_SCGC3_ADC1 | SIM_SCGC3_FTM2 | SIM_SCGC3_FTM3;
	SIM_SCGC5 = 0x00043F82;		// clocks active to all GPIO
	SIM_SCGC6 = SIM_SCGC6_RTC | SIM_SCGC6_FTM0 | SIM_SCGC6_FTM1 | SIM_SCGC6_ADC0 | SIM_SCGC6_FTFL;
	//PORTC_PCR5 = PORT_PCR_MUX(1) | PORT_PCR_DSE | PORT_PCR_SRE;
	//GPIOC_PDDR |= (1<<5);
	//GPIOC_PSOR = (1<<5);
	//while (1);
#elif defined(__MKL26Z64__)
	SIM_SCGC4 = SIM_SCGC4_USBOTG | 0xF0000030;
	SIM_SCGC5 = 0x00003F82;		// clocks active to all GPIO
	SIM_SCGC6 = SIM_SCGC6_ADC0 | SIM_SCGC6_TPM0 | SIM_SCGC6_TPM1 | SIM_SCGC6_TPM2 | SIM_SCGC6_FTFL;
#elif defined(__FRDM_K64F__)
    SIM_SCGC3 = SIM_SCGC3_ADC1 | SIM_SCGC3_FTM2 | SIM_SCGC3_FTM3;
    // clocks active to all GPIO and the Low Power Timer
    SIM_SCGC5 = SIM_SCGC5_PORTE | SIM_SCGC5_PORTD | SIM_SCGC5_PORTC | SIM_SCGC5_PORTB | SIM_SCGC5_PORTA
            | SIM_SCGC5_LPTIMER;
    SIM_SCGC6 = SIM_SCGC6_RTC | SIM_SCGC6_FTM0 | SIM_SCGC6_FTM1 | SIM_SCGC6_ADC0 | SIM_SCGC6_FTFL;
#endif
#if defined(__MK64FX512__) || defined(__MK66FX1M0__)
	SCB_CPACR = 0x00F00000;
#endif
#if defined(__MK66FX1M0__)
	LMEM_PCCCR = 0x85000003;
#endif
#if 0
	// testing only, enable ser_print
	SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIV1(0) | SIM_CLKDIV1_OUTDIV4(1);
	MCG_C4 |= MCG_C4_DMX32 | MCG_C4_DRST_DRS(1);
	SIM_SOPT2 = SIM_SOPT2_UART0SRC(1) | SIM_SOPT2_TPMSRC(1);
	SIM_SCGC4 |= 0x00000400;
	UART0_BDH = 0;
	UART0_BDL = 26; // 115200 at 48 MHz
	UART0_C2 = UART_C2_TE;
	PORTB_PCR17 = PORT_PCR_MUX(3);
#endif
#ifdef KINETISK
	// if the RTC oscillator isn't enabled, get it started early
	if (!(RTC_CR & RTC_CR_OSCE)) {
		RTC_SR = 0;
		RTC_CR = RTC_CR_SC16P | RTC_CR_SC4P | RTC_CR_OSCE;
	}
#endif
	// release I/O pins hold, if we woke up from VLLS mode
	if (PMC_REGSC & PMC_REGSC_ACKISO) PMC_REGSC |= PMC_REGSC_ACKISO;

    // since this is a write once register, make it visible to all F_CPU's
    // so we can into other sleep modes in the future at any speed
#if defined(__MK66FX1M0__)
	SMC_PMPROT = SMC_PMPROT_AHSRUN | SMC_PMPROT_AVLP | SMC_PMPROT_ALLS | SMC_PMPROT_AVLLS;
#else
	SMC_PMPROT = SMC_PMPROT_AVLP | SMC_PMPROT_ALLS | SMC_PMPROT_AVLLS;
#endif
    
	// TODO: do this while the PLL is waiting to lock....
	while (dest < &_edata) *dest++ = *src++;
	dest = &_sbss;
	while (dest < &_ebss) *dest++ = 0;

	// default all interrupts to medium priority level
	for (i=0; i < NVIC_NUM_INTERRUPTS + 16; i++) _VectorsRam[i] = _VectorsFlash[i];
	for (i=0; i < NVIC_NUM_INTERRUPTS; i++) NVIC_SET_PRIORITY(i, 128);
	SCB_VTOR = (uint32_t)_VectorsRam;	// use vector table in RAM

	// Clocks!
	//
	// Glossary:
	//    MCGOUTCLK: The clock from which is derived the system/core clock,
	//      the bus clock, the FlexBus clock, and the flash clock.
	//      MK64FN1M0VLL12 limit: 120 MHz
	//    Core clock: Clocks the Cortex-M4 core.
    //      MK64FN1M0VLL12 limit: 120 MHz
	//    System clock: Clocks the crossbar switch and the UARTs.
    //      MK64FN1M0VLL12 limit: 120 MHz
	//    Bus clock: Clocks bus slaves, peripherals, and external memory.
    //      MK64FN1M0VLL12 limit: 60 MHz
	//    FlexBus clock: Clocks the external FlexBus interface.
    //      MK64FN1M0VLL12 limit: 50 MHz
	//    Flash clock: Clocks the flash memory.
    //      MK64FN1M0VLL12 limit: 25 MHz

	// hardware always starts in FEI mode (FLL Engaged, Internal)
	//  C1[CLKS] <- 00 (MCGOUTCLK is FLL or PLL output, depending on PLLS bit)
	//  C1[IREFS] <- 1 (FLL source is Slow Internal Reference Clock)
	//  C6[PLLS] <- 0 (FLL is enabled, PLL is disabled)
    //  C2[LP] <- 0 (FLL or PLL is not disabled in bypass modes BLPI and BLPE)

	// In FEI mode, MCGOUTCLK is derived from the FLL clock (MSGFLLCLK, also known as the DCO) that
	// is controlled by the 32 kHz Internal Reference Clock, also called the Slow Internal Reference Clock.
	// The FLL loop will lock the MSGFLLCLK frequency to the FLL factor, as
	// selected by C4[DRST_DRS] and C4[DMX32] bits, times the internal reference frequency. See the
	// C4[DMX32] bit description for more details. In FEI mode, the PLL is disabled in a low-power state
	// unless C5[PLLCLKEN] is set.
	//
    // C4[DMX32] is 0 on reset.
	// C4[DRST_DRS] is 00 on reset ("low gear").
	// Together, these dictate an FLL factor of 640, and a reference clock range of 31.25–39.0625 kHz.
	// That is, MSGFLLCLK ranges from 20-25MHz.

	// From FEI mode, we can "climb" to either FBI (FLL Bypassed, Internal),
	// BLPI (Bypassed Low Power, Internal), or FEE (FLL Engaged, External) mode.

	// In FBI mode, the FLL is running but not used (i.e. bypassed). MCGOUTCLK is derived
	// from either the Slow or the Fast Internal Reference Clock (determined by C2[IRCS]).
	// Since the Fast Internal Reference Clock is 4 MHz, this is a very slow mode.

	// FEE mode:
	//   C1[CLKS] <- 00 (MCGOUTCLK is FLL or PLL output, depending on PLLS bit)
	//   C1[IREFS] <- 0 (FLL source is external clock)
	//   C6[PLLS] <- 0 (FLL is enabled, PLL is disabled)
	//   C2[LP] <- 0 (FLL or PLL is not disabled in bypass modes BLPI and BLPE)

	// In FEE mode, the FLL source becomes the external clock. MCGOUTCLK is derived as in FEI.
	// Note that C1[FRDIV], the FLL's external reference divider, must be set such that
	// the external clock divided by the division factor falls within the range of 31.25–39.0625 kHz.
	// The external clock divided by that factor is called the external reference frequency.
	// For example, with a 16 MHz external clock, the factor must be between 409.6 and 512. The only
	// divider that satisfies this is 512. (RANGE? OSCSEL?)
	// For 50 MHz, the factor must be between 1280 and 1600. The only valid divider is 1280.
	//
	// The FLL loop will lock the DCO frequency to the FLL factor (as selected by
	// C4[DRST_DRS] and C4[DMX32] bits) times the external reference frequency (as specified by
	// C1[FRDIV] and C2[RANGE]).
	//
	// For example, for a 45 MHz external clock, to fall within the range of 31.25–39.0625 kHz requires
	// an external reference divider of between 1152 and 1440. The only valid divider is 1280. Thus,
	// the external reference clock becomes 35.15625 kHz. In "low gear" (C4[DRST_DRS] = 00) this is
	// multiplied by 640, leading to a MCGOUTCLK of 22.5MHz. In "second", "third", and "fourth gears",
	// the multipliers are twice, 3x, and 4x that of first gear, so 45MHz, 67.5MHz, and 90MHz.
	//
	// We can see that at the top end of fourth gear, with a 39.0625 kHz reference, MCGOUTCLK will
	// be 100 MHz. And yet the chip is capable of 120 MHz. If you need higher speed, you need another
	// mode.

#if defined(__FRDM_K64F__)
    // Clocks!
    //
    // Glossary:
    //    MCGOUTCLK: The clock from which is derived the system/core clock,
    //      the bus clock, the FlexBus clock, and the flash clock.
    //      MK64FN1M0VLL12 limit: 120 MHz
    //    Core clock: Clocks the Cortex-M4 core.
    //      MK64FN1M0VLL12 limit: 120 MHz
    //    System clock: Clocks the crossbar switch and the UARTs.
    //      MK64FN1M0VLL12 limit: 120 MHz
    //    Bus clock: Clocks bus slaves, peripherals, and external memory.
    //      MK64FN1M0VLL12 limit: 60 MHz
    //    FlexBus clock: Clocks the external FlexBus interface.
    //      MK64FN1M0VLL12 limit: 50 MHz
    //    Flash clock: Clocks the flash memory.
    //      MK64FN1M0VLL12 limit: 25 MHz
	//
	// Section 5.3 ("High-Level device clocking diagram") of the K64 Sub-Family
	// Reference Manual (document K64P144M120SF5RM) gives a nice overview of the
	// above clocks.
	//
	// According to section 8 of the FRDM-K64F Freedom Module User’s Guide (document FRDMK64FUG),
	// it is important for the MCU clock and the Ethernet PHY chip clock to be synchronized
	// in the Ethernet MAC RMII mode. Since this seems like a good thing, we want to use
	// the clock provided by the PHY chip, which is 50MHz. Thus, we need to switch to an
	// external clock mode.

	// In addition, we want to hit the maximum 120 MHz. We can't do
	// that from any FLL mode: the only valid divider for 50 MHz is 1280, making the external
	// reference clock 39.0625 kHz, and in "fourth gear" the multiplier is 2560 which only
	// gets us to 100 MHz. Only a PLL mode will get us to 120 MHz.
	//
	// From reset, the processor always starts in FEI mode (FLL Engaged, Internal). These
	// are the starting MCG register settings that define FEI mode:
    //   C1[CLKS] <- 00 (MCGOUTCLK is FLL or PLL output, depending on PLLS bit)
    //   C1[IREFS] <- 1 (FLL source is Slow Internal Reference Clock)
    //   C6[PLLS] <- 0 (FLL is enabled, PLL is disabled)
    //   C2[LP] <- 0 (FLL or PLL is not disabled in bypass modes BLPI and BLPE)

	//
	// Thus, we need to enter FBE mode in order to transition to the PLL modes.
	//
	// According to Table 25-16 of the K64 Sub-Family Reference Manual
	// (document K64P144M120SF5RM), to enter FBE mode, we need:
    //   C1[CLKS] <- 10 (MCGOUTCLK is the external reference clock)
    //   C1[IREFS] <- 0 (FLL source is external clock)
    //   C6[PLLS] <- 0 (FLL is enabled, PLL is disabled)
    //   C2[LP] <- 0 (FLL or PLL is not disabled in bypass modes BLPI and BLPE)
	// (Note that C2[LP] doesn't need to be changed from FEI mode).
	//
	// Note also that because we are using an external clock rather than a crystal,
	//   C2[EREFS] <- 0, which is the reset value anyway.
	//
	// We also need to set C1[FRDIV] such that the 50 MHz external clock divided by that
	// factor falls within the range of 31.25–39.0625 kHz. The only divider which does that
	// is 1280, which means:
	//   C1[FRDIV] <- 110
	//
	// This yields an external *reference* clock of 50 MHz / 1280 = 39.0625 kHz. Note that
	// this divider is only valid if C7[OSCSEL] is not 1 (it is 0 at reset) and
	// C2[RANGE] is not 0 (but it is 0 at reset). So we must also change C2[RANGE]. The
	// actual value doesn't seem to matter.
	//   C2[RANGE] <- 01

	MCG_C1 = MCG_C1_CLKS(2) | MCG_C1_FRDIV(6);
	MCG_C2 = MCG_C2_RANGE0(3);

	// Now we must wait for S[IREFST] to change to zero. Technically we don't have to
	// do this, since C1[IREFS] was already zero, but let's do this anyway.

    while (MCG_S & MCG_S_IREFST) { }

    // And now we have to wait for S[CLKS] to change to 10.

    while ((MCG_S & MCG_S_CLKST_MASK) != MCG_S_CLKST(2)) { }

    // We are now allowed to set C4[DRST_DRS] and C4[DMX32]. We will choose:
    //   C4[DMX32] = 0 (reference range is not restricted)
    //   C4[DRST_DRS] = 11 ("fourth gear", multiplier is 2560)
    //
    // This yields a MCGOUTCLK of 39.0625 kHz * 2560 = 100 MHz.

    MCG_C4 = MCG_C4_DRST_DRS(3);

    // Now we must wait for the FLL to lock. This involves waiting for at least
    // 1 msec (Table 17, t(fll_acquire) of Kinetis K64F Sub-Family Data Sheet
    // (document K64P144M120SF5). At 100 MHz, that's 100kcycles. There are seven
    // instructions per loop iteration when I checked the disassembly, and there
    // is a minimum of 2 machine cycles per instruction, which gives us an upper
    // bound of 14 cycles per iteration. Thus, we need about 7200 iterations.

    {
        volatile int c;
        for (c = 0; c < 7200; c++) { }
    }

    // We are now in FBE mode. To go faster, we need to get to PBE mode. In
    // this mode, we set up the PLL to output the frequency we want, but we
    // do not yet use the output. This mode allows the PLL to lock its
    // frequency in. Once that is done, we can go to PEE, which "takes it
    // out of neutral" and applies the PLL output to MCGOUTCLK.
    //
    // The only difference between FBE and PBE mode is that
    //   C6[PLLS] <- 1 (FLL is disabled, PLL is enabled)
    //
    // We must first, however, set the PLL's divider and multiplier to appropriate
    // values. The divider is set by C5[PRDIV0], and must divide down
    // the external clock to be within the range 2 - 4 MHz. Dividers are
    // anything from 1 to 25, so for 50 MHz any divider between 13 and 25
    // will do.
    //
    // This reference frequency ends up being multiplied by the multiplier
    // C6[VDIV0], which can be anything from 24 to 55. The result is the
    // MCGOUTCLK.
    //
    // We will choose a divider of 15 (i.e. a reference clock of
    // 50 / 15 = 3 1/3 MHz) and a multiplier of 36 (so MCGOUTCLK = 120 MHz).
    //   C5[PRDIV0] <- 01110
    //   C6[VDIV0] <- 01100

    MCG_C5 = MCG_C5_PRDIV0(14);
    MCG_C6 = MCG_C6_PLLS | MCG_C6_VDIV0(12);

    // We must read S[PLLS] to ensure the state has truly changed.

    while (!(MCG_S & MCG_S_PLLST)) { }

    // And now we wait for the MCG to tell us the PLL has locked
    // in to its frequency via S[LOCK0] being set.

    while (!(MCG_S & MCG_S_LOCK0)) { }

    // And now we are in PBE mode, and we have a good 120 MHz lock for the PLL.
    // We now want to make the final transition to PEE mode. Before we do so,
    // we need to set some dividers. From MCGOUTCLK is
    // derived these other clocks: system, core, bus, FlexBus, flash. These
    // are set via dividers. Let's just set them to their maximums. Looking again
    // at section 5.3 ("High-Level device clocking diagram") of the K64 Sub-Family
    // Reference Manual (document K64P144M120SF5RM), we see that we need to get
    // into the SIM ("System Integration Module") and set the four dividers
    // via CLKDIV1[OUTDIV1] through CLKDIV1[OUTDIV4].
    //   CLKDIV1[OUTDIV1] <- 0 (system/core clock divide by 1 = 120 MHz)
    //   CLKDIV1[OUTDIV2] <- 1 (bus clock divide by 2 = 60 MHz)
    //   CLKDIV1[OUTDIV3] <- 2 (FlexBus clock divide by 3 = 40 MHz, not the fastest)
    //   CLKDIV1[OUTDIV4] <- 4 (flash clock divide by 5 = 24 MHz, close to the top)

    SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIV1(0) | SIM_CLKDIV1_OUTDIV2(1)
            | SIM_CLKDIV1_OUTDIV3(2) | SIM_CLKDIV1_OUTDIV4(4);

    // Now we can go to PEE mode. To do this:
    //   C1[CLKS] <- 00 (MCGOUTCLK is FLL or PLL output, depending on PLLS bit)

    MCG_C1 = MCG_C1_FRDIV(6); // Let's not disturb the other settings

    // And we wait for S[CLKST] to tell us that the PLL output has been
    // routed to MCGOUTCLK:

    while ((MCG_S & MCG_S_CLKST_MASK) != MCG_S_CLKST(3)) { }

    // And we are in PEE, running at full speed. We prove that we can still
    // do stuff by lighting the green LED.

    // Set up port E pin 26 (connected to the green LED) to be a GPIO. It
    // has high drive strength (DSE) and slow slew rate (SRE).
    //PORTE_PCR26 = PORT_PCR_MUX(1) | PORT_PCR_DSE | PORT_PCR_SRE;

    // Set GPIO E pin 26 to be an output

    //GPIOE_PDDR |= (1<<26);

    // Set output to 0 so LED lights up.

    //GPIOE_PCOR = (1<<26);

    // According to the Kinetis Quick Reference User Guide (also known as
    // Kinetis Peripheral Module Quick Reference or Demonstration software for Kinetis
    // peripheral module, all document KQRUG), USB requires a clock
    // frequency of 48MHz. There is a multiplexer
    // which determines the base source of the clock (FLL, PLL, or external),
    // SIM[SOPT2][USBSRC] and SIM[SOPT2][PLLFLLSEL]. You can also select
    // an internal 48MHz clock, which suspiciously the Kinetis Quick Reference User Guide
    // does not mention, but the K64 Sub-Family Reference Manual does. There is a
    // multiplier/divider combination, SIM[CLKDIV2][USBFRAC] and [USBDIV], which determines
    // the actual USB clock from the base source if the source is FLL or PLL.
    //
    // See also Crystal-less USB operation on Kinetis MCUs (AN4905)
    //
    // Since we have set the PLL frequency to 120 MHz, we would need a factor of 0.4 to get to
    // 48 MHz. The output clock = input clock * (1 + USBFRAC)/(1 + USBDIV), where
    // USBFRAC is 0-7 and USBDIV is 0 or 1. Clearly no value will get us to 0.4.
    // So, we will just try to use the internal 48 MHz clock that the K64 reference manual
    // claims we have.
    // First we enable the internal 48 MHz clock for USB:
    //   USB0_CLK_RECOVER_IRC_EN[IRC_EN] <- 1
    //   SIM[SOPT2][PLLFLLSEL] <- 11 (internal 48 MHz clock)
    //   SIM[SOPT2][USBSRC] <- 1 (not external)
    // And we also have to allow this clock to get to the USB module:
    //   SIM[SCGC4][USBOTG] <- 1 (enable)
    // Note that last setting is done already in usb_init.

    // TRACECLKSEL <- 1 (trace debug clock is core clock)
    // CLKOUTSEL <- 110 (CLKOUT pin is OSCERCLK2)
    //USB0_CLK_RECOVER_IRC_EN = USB_CLK_RECOVER_IRC_EN_IRC_EN | USB_CLK_RECOVER_IRC_EN_REG_EN;
    // USB0_CLK_RECOVER_CTRL = USB_CLK_RECOVER_CTRL_CLOCK_RECOVER_EN | USB_CLK_RECOVER_CTRL_RESTART_IFRTRIM_EN;
    SIM_SOPT2 = SIM_SOPT2_USBSRC | SIM_SOPT2_IRC48SEL | SIM_SOPT2_TRACECLKSEL | SIM_SOPT2_CLKOUTSEL(6);

#else //defined(__FRDM_K64F__)

// MCG_SC[FCDIV] defaults to divide by two for internal ref clock
// I tried changing MSG_SC to divide by 1, it didn't work for me
#if F_CPU <= 2000000
    #if defined(KINETISK)
    MCG_C1 = MCG_C1_CLKS(1) | MCG_C1_IREFS;
    #elif defined(KINETISL)
	// use the internal oscillator
	MCG_C1 = MCG_C1_CLKS(1) | MCG_C1_IREFS | MCG_C1_IRCLKEN;
    #endif
	// wait for MCGOUT to use oscillator
	while ((MCG_S & MCG_S_CLKST_MASK) != MCG_S_CLKST(1)) ;
	for (n=0; n<10; n++) ; // TODO: why do we get 2 mA extra without this delay?
	MCG_C2 = MCG_C2_IRCS;
	while (!(MCG_S & MCG_S_IRCST)) ;
	// now in FBI mode:
	//  C1[CLKS] bits are written to 01
	//  C1[IREFS] bit is written to 1
	//  C6[PLLS] is written to 0
	//  C2[LP] is written to 0
	MCG_C2 = MCG_C2_IRCS | MCG_C2_LP;
	// now in BLPI mode:
	//  C1[CLKS] bits are written to 01
	//  C1[IREFS] bit is written to 1
	//  C6[PLLS] bit is written to 0
	//  C2[LP] bit is written to 1
#else
    #if defined(KINETISK)
    // enable capacitors for crystal
    OSC0_CR = OSC_SC8P | OSC_SC2P | OSC_ERCLKEN;
    #elif defined(KINETISL)
    // enable capacitors for crystal
    OSC0_CR = OSC_SC8P | OSC_SC2P | OSC_ERCLKEN;
    #endif
	// enable osc, 8-32 MHz range, low power mode
	MCG_C2 = MCG_C2_RANGE0(2) | MCG_C2_EREFS;
	// switch to crystal as clock source, FLL input = 16 MHz / 512
	MCG_C1 =  MCG_C1_CLKS(2) | MCG_C1_FRDIV(4);
	// wait for crystal oscillator to begin
	while ((MCG_S & MCG_S_OSCINIT0) == 0) ;
	// wait for FLL to use oscillator
	while ((MCG_S & MCG_S_IREFST) != 0) ;
	// wait for MCGOUT to use oscillator
	while ((MCG_S & MCG_S_CLKST_MASK) != MCG_S_CLKST(2)) ;

	// now in FBE mode
	//  C1[CLKS] bits are written to 10
	//  C1[IREFS] bit is written to 0
	//  C1[FRDIV] must be written to divide xtal to 31.25-39 kHz
	//  C6[PLLS] bit is written to 0
	//  C2[LP] is written to 0
  #if F_CPU <= 16000000
	// if the crystal is fast enough, use it directly (no FLL or PLL)
	MCG_C2 = MCG_C2_RANGE0(2) | MCG_C2_EREFS | MCG_C2_LP;
	// BLPE mode:
	//   C1[CLKS] bits are written to 10
	//   C1[IREFS] bit is written to 0
	//   C2[LP] bit is written to 1
  #else
	// if we need faster than the crystal, turn on the PLL
   #if defined(__MK66FX1M0__)
    #if F_CPU > 120000000
	SMC_PMCTRL = SMC_PMCTRL_RUNM(3); // enter HSRUN mode
	while (SMC_PMSTAT != SMC_PMSTAT_HSRUN) ; // wait for HSRUN
    #endif
    #if F_CPU == 240000000
	MCG_C5 = MCG_C5_PRDIV0(0);
	MCG_C6 = MCG_C6_PLLS | MCG_C6_VDIV0(14);
    #elif F_CPU == 216000000
	MCG_C5 = MCG_C5_PRDIV0(0);
	MCG_C6 = MCG_C6_PLLS | MCG_C6_VDIV0(11);
    #elif F_CPU == 192000000
	MCG_C5 = MCG_C5_PRDIV0(0);
	MCG_C6 = MCG_C6_PLLS | MCG_C6_VDIV0(8);
    #elif F_CPU == 180000000
	MCG_C5 = MCG_C5_PRDIV0(1);
	MCG_C6 = MCG_C6_PLLS | MCG_C6_VDIV0(29);
    #elif F_CPU == 168000000
	MCG_C5 = MCG_C5_PRDIV0(0);
	MCG_C6 = MCG_C6_PLLS | MCG_C6_VDIV0(5);
    #elif F_CPU == 144000000
	MCG_C5 = MCG_C5_PRDIV0(0);
	MCG_C6 = MCG_C6_PLLS | MCG_C6_VDIV0(2);
    #elif F_CPU == 120000000
	MCG_C5 = MCG_C5_PRDIV0(1);
	MCG_C6 = MCG_C6_PLLS | MCG_C6_VDIV0(14);
    #elif F_CPU == 96000000 || F_CPU == 48000000 || F_CPU == 24000000
	MCG_C5 = MCG_C5_PRDIV0(1);
	MCG_C6 = MCG_C6_PLLS | MCG_C6_VDIV0(8);
    #elif F_CPU == 72000000
	MCG_C5 = MCG_C5_PRDIV0(1);
	MCG_C6 = MCG_C6_PLLS | MCG_C6_VDIV0(2);
    #elif F_CPU > 16000000
    #error "MK66FX1M0 does not support this clock speed yet...."
    #endif
   #else
    #if F_CPU == 72000000
	MCG_C5 = MCG_C5_PRDIV0(5);		 // config PLL input for 16 MHz Crystal / 6 = 2.667 Hz
    #else
	MCG_C5 = MCG_C5_PRDIV0(3);		 // config PLL input for 16 MHz Crystal / 4 = 4 MHz
    #endif
    #if F_CPU == 168000000
	MCG_C6 = MCG_C6_PLLS | MCG_C6_VDIV0(18); // config PLL for 168 MHz output
    #elif F_CPU == 144000000
	MCG_C6 = MCG_C6_PLLS | MCG_C6_VDIV0(12); // config PLL for 144 MHz output
    #elif F_CPU == 120000000
	MCG_C6 = MCG_C6_PLLS | MCG_C6_VDIV0(6); // config PLL for 120 MHz output
    #elif F_CPU == 72000000
	MCG_C6 = MCG_C6_PLLS | MCG_C6_VDIV0(3); // config PLL for 72 MHz output
    #elif F_CPU == 96000000 || F_CPU == 48000000 || F_CPU == 24000000
	MCG_C6 = MCG_C6_PLLS | MCG_C6_VDIV0(0); // config PLL for 96 MHz output
    #elif F_CPU > 16000000
    #error "This clock speed isn't supported..."
    #endif
   #endif

	// wait for PLL to start using xtal as its input
	while (!(MCG_S & MCG_S_PLLST)) ;
	// wait for PLL to lock
	while (!(MCG_S & MCG_S_LOCK0)) ;
	// now we're in PBE mode
  #endif
#endif
	// now program the clock dividers
#if F_CPU == 240000000
	// config divisors: 240 MHz core, 60 MHz bus, 30 MHz flash, USB = 240 / 5
	// TODO: gradual ramp-up for HSRUN mode
	#if F_BUS == 60000000
	SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIV1(0) | SIM_CLKDIV1_OUTDIV2(3) | SIM_CLKDIV1_OUTDIV4(7);
	#elif F_BUS == 80000000
	SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIV1(0) | SIM_CLKDIV1_OUTDIV2(2) | SIM_CLKDIV1_OUTDIV4(7);
	#elif F_BUS == 120000000
	SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIV1(0) | SIM_CLKDIV1_OUTDIV2(1) | SIM_CLKDIV1_OUTDIV4(7);
	#else
	#error "This F_CPU & F_BUS combination is not supported"
	#endif
	SIM_CLKDIV2 = SIM_CLKDIV2_USBDIV(4);
#elif F_CPU == 216000000
	// config divisors: 216 MHz core, 54 MHz bus, 27 MHz flash, USB = IRC48M
	// TODO: gradual ramp-up for HSRUN mode
	#if F_BUS == 54000000
	SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIV1(0) | SIM_CLKDIV1_OUTDIV2(3) | SIM_CLKDIV1_OUTDIV4(7);
	#elif F_BUS == 72000000
	SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIV1(0) | SIM_CLKDIV1_OUTDIV2(2) | SIM_CLKDIV1_OUTDIV4(7);
	#elif F_BUS == 108000000
	SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIV1(0) | SIM_CLKDIV1_OUTDIV2(1) | SIM_CLKDIV1_OUTDIV4(7);
	#else
	#error "This F_CPU & F_BUS combination is not supported"
	#endif
	SIM_CLKDIV2 = SIM_CLKDIV2_USBDIV(0);
#elif F_CPU == 192000000
	// config divisors: 192 MHz core, 48 MHz bus, 27.4 MHz flash, USB = 192 / 4
	// TODO: gradual ramp-up for HSRUN mode
	#if F_BUS == 48000000
	SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIV1(0) | SIM_CLKDIV1_OUTDIV2(3) | SIM_CLKDIV1_OUTDIV4(6);
	#elif F_BUS == 64000000
	SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIV1(0) | SIM_CLKDIV1_OUTDIV2(2) | SIM_CLKDIV1_OUTDIV4(6);
	#elif F_BUS == 96000000
	SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIV1(0) | SIM_CLKDIV1_OUTDIV2(1) | SIM_CLKDIV1_OUTDIV4(6);
	#else
	#error "This F_CPU & F_BUS combination is not supported"
	#endif
	SIM_CLKDIV2 = SIM_CLKDIV2_USBDIV(3);
#elif F_CPU == 180000000
	// config divisors: 180 MHz core, 60 MHz bus, 25.7 MHz flash, USB = IRC48M
	#if F_BUS == 60000000
	SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIV1(0) | SIM_CLKDIV1_OUTDIV2(2) | SIM_CLKDIV1_OUTDIV4(6);
	#elif F_BUS == 90000000
	SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIV1(0) | SIM_CLKDIV1_OUTDIV2(1) | SIM_CLKDIV1_OUTDIV4(6);
	#else
	#error "This F_CPU & F_BUS combination is not supported"
	#endif
	SIM_CLKDIV2 = SIM_CLKDIV2_USBDIV(0);
#elif F_CPU == 168000000
	// config divisors: 168 MHz core, 56 MHz bus, 28 MHz flash, USB = 168 * 2 / 7
	SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIV1(0) | SIM_CLKDIV1_OUTDIV2(2) | SIM_CLKDIV1_OUTDIV4(5);
	SIM_CLKDIV2 = SIM_CLKDIV2_USBDIV(6) | SIM_CLKDIV2_USBFRAC;
#elif F_CPU == 144000000
	// config divisors: 144 MHz core, 48 MHz bus, 28.8 MHz flash, USB = 144 / 3
	#if F_BUS == 48000000
	SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIV1(0) | SIM_CLKDIV1_OUTDIV2(2) | SIM_CLKDIV1_OUTDIV4(4);
	#elif F_BUS == 72000000
	SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIV1(0) | SIM_CLKDIV1_OUTDIV2(1) | SIM_CLKDIV1_OUTDIV4(4);
	#else
	#error "This F_CPU & F_BUS combination is not supported"
	#endif
	SIM_CLKDIV2 = SIM_CLKDIV2_USBDIV(2);
#elif F_CPU == 120000000
	// config divisors: 120 MHz core, 60 MHz bus, 24 MHz flash, USB = 128 * 2 / 5
	#if F_BUS == 60000000
	SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIV1(0) | SIM_CLKDIV1_OUTDIV2(1) | SIM_CLKDIV1_OUTDIV4(4);
	#elif F_BUS == 120000000
	SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIV1(0) | SIM_CLKDIV1_OUTDIV2(0) | SIM_CLKDIV1_OUTDIV4(4);
	#else
	#error "This F_CPU & F_BUS combination is not supported"
	#endif
	SIM_CLKDIV2 = SIM_CLKDIV2_USBDIV(4) | SIM_CLKDIV2_USBFRAC;
#elif F_CPU == 96000000
	// config divisors: 96 MHz core, 48 MHz bus, 24 MHz flash, USB = 96 / 2
	#if F_BUS == 48000000
	SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIV1(0) | SIM_CLKDIV1_OUTDIV2(1) | SIM_CLKDIV1_OUTDIV4(3);
	#elif F_BUS == 96000000
	SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIV1(0) | SIM_CLKDIV1_OUTDIV2(0) | SIM_CLKDIV1_OUTDIV4(3);
	#else
	#error "This F_CPU & F_BUS combination is not supported"
	#endif
	SIM_CLKDIV2 = SIM_CLKDIV2_USBDIV(1);
#elif F_CPU == 72000000
	// config divisors: 72 MHz core, 36 MHz bus, 24 MHz flash, USB = 72 * 2 / 3
	#if F_BUS == 36000000
	SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIV1(0) | SIM_CLKDIV1_OUTDIV2(1) | SIM_CLKDIV1_OUTDIV4(2);
	#elif F_BUS == 72000000
	SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIV1(0) | SIM_CLKDIV1_OUTDIV2(0) | SIM_CLKDIV1_OUTDIV4(2);
	#else
	#error "This F_CPU & F_BUS combination is not supported"
	#endif
	SIM_CLKDIV2 = SIM_CLKDIV2_USBDIV(2) | SIM_CLKDIV2_USBFRAC;
#elif F_CPU == 48000000
	// config divisors: 48 MHz core, 48 MHz bus, 24 MHz flash, USB = 96 / 2
  #if defined(KINETISK)
	SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIV1(1) | SIM_CLKDIV1_OUTDIV2(1) | SIM_CLKDIV1_OUTDIV3(1) |  SIM_CLKDIV1_OUTDIV4(3);
	SIM_CLKDIV2 = SIM_CLKDIV2_USBDIV(1);
  #elif defined(KINETISL)
	SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIV1(1) | SIM_CLKDIV1_OUTDIV4(1);
  #endif
#elif F_CPU == 24000000
	// config divisors: 24 MHz core, 24 MHz bus, 24 MHz flash, USB = 96 / 2
	#if defined(KINETISK)
	SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIV1(3) | SIM_CLKDIV1_OUTDIV2(3) | SIM_CLKDIV1_OUTDIV3(3) | SIM_CLKDIV1_OUTDIV4(3);
	SIM_CLKDIV2 = SIM_CLKDIV2_USBDIV(1);
	#elif defined(KINETISL)
	SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIV1(3) | SIM_CLKDIV1_OUTDIV4(0);
	#endif
#elif F_CPU == 16000000
	// config divisors: 16 MHz core, 16 MHz bus, 16 MHz flash
  #if defined(KINETISK)
	SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIV1(0) | SIM_CLKDIV1_OUTDIV2(0) | SIM_CLKDIV1_OUTDIV3(0) | SIM_CLKDIV1_OUTDIV4(0);
  #elif defined(KINETISL)
	SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIV1(0) | SIM_CLKDIV1_OUTDIV4(0);
  #endif
#elif F_CPU == 8000000
	// config divisors: 8 MHz core, 8 MHz bus, 8 MHz flash
  #if defined(KINETISK)
	SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIV1(1) | SIM_CLKDIV1_OUTDIV2(1) | SIM_CLKDIV1_OUTDIV3(1) | SIM_CLKDIV1_OUTDIV4(1);
  #elif defined(KINETISL)
	SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIV1(1) | SIM_CLKDIV1_OUTDIV4(0);
  #endif
#elif F_CPU == 4000000
	// config divisors: 4 MHz core, 4 MHz bus, 2 MHz flash
	// since we are running from external clock 16MHz
	// fix outdiv too -> cpu 16/4, bus 16/4, flash 16/4
	// here we can go into vlpr?
	// config divisors: 4 MHz core, 4 MHz bus, 4 MHz flash
  #if defined(KINETISK)
	SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIV1(3) | SIM_CLKDIV1_OUTDIV2(3) | SIM_CLKDIV1_OUTDIV3(3) | SIM_CLKDIV1_OUTDIV4(3);
  #elif defined(KINETISL)
	SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIV1(3) | SIM_CLKDIV1_OUTDIV4(0);
  #endif
#elif F_CPU == 2000000
	// since we are running from the fast internal reference clock 4MHz
	// but is divided down by 2 so we actually have a 2MHz, MCG_SC[FCDIV] default is 2
	// fix outdiv -> cpu 2/1, bus 2/1, flash 2/2
	// config divisors: 2 MHz core, 2 MHz bus, 1 MHz flash
  #if defined(KINETISK)
	SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIV1(0) | SIM_CLKDIV1_OUTDIV2(0) | SIM_CLKDIV1_OUTDIV4(1);
  #elif defined(KINETISL)
	// config divisors: 2 MHz core, 1 MHz bus, 1 MHz flash
	SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIV1(0) | SIM_CLKDIV1_OUTDIV4(1);
  #endif
#else
#error "Error, F_CPU must be 192, 180, 168, 144, 120, 96, 72, 48, 24, 16, 8, 4, or 2 MHz"
#endif

#if F_CPU > 16000000
	// switch to PLL as clock source, FLL input = 16 MHz / 512
	MCG_C1 = MCG_C1_CLKS(0) | MCG_C1_FRDIV(4);
	// wait for PLL clock to be used
	while ((MCG_S & MCG_S_CLKST_MASK) != MCG_S_CLKST(3)) ;
	// now we're in PEE mode
	// USB uses PLL clock, trace is CPU clock, CLKOUT=OSCERCLK0
	#if defined(KINETISK)
	#if F_CPU == 216000000 || F_CPU == 180000000
	SIM_SOPT2 = SIM_SOPT2_USBSRC | SIM_SOPT2_IRC48SEL | SIM_SOPT2_TRACECLKSEL | SIM_SOPT2_CLKOUTSEL(6);
	#else
	SIM_SOPT2 = SIM_SOPT2_USBSRC | SIM_SOPT2_PLLFLLSEL | SIM_SOPT2_TRACECLKSEL | SIM_SOPT2_CLKOUTSEL(6);
	#endif
	#elif defined(KINETISL)
	SIM_SOPT2 = SIM_SOPT2_USBSRC | SIM_SOPT2_PLLFLLSEL | SIM_SOPT2_CLKOUTSEL(6)
		| SIM_SOPT2_UART0SRC(1) | SIM_SOPT2_TPMSRC(1);
	#endif
#else
    
#if F_CPU == 2000000
	SIM_SOPT2 = SIM_SOPT2_TRACECLKSEL | SIM_SOPT2_CLKOUTSEL(4) | SIM_SOPT2_UART0SRC(3);
#else
    SIM_SOPT2 = SIM_SOPT2_TRACECLKSEL | SIM_SOPT2_CLKOUTSEL(6) | SIM_SOPT2_UART0SRC(2);
#endif
    
#endif

#if F_CPU <= 2000000
    // since we are not going into "stop mode" i removed it
	SMC_PMCTRL = SMC_PMCTRL_RUNM(2); // VLPR mode :-)
#endif

#endif //defined(__FRDM_K64F__)

	// initialize the SysTick counter
	SYST_RVR = (F_CPU / 1000) - 1;
	SYST_CVR = 0;
	SYST_CSR = SYST_CSR_CLKSOURCE | SYST_CSR_TICKINT | SYST_CSR_ENABLE;
    SCB_SHPR3 = 0x20200000;  // Systick, PendSV interrupts = priority 32
    // SCB_SHPR3 |= 0x00200000;  // Systick interrupts = priority 32

	//init_pins();
	__enable_irq();

	_init_Teensyduino_internal_();
    //PORTE_PCR26 = PORT_PCR_MUX(1) | PORT_PCR_DSE | PORT_PCR_SRE;
    //GPIOE_PDDR |= (1<<26);
    //GPIOE_PCOR = (1<<26);
    //while(1) { }

#if defined(KINETISK)
	// RTC initialization
	if (RTC_SR & RTC_SR_TIF) {
		// this code will normally run on a power-up reset
		// when VBAT has detected a power-up.  Normally our
		// compiled-in time will be stale.  Write a special
		// flag into the VBAT register file indicating the
		// RTC is set with known-stale time and should be
		// updated when fresh time is known.
		#if ARDUINO >= 10600
		rtc_set((uint32_t)&__rtc_localtime);
		#else
		rtc_set(TIME_T);
		#endif
		*(uint32_t *)0x4003E01C = 0x5A94C3A5;
	}
	if ((RCM_SRS0 & RCM_SRS0_PIN) && (*(uint32_t *)0x4003E01C == 0x5A94C3A5)) {
		// this code should run immediately after an upload
		// where the Teensy Loader causes the Mini54 to reset.
		// Our compiled-in time will be very fresh, so set
		// the RTC with this, and clear the VBAT resister file
		// data so we don't mess with the time after it's been
		// set well.
		#if ARDUINO >= 10600
		rtc_set((uint32_t)&__rtc_localtime);
		#else
		rtc_set(TIME_T);
		#endif
		*(uint32_t *)0x4003E01C = 0;
	}
#endif

	__libc_init_array();

	startup_late_hook();
	main();
	while (1) ;
}

char *__brkval = (char *)&_ebss;

void * _sbrk(int incr)
{
	char *prev = __brkval;
	__brkval += incr;
	return prev;
}

__attribute__((weak)) 
int _read(int file, char *ptr, int len)
{
	return 0;
}

__attribute__((weak)) 
int _close(int fd)
{
	return -1;
}

#include <sys/stat.h>

__attribute__((weak)) 
int _fstat(int fd, struct stat *st)
{
	st->st_mode = S_IFCHR;
	return 0;
}

__attribute__((weak)) 
int _isatty(int fd)
{
	return 1;
}

__attribute__((weak)) 
int _lseek(int fd, long long offset, int whence)
{
	return -1;
}

__attribute__((weak)) 
void _exit(int status)
{
	while (1);
}

__attribute__((weak)) 
void __cxa_pure_virtual()
{
	while (1);
}

__attribute__((weak)) 
int __cxa_guard_acquire (char *g) 
{
	return !(*g);
}

__attribute__((weak)) 
void __cxa_guard_release(char *g)
{
	*g = 1;
}

int nvic_execution_priority(void)
{
	int priority=256;
	uint32_t primask, faultmask, basepri, ipsr;

	// full algorithm in ARM DDI0403D, page B1-639
	// this isn't quite complete, but hopefully good enough
	__asm__ volatile("mrs %0, faultmask\n" : "=r" (faultmask)::);
	if (faultmask) return -1;
	__asm__ volatile("mrs %0, primask\n" : "=r" (primask)::);
	if (primask) return 0;
	__asm__ volatile("mrs %0, ipsr\n" : "=r" (ipsr)::);
	if (ipsr) {
		if (ipsr < 16) priority = 0; // could be non-zero
		else priority = NVIC_GET_PRIORITY(ipsr - 16);
	}
	__asm__ volatile("mrs %0, basepri\n" : "=r" (basepri)::);
	if (basepri > 0 && basepri < priority) priority = basepri;
	return priority;
}


#if defined(HAS_KINETIS_HSRUN) && F_CPU > 120000000
int kinetis_hsrun_disable(void)
{
	if (SMC_PMSTAT == SMC_PMSTAT_HSRUN) {
		// First, reduce the CPU clock speed, but do not change
		// the peripheral speed (F_BUS).  Serial1 & Serial2 baud
		// rates will be impacted, but most other peripherals
		// will continue functioning at the same speed.
		#if F_CPU == 240000000 && F_BUS == 60000000
			SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIVS(1, 3, 1, 7); // ok
		#elif F_CPU == 240000000 && F_BUS == 80000000
			SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIVS(2, 2, 2, 8); // ok
		#elif F_CPU == 240000000 && F_BUS == 120000000
			SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIVS(1, 1, 1, 7); // ok
		#elif F_CPU == 216000000 && F_BUS == 54000000
			SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIVS(1, 3, 1, 7); // ok
		#elif F_CPU == 216000000 && F_BUS == 72000000
			SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIVS(2, 2, 2, 8); // ok
		#elif F_CPU == 216000000 && F_BUS == 108000000
			SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIVS(1, 1, 1, 7); // ok
		#elif F_CPU == 192000000 && F_BUS == 48000000
			SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIVS(1, 3, 1, 7); // ok
		#elif F_CPU == 192000000 && F_BUS == 64000000
			SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIVS(2, 2, 2, 8); // ok
		#elif F_CPU == 192000000 && F_BUS == 96000000
			SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIVS(1, 1, 1, 7); // ok
		#elif F_CPU == 180000000 && F_BUS == 60000000
			SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIVS(2, 2, 2, 8); // ok
		#elif F_CPU == 180000000 && F_BUS == 90000000
			SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIVS(1, 1, 1, 7); // ok
		#elif F_CPU == 168000000 && F_BUS == 56000000
			SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIVS(2, 2, 2, 5); // ok
		#elif F_CPU == 144000000 && F_BUS == 48000000
			SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIVS(2, 2, 2, 5); // ok
		#elif F_CPU == 144000000 && F_BUS == 72000000
			SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIVS(1, 1, 1, 5); // ok
		#else
			return 0;
		#endif
		// Then turn off HSRUN mode
		SMC_PMCTRL = SMC_PMCTRL_RUNM(0);
		while (SMC_PMSTAT == SMC_PMSTAT_HSRUN) ; // wait
		return 1;
	}
	return 0;
}

int kinetis_hsrun_enable(void)
{
	if (SMC_PMSTAT == SMC_PMSTAT_RUN) {
		// Turn HSRUN mode on
		SMC_PMCTRL = SMC_PMCTRL_RUNM(3);
		while (SMC_PMSTAT != SMC_PMSTAT_HSRUN) ; // wait
		// Then configure clock for full speed
		#if F_CPU == 240000000 && F_BUS == 60000000
			SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIVS(0, 3, 0, 7);
		#elif F_CPU == 240000000 && F_BUS == 80000000
			SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIVS(0, 2, 0, 7);
		#elif F_CPU == 240000000 && F_BUS == 120000000
			SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIVS(0, 1, 0, 7);
		#elif F_CPU == 216000000 && F_BUS == 54000000
			SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIVS(0, 3, 0, 7);
		#elif F_CPU == 216000000 && F_BUS == 72000000
			SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIVS(0, 2, 0, 7);
		#elif F_CPU == 216000000 && F_BUS == 108000000
			SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIVS(0, 1, 0, 7);
		#elif F_CPU == 192000000 && F_BUS == 48000000
			SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIVS(0, 3, 0, 6);
		#elif F_CPU == 192000000 && F_BUS == 64000000
			SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIVS(0, 2, 0, 6);
		#elif F_CPU == 192000000 && F_BUS == 96000000
			SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIVS(0, 1, 0, 6);
		#elif F_CPU == 180000000 && F_BUS == 60000000
			SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIVS(0, 2, 0, 6);
		#elif F_CPU == 180000000 && F_BUS == 90000000
			SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIVS(0, 1, 0, 6);
		#elif F_CPU == 168000000 && F_BUS == 56000000
			SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIVS(0, 2, 0, 5);
		#elif F_CPU == 144000000 && F_BUS == 48000000
			SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIVS(0, 2, 0, 4);
		#elif F_CPU == 144000000 && F_BUS == 72000000
			SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIVS(0, 1, 0, 4);
		#else
			return 0;
		#endif
		return 1;
	}
	return 0;
}
#endif // HAS_KINETIS_HSRUN && F_CPU > 120000000

