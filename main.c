#include "FinalProjectHeader.h"

int clockFrequency = 16000000;

double distanceThreshold = 25.0; // Minimum clearence distance between a sensor and obstacles in centimeters.
volatile double distance3 = 100.0; // Distance between Sensor 3 and obstacles in centimeters.
volatile double distance2 = 100.0; // Distance between Sensor 2 and obstacles in centimeters.
volatile double distance1 = 100.0; // Distance between Sensor 1 and obstacles in centimeters.
volatile double distance0 = 100.0; // Distance between Sensor 0 and obstacles in centimeters.

volatile char receivedData = SIGNAL_MIDDLE;

volatile short currentSensor = 0;
volatile int startTime = 0;

int main()
{
	initializePortA();
	initializePortC();
	initializePortE();
	initializeTimer0();
	initializeUART0();
	initializeUART1();
	while (1)
	{
		if (distance0 >= distanceThreshold && distance1 >= distanceThreshold && distance2 >= distanceThreshold)
		{
			if (receivedData == SIGNAL_LEFT)
			{
				turnLeft();
			}
			else if (receivedData == SIGNAL_RIGHT)
			{
				turnRight();
			}
			else
			{
				moveForward();
			}
		}
		else if (distance0 < distanceThreshold && distance2 >= distanceThreshold)
		{
			turnRight();
		}
		else if (distance0 >= distanceThreshold && distance2 < distanceThreshold)
		{
			turnLeft();
		}
		else if (distance3 >= distanceThreshold)
		{
			moveBackward();
		}
		else
		{
			die();
		}
	}
	return 0;
}

void initializePortA()
{
	SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOA; // Enables Port A GPIO.
	GPIO_PORTA_DIR_R |= TRIG3 | TRIG2 | TRIG1 | TRIG0; // Sets Port A [5:2] as output.
	GPIO_PORTA_DEN_R |= TRIG3 | TRIG2 | TRIG1 | TRIG0; // Enables digital Port A [5:2].
}

void initializePortC()
{
	SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOC; // Enables Port C GPIO.
	GPIO_PORTC_DIR_R &= ~(ECHO3 | ECHO2 | ECHO1 | ECHO0); // Sets Port C [7:4] as input.
	GPIO_PORTC_DEN_R |= ECHO3 | ECHO2 | ECHO1 | ECHO0; // Enables digital Port C [7:4].
	GPIO_PORTC_IBE_R = ECHO3 | ECHO2 | ECHO1 | ECHO0; // Sets both edges detection mode.
	GPIO_PORTC_IM_R = ECHO3 | ECHO2 | ECHO1 | ECHO0; // Sets interrupt mask.
	NVIC_EN0_R |= 1<<2; // Enables Interrupt 2 by setting EN0[2] to 1.
}

void initializePortE()
{
	SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOE; // Enables Port E GPIO.
	GPIO_PORTE_DIR_R |= MOTORR1 | MOTORR0 | MOTORL1 | MOTORL0; // Sets Port E [3:0] as output.
	GPIO_PORTE_DEN_R |= MOTORR1 | MOTORR0 | MOTORL1 | MOTORL0; // Enables digital Port E [3:0].
}

void initializeTimer0()
{
	SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R0; // Enables Timer 0.
	TIMER0_CTL_R &= ~TIMER_CTL_TAEN; // Sets Bit 0 to 0. Disables the timer.
	TIMER0_CFG_R |= TIMER_CFG_32_BIT_TIMER; // Selects the 32-bit timer configuration.
	TIMER0_TAMR_R |= TIMER_TAMR_TAMR_PERIOD; // Sets periodic timer mode.
	double interval = 0.08; // 80 ms.
	TIMER0_TAILR_R = (int)((double)clockFrequency * interval);
	TIMER0_IMR_R |= TIMER_IMR_TATOIM; // Enables Timer 0 A time-out interrupt.
	TIMER0_CTL_R |= TIMER_CTL_TAEN; // Sets Bit 0 to 1. Enables the timer and starts counting.
	NVIC_EN0_R |= 1<<19; // Enables Interrupt 19 by setting EN0[19] to 1.
}

void initializeTimer1()
{
	SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1; // Enables Timer 1.
	TIMER1_CTL_R &= ~TIMER_CTL_TAEN; // Sets Bit 0 to 0. Disables the timer.
	TIMER1_CFG_R |= TIMER_CFG_32_BIT_TIMER; // Selects the 32-bit timer configuration.
	TIMER1_TAMR_R |= TIMER_TAMR_TAMR_1_SHOT; // Sets one-shot timer mode.
	double interval = 0.000010; // 10 us.
	TIMER1_TAILR_R = (int)((double)clockFrequency * interval);
	TIMER1_IMR_R |= TIMER_IMR_TATOIM; // Enables Timer 1 A time-out interrupt.
	TIMER1_CTL_R |= TIMER_CTL_TAEN; // Sets Bit 0 to 1. Enables the timer and starts counting.
	NVIC_EN0_R |= 1<<21; // Enables Interrupt 21 by setting EN0[21] to 1.
}

void initializeUART0()
{
	SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0; // Enables UART module 0.
	SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOA; // Enables Port A GPIO.
	GPIO_PORTA_AFSEL_R |= U0TX | U0RX; // Sets PA[1:0] for the alternate function (UART0).
	GPIO_PORTA_PCTL_R |= GPIO_PCTL_PA1_U0TX | GPIO_PCTL_PA0_U0RX; // Sets PA1 for UART0 TX and PA0 for UART0 RX.
	GPIO_PORTA_DEN_R |= U0TX | U0RX; // Enables the digital function of PA[1:0].
	UART0_CTL_R &= ~UART_CTL_UARTEN; // Disables UART0.
	double brd = (double)clockFrequency / (double)(CLK_DIV * BAUD_RATE);
	int iBRD = (int)brd;
	int fBRD = (int)((brd - (double)iBRD) * 64.0 + 0.5);
	UART0_IBRD_R = iBRD; // Sets the integer portion of the BRD.
	UART0_FBRD_R = fBRD; // Sets the fractional portion of the BRD.
	UART0_LCRH_R = UART_LCRH_WLEN_8; // Sets 8 bits in a frame.
	UART0_CC_R = UART_CC_CS_SYSCLK; // Sets the clock source to the system clock.
	UART0_IM_R = UART_IM_RXIM; // Sets the interrupt mask to receive interrupt.
	UART0_IFLS_R = UART_IFLS_RX1_8; // Interrupts with RX FIFO >= 1/8 full.
	UART0_CTL_R = UART_CTL_RXE | UART_CTL_TXE | UART_CTL_UARTEN; // Enables UART0, as well as transmitting and receiving.
	NVIC_EN0_R |= 1<<5; // Enables Interrupt 5 by setting EN0[5] to 1.
}

void initializeUART1()
{
	SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R1; // Enables UART module 1.
	SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOB; // Enables Port B GPIO.
	GPIO_PORTB_AFSEL_R |= U1TX | U1RX; // Sets PB[1:0] for the alternate function (UART1).
	GPIO_PORTB_PCTL_R |= GPIO_PCTL_PB1_U1TX | GPIO_PCTL_PB0_U1RX; // Sets PB1 for UART1 TX and PB0 for UART1 RX.
	GPIO_PORTB_DEN_R |= U1TX | U1RX; // Enables the digital function of PB[1:0].
	UART1_CTL_R &= ~UART_CTL_UARTEN; // Disables UART1.
	double brd = (double)clockFrequency / (double)(CLK_DIV * BAUD_RATE);
	int iBRD = (int)brd;
	int fBRD = (int)((brd - (double)iBRD) * 64.0 + 0.5);
	UART1_IBRD_R = iBRD; // Sets the integer portion of the BRD.
	UART1_FBRD_R = fBRD; // Sets the fractional portion of the BRD.
	UART1_LCRH_R = UART_LCRH_WLEN_8; // Sets 8 bits in a frame.
	UART1_CC_R = UART_CC_CS_SYSCLK; // Sets the clock source to the system clock.
	UART1_IM_R = UART_IM_RXIM; // Sets the interrupt mask to receive interrupt.
	UART1_IFLS_R = UART_IFLS_RX1_8; // Interrupts with RX FIFO >= 1/8 full.
	UART1_CTL_R = UART_CTL_RXE | UART_CTL_TXE | UART_CTL_UARTEN; // Enables UART1, as well as transmitting and receiving.
	NVIC_EN0_R |= 1<<6; // Enables Interrupt 6 by setting EN0[6] to 1.
}

void Port_C_Handler()
{
	GPIO_PORTC_ICR_R |= ECHO3 | ECHO2 | ECHO1 | ECHO0; // Clears the current interrupt.
	int digitalSignal = (GPIO_PORTC_DATA_R & (1<<(currentSensor + 4)))>>(currentSensor + 4);
	if (digitalSignal == 1)
	{
		startTime = TIMER0_TAR_R;
	}
	else
	{
		int endTime = TIMER0_TAR_R;
		double distance = (double)(-endTime + startTime) * 340.0 / 2.0 * 100.0 / (double)clockFrequency;
		switch (currentSensor)
		{
		case 0:
			distance0 = distance;
			currentSensor = 1;
			// printf("Distance 0: ");
			writeString1("Distance 0: ");
			break;
		case 1:
			distance1 = distance;
			currentSensor = 2;
			// printf("Distance 1: ");
			writeString1("Distance 1: ");
			break;
		case 2:
			distance2 = distance;
			currentSensor = 3;
			// printf("Distance 2: ");
			writeString1("Distance 2: ");
			break;
		case 3:
			distance3 = distance;
			currentSensor = 0;
			// printf("Distance 3: ");
			writeString1("Distance 3: ");
			break;
		default:
			break;
		}
		// printf("%.4f cm\n\n", distance);
		char buffer[8];
		snprintf(buffer, 8, "%7.4f", distance);
		writeString1(buffer);
		writeString1(" cm\n\r");
	}
}

void UART0_Handler()
{
	UART0_ICR_R |= UART_ICR_RXIC; // Sets Bit 4 to 1. Clears the receive interrupt.
	char c = readChar0();
}

void UART1_Handler()
{
	UART1_ICR_R |= UART_ICR_RXIC; // Sets Bit 4 to 1. Clears the receive interrupt.
	char c = readChar1();
	receivedData = c;
	writeString0("Data received: ");
	writeChar0(c);
	writeString0("\n\r");
}

void Timer_0_Handler()
{
	TIMER0_ICR_R |= TIMER_ICR_TATOCINT; // Sets Bit 0 to 1. Clears the current interrupt.
	initializeTimer1();
	GPIO_PORTA_DATA_R |= 1<<(currentSensor + 2);
}

void Timer_1_Handler()
{
	TIMER1_ICR_R |= TIMER_ICR_TATOCINT; // Sets Bit 0 to 1. Clears the current interrupt.
	GPIO_PORTA_DATA_R &= ~(1<<(currentSensor + 2));
}

char readChar0()
{
	while ((UART0_FR_R & UART_FR_RXFE) != 0) {} // Waits until the receiver is not empty.
	char c = UART0_DR_R; // Reads the data received by the UART.
	return c;
}

void writeChar0(char c)
{
	while ((UART0_FR_R & UART_FR_TXFF) != 0) {} // Waits until the transmitter is not full.
	UART0_DR_R = c; // Writes data from the transmitter.
}

void writeString0(char *s)
{
	while (*s)
	{
		writeChar0(*s);
		s++;
	}
}

char readChar1()
{
	while ((UART1_FR_R & UART_FR_RXFE) != 0) {} // Waits until the receiver is not empty.
	char c = UART1_DR_R; // Reads the data received by the UART.
	return c;
}

void writeChar1(char c)
{
	while ((UART1_FR_R & UART_FR_TXFF) != 0) {} // Waits until the transmitter is not full.
	UART1_DR_R = c; // Writes data from the transmitter.
}

void writeString1(char *s)
{
	while (*s)
	{
		writeChar1(*s);
		s++;
	}
}

void moveForward()
{
	GPIO_PORTE_DATA_R |= (MOTORR1 | MOTORL1);
	GPIO_PORTE_DATA_R &= ~(MOTORR0 | MOTORL0);
}

void moveBackward()
{
	GPIO_PORTE_DATA_R |= (MOTORR0 | MOTORL0);
	GPIO_PORTE_DATA_R &= ~(MOTORR1 | MOTORL1);
}

void turnLeft()
{
	GPIO_PORTE_DATA_R |= (MOTORR1 | MOTORL0);
	GPIO_PORTE_DATA_R &= ~(MOTORL1 | MOTORR0);
}

void turnRight()
{
	GPIO_PORTE_DATA_R |= (MOTORL1 | MOTORR0);
	GPIO_PORTE_DATA_R &= ~(MOTORR1 | MOTORL0);
}

void die()
{
	GPIO_PORTE_DATA_R &= ~(MOTORR1 | MOTORR0 | MOTORL1 | MOTORL0);
}
