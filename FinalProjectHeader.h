#ifndef FINAL_PROJECT_HEADER
#define FINAL_PROJECT_HEADER

/******************************************************************************/
// GPIO Port A Registers
/******************************************************************************/
#define GPIO_PORTA_DATA_R              (*((volatile unsigned long *)0x400043FC))
#define GPIO_PORTA_DIR_R               (*((volatile unsigned long *)0x40004400))
#define GPIO_PORTA_AFSEL_R             (*((volatile unsigned long *)0x40004420))
#define GPIO_PORTA_DEN_R               (*((volatile unsigned long *)0x4000451C))
#define GPIO_PORTA_PCTL_R              (*((volatile unsigned long *)0x4000452C))

/******************************************************************************/
// GPIO Port B Registers
/******************************************************************************/
#define GPIO_PORTB_AFSEL_R             (*((volatile unsigned long *)0x40005420))
#define GPIO_PORTB_DEN_R               (*((volatile unsigned long *)0x4000551C))
#define GPIO_PORTB_PCTL_R              (*((volatile unsigned long *)0x4000552C))

/******************************************************************************/
// GPIO Port C Registers
/******************************************************************************/
#define GPIO_PORTC_DATA_R              (*((volatile unsigned long *)0x400063FC))
#define GPIO_PORTC_DIR_R               (*((volatile unsigned long *)0x40006400))
#define GPIO_PORTC_IS_R                (*((volatile unsigned long *)0x40006404))
#define GPIO_PORTC_IBE_R               (*((volatile unsigned long *)0x40006408))
#define GPIO_PORTC_IM_R                (*((volatile unsigned long *)0x40006410))
#define GPIO_PORTC_ICR_R               (*((volatile unsigned long *)0x4000641C))
#define GPIO_PORTC_DEN_R               (*((volatile unsigned long *)0x4000651C))

/******************************************************************************/
// UART 0 Registers
/******************************************************************************/
#define UART0_DR_R                     (*((volatile unsigned long *)0x4000C000))
#define UART0_FR_R                     (*((volatile unsigned long *)0x4000C018))
#define UART0_IBRD_R                   (*((volatile unsigned long *)0x4000C024))
#define UART0_FBRD_R                   (*((volatile unsigned long *)0x4000C028))
#define UART0_LCRH_R                   (*((volatile unsigned long *)0x4000C02C))
#define UART0_CTL_R                    (*((volatile unsigned long *)0x4000C030))
#define UART0_IFLS_R                   (*((volatile unsigned long *)0x4000C034))
#define UART0_IM_R                     (*((volatile unsigned long *)0x4000C038))
#define UART0_ICR_R                    (*((volatile unsigned long *)0x4000C044))
#define UART0_CC_R                     (*((volatile unsigned long *)0x4000CFC8))

/******************************************************************************/
// UART 1 Registers
/******************************************************************************/
#define UART1_DR_R                     (*((volatile unsigned long *)0x4000D000))
#define UART1_FR_R                     (*((volatile unsigned long *)0x4000D018))
#define UART1_IBRD_R                   (*((volatile unsigned long *)0x4000D024))
#define UART1_FBRD_R                   (*((volatile unsigned long *)0x4000D028))
#define UART1_LCRH_R                   (*((volatile unsigned long *)0x4000D02C))
#define UART1_CTL_R                    (*((volatile unsigned long *)0x4000D030))
#define UART1_IFLS_R                   (*((volatile unsigned long *)0x4000D034))
#define UART1_IM_R                     (*((volatile unsigned long *)0x4000D038))
#define UART1_ICR_R                    (*((volatile unsigned long *)0x4000D044))
#define UART1_CC_R                     (*((volatile unsigned long *)0x4000DFC8))

/******************************************************************************/
// GPIO Port E Registers
/******************************************************************************/
#define GPIO_PORTE_DATA_R              (*((volatile unsigned long *)0x400243FC))
#define GPIO_PORTE_DIR_R               (*((volatile unsigned long *)0x40024400))
#define GPIO_PORTE_DEN_R               (*((volatile unsigned long *)0x4002451C))

/******************************************************************************/
// Timer 0 Registers
/******************************************************************************/
#define TIMER0_CFG_R                   (*((volatile unsigned long *)0x40030000))
#define TIMER0_TAMR_R                  (*((volatile unsigned long *)0x40030004))
#define TIMER0_CTL_R                   (*((volatile unsigned long *)0x4003000C))
#define TIMER0_IMR_R                   (*((volatile unsigned long *)0x40030018))
#define TIMER0_ICR_R                   (*((volatile unsigned long *)0x40030024))
#define TIMER0_TAILR_R                 (*((volatile unsigned long *)0x40030028))
#define TIMER0_TAR_R                   (*((volatile unsigned long *)0x40030048))

/******************************************************************************/
// Timer 1 Registers
/******************************************************************************/
#define TIMER1_CFG_R                   (*((volatile unsigned long *)0x40031000))
#define TIMER1_TAMR_R                  (*((volatile unsigned long *)0x40031004))
#define TIMER1_CTL_R                   (*((volatile unsigned long *)0x4003100C))
#define TIMER1_IMR_R                   (*((volatile unsigned long *)0x40031018))
#define TIMER1_ICR_R                   (*((volatile unsigned long *)0x40031024))
#define TIMER1_TAILR_R                 (*((volatile unsigned long *)0x40031028))

/******************************************************************************/
// System Control Registers
/******************************************************************************/
#define SYSCTL_RCGC2_R                 (*((volatile unsigned long *)0x400FE108))
#define SYSCTL_RCGCTIMER_R             (*((volatile unsigned long *)0x400FE604))
#define SYSCTL_RCGCUART_R              (*((volatile unsigned long *)0x400FE618))

/******************************************************************************/
// NVIC Registers
/******************************************************************************/
#define NVIC_EN0_R                     (*((volatile unsigned long *)0xE000E100))

/******************************************************************************/
// Constants for the GPIO Registers
/******************************************************************************/
#define GPIO_PCTL_PA1_U0TX 0x00000010 // U0TX on PA1
#define GPIO_PCTL_PA0_U0RX 0x00000001 // U0RX on PA0
#define GPIO_PCTL_PB1_U1TX 0x00000010  // U1TX on PB1
#define GPIO_PCTL_PB0_U1RX 0x00000001  // U1RX on PB0

/******************************************************************************/
// Constants for the UART Registers
/******************************************************************************/
#define UART_FR_TXFF 0x00000020 // UART Transmit FIFO Full
#define UART_FR_RXFE 0x00000010 // UART Receive FIFO Empty
#define UART_LCRH_WLEN_8 0x00000060 // 8 bits
#define UART_CTL_RXE 0x00000200 // UART Receive Enable
#define UART_CTL_TXE 0x00000100 // UART Transmit Enable
#define UART_CTL_UARTEN 0x00000001 // UART Enable
#define UART_IFLS_RX1_8 0x00000000 // RX FIFO >= 1/8 full
#define UART_IM_RXIM 0x00000010 // UART Receive Interrupt Mask
#define UART_ICR_RXIC 0x00000010 // Receive Interrupt Clear
#define UART_CC_CS_SYSCLK 0x00000000 // The system clock (default)

/******************************************************************************/
// Constants for the TIMER Registers
/******************************************************************************/
#define TIMER_CTL_TAEN 0x00000001  // GPTM Timer A Enable
#define TIMER_CFG_32_BIT_TIMER 0x00000000  // 32-bit timer configuration
#define TIMER_TAMR_TAMR_PERIOD 0x00000002 // Periodic Timer mode
#define TIMER_TAMR_TAMR_1_SHOT 0x00000001 // One-Shot Timer mode
#define TIMER_IMR_TATOIM 0x00000001 // GPTM Timer A Time-Out Interrupt Mask
#define TIMER_ICR_TATOCINT 0x00000001 // GPTM Timer A Time-Out Raw Interrupt
#define TIMER_CTL_TAOTE 0x00000020  // GPTM Timer A Output Trigger Enable

/******************************************************************************/
// Constants for the SYSCEL Registers
/******************************************************************************/
#define SYSCTL_RCGC2_GPIOE 0x00000010 // Port E Clock Gating Control
#define SYSCTL_RCGC2_GPIOC 0x00000004 // Port C Clock Gating Control
#define SYSCTL_RCGC2_GPIOB 0x00000002 // Port B Clock Gating Control
#define SYSCTL_RCGC2_GPIOA 0x00000001 // Port A Clock Gating Control
#define SYSCTL_RCGCTIMER_R1 0x00000002 // Timer 1 Run Mode Clock Gating Control
#define SYSCTL_RCGCTIMER_R0 0x00000001 // Timer 0 Run Mode Clock Gating Control
#define SYSCTL_RCGCUART_R1 0x00000002 // UART Module 1 Run Mode Clock Gating Control
#define SYSCTL_RCGCUART_R0 0x00000001 // UART Module 0 Run Mode Clock Gating Control

/******************************************************************************/
// Other Constants in This Project
/******************************************************************************/
#define TRIG3 1<<5 // PA5
#define TRIG2 1<<4 // PA4
#define TRIG1 1<<3 // PA3
#define TRIG0 1<<2 // PA2

#define U0TX 1<<1 // PA1
#define U0RX 1<<0 // PA0

#define U1TX 1<<1 // PB1
#define U1RX 1<<0 // PB0

#define ECHO3 1<<7 // PC7
#define ECHO2 1<<6 // PC6
#define ECHO1 1<<5 // PC5
#define ECHO0 1<<4 // PC4

#define MOTORR1 1<<3 // PE3
#define MOTORR0 1<<2 // PE2
#define MOTORL1 1<<1 // PE1
#define MOTORL0 1<<0 // PE0

#define BAUD_RATE 9600
#define CLK_DIV 16

#define SIGNAL_MIDDLE 'M'
#define SIGNAL_LEFT 'R'
#define SIGNAL_RIGHT 'L'

/******************************************************************************/
// Functions in This Project
/******************************************************************************/
void initializePortA();
void initializePortC();
void initializePortE();
void initializeTimer0();
void initializeTimer1();
void initializeUART0();
void initializeUART1();

void Port_C_Handler();
void UART0_Handler();
void UART1_Handler();
void Timer_0_Handler();
void Timer_1_Handler();

char readChar0();
void writeChar0(char c);
void writeString0(char *s);

char readChar1();
void writeChar1(char c);
void writeString1(char *s);

void moveForward();
void moveBackward();
void turnLeft();
void turnRight();
void die();

#endif
