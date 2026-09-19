#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <math.h>
#include "clock.h"
#include "wait.h"
#include "uart0.h"
#include "fields.h"
#include "eeprom.h"
#include "tm4c123gh6pm.h"

//#define DEBUG_115200

#define DMX_CHANNELS 512
#define MODE_ADDRESS 0          // The EEPROM address of the controller/device mode
#define DEVICE_ADDR_ADDR 1      // The EEPROM address of the device address

#define DE          (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 6*4)))
#define RED_LED     (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 1*4)))
#define GREEN_LED   (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 3*4)))
#define DE_MASK         0b01000000
#define RED_LED_MASK    0b00000010
#define GREEN_LED_MASK  0b00001000

// External LEDs controlled by PWM
#define OUT_RED_LED     (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 6*4)))
#define OUT_GREEN_LED   (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 7*4)))
#define OUT_RED_LED_MASK    0b01000000
#define OUT_GREEN_LED_MASK  0b10000000

#define UART4_RX_MASK   0b00010000
#define UART4_TX_MASK   0b00100000
#define NVIC_UART4_OFFSET 0x30  // 0x10 for peripheral IRQ offset + 0x20 for EN1 offset

struct DmxData {
    uint8_t data[DMX_CHANNELS];     // 1st element should be reserved for start code
    uint8_t rxData[DMX_CHANNELS];   // Holds the received data
    uint16_t addr;                  // Holds the last sent address
    bool controllerMode;            // 1: controller | 0: receiver
    uint16_t max;                   // Number of bytes to be sent
    bool waiting;                   // State machine doesn't run when waiting
    uint8_t state;                  // 0: BRK, 1: MAB/SC, 2: DATA
    uint16_t deviceAddr;            // Address of device when in device mode
    bool txOn;                      // Whether or not you're sending DMX stream
    uint16_t rxCounter;             // Counts number of RXs until next BRK
    uint8_t rxStartCode;            // Holds the received start code
    bool rxDataChanged;             // Indicates whether or not the received data is different than what it used to be
    bool blinking;                  // True when rxData changed and waiting to finish blink, false otherwise
} dmxData;

#define MAX_CHARS 32    // User can only use up to 32-character long commands
#define MAX_FIELDS 6    // Up to 6 fields in the command (including command itself)

struct UserData {
    char buffer[MAX_CHARS+1];
    uint8_t fieldCount;
    uint8_t fieldPosition[MAX_FIELDS];
    char fieldType[MAX_FIELDS];
} userData;

#define NUM_SIGNALS 10
#define INACTIVE 0
#define RAMP     1
#define PULSE    2

typedef struct Signal {
    uint8_t type;       // Either INACTIVE, RAMP, or PULSE
    uint16_t addr;      // dmx address to send the value on
    uint32_t time;      // in ms
    uint8_t start;
    uint8_t stop;
    uint32_t period;     // Number of timer2Isr's (100us) per increment
    uint32_t isrsLeft;   // How many timer2Isr's (100us) still need to happen before incrementing value
    bool positiveSlope; // Used for pulse signals to indicate decreasing value
    uint32_t count;     // Number of signals to to send
    bool onStop;        // Whether or not, during PULSE, value is currently stop value or start value
} Signal;
Signal signals[NUM_SIGNALS];

#define MAX_DATASTR 5   // String of data cannot be higher than 5 characters ('5', '1', '2', '\n', '\0')

void initHw() {
    initSystemClockTo40Mhz();

    // Enable clock, set to output, & enable pin for D, DE, R, & RED_LED
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R4;    // Enable Uart4 clk
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R2;    // Enable PORTC clk for DE & Uart4 Rx/Tx GPIOs
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;  // Enable timer1 clk
    _delay_cycles(3);

    GPIO_PORTC_DIR_R |= DE_MASK;
    GPIO_PORTC_DEN_R |= DE_MASK | UART4_RX_MASK | UART4_TX_MASK;
    GPIO_PORTC_AFSEL_R |= UART4_RX_MASK | UART4_TX_MASK;
    GPIO_PORTC_PCTL_R &= ~(GPIO_PCTL_PC4_M | GPIO_PCTL_PC5_M);
    GPIO_PORTC_PCTL_R |= GPIO_PCTL_PC4_U4RX | GPIO_PCTL_PC5_U4TX;

    // Modify Uart4 registers
    UART4_CTL_R = 0;
    UART4_CC_R = UART_CC_CS_SYSCLK;
    UART4_IBRD_R = 10;  // 40MHz/(16*250kbps) = 10
    UART4_FBRD_R = 0;
    #ifdef DEBUG_115200
    UART4_IBRD_R = 21; // 115.2kbps
    UART4_FBRD_R = 45;
    #endif
    UART4_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_STP2;
    UART4_CTL_R = UART_CTL_RXE | UART_CTL_TXE | UART_CTL_UARTEN;
    UART4_IM_R |= UART_IM_RXIM;
    NVIC_EN1_R = 1 << (INT_UART4 - NVIC_UART4_OFFSET);

    // set up LEDs
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5;
    _delay_cycles(3);
    GPIO_PORTF_DIR_R |= RED_LED_MASK | GREEN_LED_MASK;
    GPIO_PORTF_DEN_R |= RED_LED_MASK | GREEN_LED_MASK;

    initEeprom();

    // Configure Timer1
    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;
    TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;
    TIMER1_TAMR_R = TIMER_TAMR_TAMR_1_SHOT;

    // Set up Timer1 interrupts
    TIMER1_IMR_R = TIMER_IMR_TATOIM;
    NVIC_EN0_R = 1 << (INT_TIMER1A-16);

    // Configure PWM-related clocks and pins
    SYSCTL_RCGCPWM_R |= SYSCTL_RCGCPWM_R1;
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R0;
    _delay_cycles(3);
    GPIO_PORTA_DEN_R |= OUT_RED_LED_MASK | OUT_GREEN_LED_MASK;
    GPIO_PORTA_AFSEL_R |= OUT_RED_LED_MASK | OUT_GREEN_LED_MASK;
    GPIO_PORTA_PCTL_R &= ~(GPIO_PCTL_PA6_M | GPIO_PCTL_PA7_M);
    GPIO_PORTA_PCTL_R |= GPIO_PCTL_PA6_M1PWM2 | GPIO_PCTL_PA7_M1PWM3;

    // Configure PWM module 1
    SYSCTL_SRPWM_R = SYSCTL_SRPWM_R1;   // PWM1 software reset
    SYSCTL_SRPWM_R = 0;                 // Finish reset
    PWM1_1_CTL_R = 0;                   // Turn off PWM1 generator 1
    PWM1_1_GENA_R = PWM_1_GENA_ACTCMPAD_ONE | PWM_1_GENA_ACTLOAD_ZERO;
    PWM1_1_GENB_R = PWM_1_GENB_ACTCMPBD_ONE | PWM_1_GENB_ACTLOAD_ZERO;
    PWM1_1_LOAD_R = 1024;               // 40MHz/2/1024 ~= 20kHz
    PWM1_1_CMPA_R = 0;                  // start with pin off
    PWM1_1_CMPB_R = 0;                  // start with pin off
    PWM1_1_CTL_R = PWM_1_CTL_ENABLE;    // Turn on PWM1 generator 1
    PWM1_ENABLE_R = PWM_ENABLE_PWM2EN | PWM_ENABLE_PWM3EN;  // Enable outputs

    initUart0();
}

void initDmxData() {
    int i;
    for (i = 0; i < DMX_CHANNELS; i++) {                    // Clear tx data & rx data
        dmxData.data[i] = 0;
        dmxData.rxData[i] = 0;
    }
    dmxData.addr = 0;                                       // Start from 1st address
    dmxData.controllerMode = false;                         // Initialize to device mode
    dmxData.max = DMX_CHANNELS;                             // Initialize to sending all possible data
    dmxData.waiting = false;                                // Start state machine
    dmxData.state = 0;                                      // Start from sending BRK
    dmxData.deviceAddr = 1;
    dmxData.txOn = true;
    dmxData.rxCounter = 0;
    dmxData.rxStartCode = 1;                                // Init so a 0 actually needs to be rx'd
    dmxData.rxDataChanged = 0;                              // Init so a change isn't detected immediately
    dmxData.blinking = 0;                                   // false so you can see that you're receiving data initially
}

void clearUserData() {
    int i;

    for (i = 0; i <= MAX_CHARS; i++) {
        userData.buffer[i] = '\0';
    }
    for (i = 0; i < MAX_FIELDS; i++) {
        userData.fieldType[i] = '\0';
        userData.fieldPosition[i] = 0;
    }

    userData.fieldCount = 0;
}

void clearSignalData() {
    int i = 0;
    for (i = 0; i < NUM_SIGNALS; i++) {
        signals[i].type = INACTIVE;
        signals[i].addr = 0;
        signals[i].time = 0;
        signals[i].start = 0;
        signals[i].stop = 0;
        signals[i].period = 0;
        signals[i].isrsLeft = 0;
        signals[i].positiveSlope = true;
        signals[i].count = 1;
        signals[i].onStop = false;
    }
}

void timer1Isr() {

    TIMER1_ICR_R = TIMER_ICR_TATOCINT;  // Clear timer1 interrupt flag
    dmxData.waiting = false;

    if (dmxData.txOn) {
        switch (dmxData.state) {
            case 0: dmxData.state = 1; break;
            case 1: dmxData.state = 2; break;
            case 2: if (dmxData.addr >= dmxData.max) dmxData.state = 0; break;
            case 3: dmxData.state = 0; break;
        }
    }
    else {
        RED_LED = 1;    // Turn activity LED back on to blink
        dmxData.state = 3;  // Do-nothing state
    }
}

void startTimer1(uint32_t cycles) {
    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;    // Turn off Timer1
    TIMER1_TAILR_R = cycles;            // Change Timer1 counter
    TIMER1_CTL_R |= TIMER_CTL_TAEN;     // Turn on Timer1
}

void timer2Isr() {
    TIMER2_ICR_R = TIMER_ICR_TATOCINT;  // Clear interrupt flag

    int i = 0;
    for (i = 0; i < NUM_SIGNALS; i++) {
        if (signals[i].type != INACTIVE) {
            // Check if signal reached final value, set it to inactive [unless more counts]
            //if (dmxData.data[signals[i].addr] == signals[i].stop && signals[i].count == 0) {
            //    signals[i].type = INACTIVE;
            //}
            if (signals[i].type == RAMP) {
                // When we've waited enough ISRs, increment data and reset isrsLeft to period
                if (signals[i].isrsLeft == 0) {
                    (signals[i].positiveSlope) ? dmxData.data[signals[i].addr]++ : dmxData.data[signals[i].addr]--; // Choose to increment or decrement

                    // If signal reached stop value, make INACTIVE [unless more counts]
                    if (dmxData.data[signals[i].addr] == signals[i].stop) {
                        if (signals[i].count == 0) {
                            signals[i].type = INACTIVE;
                        }
                        else {
                            dmxData.data[signals[i].addr] = signals[i].start;
                            signals[i].count--;
                        }
                    }
                    signals[i].isrsLeft = signals[i].period;
                }
                // If we haven't waited enough, decrement how much we have to wait
                else {
                    signals[i].isrsLeft--;
                }
            }
            else if (signals[i].type == PULSE) {
                if (signals[i].isrsLeft == 0) {
                    dmxData.data[signals[i].addr] = (signals[i].onStop) ? signals[i].start : signals[i].stop;

                    if (signals[i].count == 0) {
                        signals[i].type = INACTIVE;
                    }
                    else {
                        signals[i].isrsLeft = signals[i].period;
                        signals[i].onStop ^= 1;
                        signals[i].count--;
                    }
                }
                else {
                    signals[i].isrsLeft--;
                }
            }
        }
    }
}

void startTimer2() {
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R2;  // Enable timer2 clk
    _delay_cycles(3);

    TIMER2_CTL_R &= ~TIMER_CTL_TAEN;
    TIMER2_CFG_R = TIMER_CFG_16_BIT;
    TIMER2_TAMR_R = TIMER_TAMR_TAMR_PERIOD;
    TIMER2_TAILR_R = 4000;   // 100us (10,000Hz interrupt rate)
    TIMER2_IMR_R = TIMER_IMR_TATOIM;
    TIMER2_CTL_R |= TIMER_CTL_TAEN;
    NVIC_EN0_R = 1 << (INT_TIMER2A-16);
}

char* toArray(int number)
{
    int n;
    if (number != 0) {
        n = log10(number) + 1;
    } else {
        n = 1;
    }
    int i;
    char* numberArray = calloc(n, sizeof(char));
    for (i = n-1; i >= 0; --i, number /= 10)
    {
        numberArray[i] = (number % 10) + '0';
    }
    return numberArray;
}

void txStateMachine()
{
    UART4_CTL_R &= ~UART_CTL_RXE;   // Turn off Uart4 Rx modules
    UART4_CTL_R |= UART_CTL_TXE;    // Turn on Uart4 Tx module

    RED_LED = 0;
    GREEN_LED = dmxData.txOn;       // Turn on green Tx LED when transmitting DMX data

    if (!dmxData.waiting) {
        switch (dmxData.state) {
        case 0:
            if (!dmxData.txOn) return;

            dmxData.addr = 0;
            UART4_LCRH_R |= UART_LCRH_BRK;      // Set BRK
            #ifndef DEBUG_115200
            startTimer1(3680);                   // 92us
            #endif
            #ifdef DEBUG_115200
            startTimer1(7986);                   // 200us
            #endif
            break;
        case 1:
            UART4_LCRH_R &= ~UART_LCRH_BRK;     // Clear BRK
            #ifndef DEBUG_115200
            _delay_cycles(480);                 // 12us (250kbps)
            #endif
            #ifdef DEBUG_115200
            _delay_cycles(1042);                // 26us (115.2kbps)
            #endif
            //while (UART4_FR_R & UART_FR_BUSY);  // Block until FIFO is free
            UART4_DR_R = 0;                     // Send start code
            #ifndef DEBUG_115200
            startTimer1(1760);                   // 44us (250kbps)
            #endif
            #ifdef DEBUG_115200
            startTimer1(3820);                   // 95us (115.2kbps)
            #endif
            break;
        case 2:
            //while (UART4_FR_R & UART_FR_BUSY);  // Block until FIFO is free
            UART4_DR_R = dmxData.data[dmxData.addr++];  // Write data
            #ifndef DEBUG_115200
            startTimer1(1760);                   // 44us (250kbps)
            #endif
            #ifdef DEBUG_115200
            startTimer1(3820);                   // 95us (115.2kbps)
            #endif
            break;
        case 3:
            startTimer1(5);
            break;
        }

        dmxData.waiting = true;
    }
}

void userInterface()
{
    if (kbhitUart0()) {
        clearUserData();
        RED_LED ^= 1;

        getsUart0(userData.buffer, MAX_CHARS);

        parseFields(userData.buffer, userData.fieldType, userData.fieldPosition, &userData.fieldCount, MAX_FIELDS, MAX_CHARS);

        if (isCommand(userData.buffer, userData.fieldCount, "clear", 0)) {
            int i;
            for (i = 0; i < DMX_CHANNELS; i++) dmxData.data[i] = 0;
        }
        else if (isCommand(userData.buffer, userData.fieldCount, "set", 2)) {
            uint16_t addr = getFieldInteger(userData.buffer, userData.fieldPosition, userData.fieldCount, 1);
            uint8_t data = getFieldInteger(userData.buffer, userData.fieldPosition, userData.fieldCount, 2);
            if (addr > 512 || addr == 0) {
                putsUart0("Invalid address!\n");
            } else {
                dmxData.data[addr-1] = data;    // Doing addr-1 because converting 0-511 -> 1-512
            }
        }
        else if (isCommand(userData.buffer, userData.fieldCount, "get", 1)) {
            uint16_t addr = getFieldInteger(userData.buffer, userData.fieldPosition, userData.fieldCount, 1);
            if (addr > 512 || addr == 0) {
                putsUart0("Invalid address!\n");
            } else {
                char* dataStr = toArray(dmxData.data[addr-1]);
                putsUart0(dataStr);
                putcUart0('\n');
                free(dataStr);
            }
        }
        else if (isCommand(userData.buffer, userData.fieldCount, "max", 1)) {
            uint16_t max = getFieldInteger(userData.buffer, userData.fieldPosition, userData.fieldCount, 1);
            if (max > 512) {
                putsUart0("Highest max is 512!\n");
            } else {
                dmxData.max = max;
            }
        }
        else if (isCommand(userData.buffer, userData.fieldCount, "controller", 0)) {
            dmxData.controllerMode = true;
            writeEeprom(MODE_ADDRESS, 1);
            DE = 1;
        }
        else if (isCommand(userData.buffer, userData.fieldCount, "device", 0)) {
            dmxData.controllerMode = false;
            writeEeprom(MODE_ADDRESS, 0);
            DE = 0;
            GREEN_LED = 0;

            // Save specified device address (1 if not specified)
            if (userData.fieldCount == 1) {         // no device address specified
                dmxData.deviceAddr = 1;
                writeEeprom(DEVICE_ADDR_ADDR, 1);
            } else {                                // device address specified
                uint16_t deviceAddr = getFieldInteger(userData.buffer, userData.fieldPosition, userData.fieldCount, 1);
                if (deviceAddr > 512 || deviceAddr == 0) {
                    putsUart0("Invalid address!\n");
                } else {
                    dmxData.deviceAddr = deviceAddr;
                    writeEeprom(DEVICE_ADDR_ADDR, dmxData.deviceAddr);
                }
            }
        }
        else if (isCommand(userData.buffer, userData.fieldCount, "on", 0)) {
            if (dmxData.controllerMode) {
                dmxData.txOn = true;
                dmxData.waiting = false;
            } else {
                putsUart0("Must be in controller mode to set DMX on!\n");
            }
        }
        else if (isCommand(userData.buffer, userData.fieldCount, "off", 0)) {
            if (dmxData.controllerMode) {
                dmxData.txOn = false;
            } else {
                putsUart0("Must be in controller mode to set DMX off!\n");
            }
        }
        else if (isCommand(userData.buffer, userData.fieldCount, "ramp", 5)) {
            bool freeSignalFound = false;

            // Set the variables for the first available signal
            int i = 0;
            for (; i < NUM_SIGNALS; i++) {
                if (signals[i].type == INACTIVE) {
                    signals[i].type = RAMP;
                    signals[i].addr = getFieldInteger(userData.buffer, userData.fieldPosition, userData.fieldCount, 1)-1;   // -1 to scale 0->511 to 1->512
                    signals[i].time = getFieldInteger(userData.buffer, userData.fieldPosition, userData.fieldCount, 2);
                    signals[i].start = getFieldInteger(userData.buffer, userData.fieldPosition, userData.fieldCount, 3);
                    signals[i].stop = getFieldInteger(userData.buffer, userData.fieldPosition, userData.fieldCount, 4);
                    signals[i].count = getFieldInteger(userData.buffer, userData.fieldPosition, userData.fieldCount, 5)-1;
                    signals[i].positiveSlope = signals[i].start <= signals[i].stop;
                    signals[i].period = signals[i].time/abs(signals[i].stop - signals[i].start) * 10;
                    signals[i].isrsLeft = signals[i].period;
                    dmxData.data[signals[i].addr] = signals[i].start;
                    freeSignalFound = true;
                    break;
                }
            }
            if (!freeSignalFound) putsUart0("All 10 signals are being used!\n");
        }
        else if (isCommand(userData.buffer, userData.fieldCount, "pulse", 5)) {
            bool freeSignalFound = false;

            // Set the variables for the first available signal
            int i = 0;
            for (; i < NUM_SIGNALS; i++) {
                if (signals[i].type == INACTIVE) {
                    signals[i].type = PULSE;
                    signals[i].addr = getFieldInteger(userData.buffer, userData.fieldPosition, userData.fieldCount, 1)-1;
                    signals[i].time = getFieldInteger(userData.buffer, userData.fieldPosition, userData.fieldCount, 2);
                    signals[i].start = getFieldInteger(userData.buffer, userData.fieldPosition, userData.fieldCount, 3);
                    signals[i].stop = getFieldInteger(userData.buffer, userData.fieldPosition, userData.fieldCount, 4);
                    signals[i].count = (getFieldInteger(userData.buffer, userData.fieldPosition, userData.fieldCount, 5)-1) << 1;   // Multiply by 2 for both high and low
                    signals[i].onStop = false;
                    signals[i].period = signals[i].time * 10;   // (time in ms * 0.001)/100us
                    signals[i].isrsLeft = signals[i].period;
                    dmxData.data[signals[i].addr] = signals[i].start;
                    freeSignalFound = true;
                    break;
                }
            }
            if (!freeSignalFound) putsUart0("All 10 signals are being used!\n");
        }
        else {
            putsUart0("Unrecognized command\n");
        }
    }
}

uint8_t getByteUart4()
{
    while (UART4_FR_R & UART_FR_RXFE);              // block until data comes in
    return UART4_DR_R & 0xFF;                       // get byte from the fifo
}

void rxHandler()
{
    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;    // Turn off Timer1
    UART4_IM_R |= UART_IM_RXIM;         // Allow Rx interrupts
    UART4_CTL_R |= UART_CTL_RXE;
    UART4_CTL_R &= ~UART_CTL_TXE;       // Turn off Uart4 Tx module

    PWM1_1_CMPA_R = dmxData.rxData[dmxData.deviceAddr-1]*4;   // OUT_RED_LED    (x4 to scale 255->1020)
    PWM1_1_CMPB_R = dmxData.rxData[dmxData.deviceAddr]*4;     // OUT_GREEN_LED

    //RED_LED = 0;
}

void uart4RxIsr()
{
    dmxData.rxDataChanged = false;
    int data = UART4_DR_R;

    if (dmxData.rxCounter == 0) {               // 1st valid data after break error
        if (!dmxData.blinking) RED_LED = 1;     // Turn on activity LED when not blinking (stays on until BRK)
        dmxData.rxStartCode = data & 0xFF;      // set to start code
        dmxData.rxCounter = 1;                      //
    }
    else if (dmxData.rxStartCode == 0 && !(data & UART_DR_BE)) {  // if no break error on rx'd data,
        if (dmxData.rxData[dmxData.rxCounter-1] != (data & 0xFF)) dmxData.rxDataChanged = true;
        dmxData.rxData[dmxData.rxCounter-1] = data & 0xFF;     // store data
        dmxData.rxCounter++;
    }
    else if (data & UART_DR_BE) {    // if data has break error
        dmxData.rxCounter = 0;
        RED_LED = 0;                // Turn activity LED off (stays off until next data)
    }

    if (dmxData.rxCounter > 512) {
        //putsUart0("Got more than 512 frames!");
        dmxData.rxCounter = 0;
    }

    // Keep LED off for 0.5s (+ until next start code) if data was changed to diff value
    if (dmxData.rxDataChanged && (data & UART_DR_BE)) {
        RED_LED = 0;
        startTimer1(20000000);      // Turn LED back on in timer1Isr
        dmxData.blinking = true;    // Make other activity LED stay off until this is cleared
    }

    //UART4_ICR_R |= UART_ICR_RXIC;
}

int main(void)
{
    initDmxData();
    clearSignalData();
    initHw();
    startTimer2();

    // Set to saved mode
    dmxData.controllerMode = readEeprom(MODE_ADDRESS);
    dmxData.deviceAddr = readEeprom(DEVICE_ADDR_ADDR);
    DE = dmxData.controllerMode;

    while (true) {
        (dmxData.controllerMode) ? txStateMachine() : rxHandler();
        userInterface();
    }
}
