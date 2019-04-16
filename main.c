
/*
 * TODO:
 *  - combine CCW_motion and CW_motion into one function with a direction argument that is a 1 or 0.
 *      example prototype: void motor_motion(int steps, char direction);
 */

#include  <msp430xG46x.h>
#include <math.h>
//#include  <stdio.h>
#define POS4 BIT7
char direction = 0; // sets spin direction of motor
volatile long int ADC_x, ADC_y, ADC_z;

void CCW_motion(float angle){
    static int i = 0;
    int steps =
    P2OUT &= ~(BIT0 | BIT3 | BIT6 | BIT7);

    if(i == 1){
        P2OUT ^= BIT0;
    }
    else if(i == 2){
        P2OUT ^= BIT3;
    }
    else if(i == 3){
        P2OUT ^= BIT6;
    }
    else{
        P2OUT ^= BIT7;
        i = 0;
    }
    i++;
}

void CW_motion(){
    int i = 3;
//    while(steps){
    //    P1OUT &= ~BIT6;
    //    P2OUT &= ~(BIT5 | BIT4 | BIT3);
        P2OUT &= ~(BIT0 | BIT3 | BIT6 | BIT7);
        if(i == 1){
            P2OUT ^= BIT0;
        }
        else if(i == 2){
            P2OUT ^= BIT3;
        }
        else if(i == 3){
            P2OUT ^= BIT6;
        }
        else{
            P2OUT ^= BIT7;
            i = 4;
        }
        i--;
//    }
}
void TimerA_setup(void) {
    TACCR0 = 200;                      // 6552 / 32768 Hz = 0.2s
    TACTL = TASSEL_1 + MC_1;            // ACLK, up mode
    TACCTL0 = CCIE;                     // Enabled interrupt
}

void TimerB_setup(void) {
    // setup buzzer on Port 3.5
    P3DIR |= BIT5;  // P3.5 set up as output
    P3SEL |= BIT5;

    TB0CCTL4 = OUTMOD_7;          // PWM reset/set mode
    TB0CTL = TBSSEL_1 | MC_1;     // ACLK is clock source, UP mode
    TB0CCR0 = 31;                // Set TB0 (and maximum) count value
    TB0CCR4 = 0;                // Set TB4 count value
}

void ADC_setup(void) {
    int i =0;

    P6DIR &= ~BIT3 + ~BIT4 + ~BIT6;              // Configure P6.3 and P6.7 as input pins
    P6SEL |= BIT3 + BIT4 + BIT6;                 // Configure P6.3 and P6.7 as analog pins
    ADC12CTL0 = ADC12ON + SHT0_6 + MSC;          // configure ADC converter
    ADC12CTL1 = SHP + CONSEQ_1;                  // Use sample timer, single sequence
    ADC12MCTL0 = INCH_3 + SREF_0;                // ADC A3 pin - Accel X-axis
    ADC12MCTL1 = INCH_4 + SREF_0;                // ADC A4 pin - Accel Y-axis
    ADC12MCTL2 = INCH_6 + + SREF_0 + EOS;        // ADC A6 pin - Accel Z-axis
                                                 // EOS - End of Sequence for Conversions
    ADC12IE |= 0x04;                    // Enable ADC12IFG.2
    for (i = 0; i < 0x3600; i++);       // Delay for reference start-up
    ADC12CTL0 |= ENC;                   // Enable conversions
}

void UART_putCharacter(char c) {
    while(!(IFG2 & UCA0TXIFG));         // Wait for previous character to be sent
    UCA0TXBUF = c;                      // Send byte to the buffer for transmitting
}

void UART_setup(void) {
    P2SEL |= BIT4 + BIT5;               // Set up Rx and Tx bits
    UCA0CTL0 = 0;                       // Set up default RS-232 protocol
    UCA0CTL1 |= BIT0 + UCSSEL_2;        // Disable device, set clock
    UCA0BR0 = 27;                       // 1048576 Hz / 38400
    UCA0BR1 = 0;
    UCA0MCTL = 0x94;
    UCA0CTL1 &= ~BIT0;                  // Start UART device
}

float alpha_beta_mag(float alpha, float beta, float inphase, float quadrature){
    /* magnitude ~= alpha * max(|I|, |Q|) + beta * min(|I|, |Q|) */
    float abs_inphase = fabsf(inphase);
    float abs_quadrature = fabsf(quadrature);
    if (abs_inphase > abs_quadrature) {
          return alpha * abs_inphase + beta * abs_quadrature;
       }
    else {
          return alpha * abs_quadrature + beta * abs_inphase;
       }
}

float sendData(void) {
    // Sends accel data through UART and returns the magnitude of the acceleration
    int i;
    float magnitude,angle,pitch;
    magnitude = 0;

    accel_x = ((ADC_x*3.3/4095-1.65)/0.3);
    accel_y = ((ADC_y*3.3/4095-1.65)/0.3);
    accel_z = ((ADC_z*3.3/4095-1.65)/0.3);
//    __no_operation();
//    __no_operation();
//    __no_operation();
//    pitch = atan2f((-accel_y),sqrt(accel_z*accel_z+accel_x*accel_x))*180/M_PI;
    angle = (atan2f(accel_z,accel_y)*180/M_PI) + 180;
//    magnitude = powf(accel_x*accel_x + accel_y*accel_y +accel_z*accel_z, 0.5);
//    magnitude = alpha_beta_mag(0.960434,0.397825,accel_x,accel_y);
//    magnitude = alpha_beta_mag(0.960434,0.397825,magnitude,accel_z);

    // Use character pointers to send one byte at a time
    char *xpointer=(char *)&accel_x;
    char *ypointer=(char *)&accel_y;
    char *zpointer=(char *)&accel_z;

    UART_putCharacter(0x55);            // Send header
    for(i = 0; i < 4; i++) {            // Send x percentage - one byte at a time
        UART_putCharacter(xpointer[i]);
    }
    for(i = 0; i < 4; i++) {            // Send y percentage - one byte at a time
        UART_putCharacter(ypointer[i]);
    }
    for(i = 0; i < 4; i++) {            // Send y percentage - one byte at a time
        UART_putCharacter(zpointer[i]);
    }

    return magnitude;
}


void main(void) {
    WDTCTL = WDT_ADLY_16; // 1 second interval
    IE1 |= WDTIE;                             // Enable WDT interrupt
    TimerA_setup();                     // Setup timer to send ADC data
    TimerB_setup();
    ADC_setup();                        // Setup ADC
    P2DIR |= BIT0 | BIT3 | BIT6 | BIT7;
    P2OUT &= ~(BIT0 | BIT3 | BIT6 | BIT7);
//    UART_setup();                       // Setup UART for RS-232
    _EINT();
    float deltaAngle = 0;
    float accel_x, accel_y, accel_z;
    float setPoint = 0;

    while (1){
        ADC12CTL0 |= ADC12SC;               // Start conversions
        __bis_SR_register(LPM0_bits + GIE); // Enter LPM0

        accel_x = ((ADC_x*3.3/4095-1.65)/0.3);
        accel_y = ((ADC_y*3.3/4095-1.65)/0.3);
        accel_z = ((ADC_z*3.3/4095-1.65)/0.3);

        angle = (atan2f(accel_z,accel_y)*180/M_PI) + 180; // process variable PV(t)

        deltaAngle = setPoint - angle; // manipulated variable MV(t)
        //PID control loop
        while(deltaAngle){
            if(deltaAngle > 0){
                // move motor clockwise?

                rotate_by_angle(deltaAngle)
            }
            else{
                // move motor counterclockwise?
            }
        }
    }
}

#pragma vector = ADC12_VECTOR
__interrupt void ADC12ISR(void) {
    ADC_x = ADC12MEM0;      // Move results, IFG is cleared
    ADC_y = ADC12MEM1;
    ADC_z = ADC12MEM2;

    __bic_SR_register_on_exit(LPM0_bits); // Exit LPM0
}


#pragma vector = TIMERA0_VECTOR
__interrupt void timerA_isr() {
    static int i = 0;

    if(i == 2048){
        if(direction == 1){
            direction = 0;

        }
        else{
            direction = 1;
        }
        TACCR0 = 200;
        IE1 |= WDTIE;
        i = 0;
    }
    else{
        i++;
    }

    if(direction == 1){
            CW_motion();
        }
        else{
            CCW_motion();
        }
        LPM0_EXIT;
}

#pragma vector = WDT_VECTOR
__interrupt void watchdog_timer(void) {
    static int i = 0;

    if(i == 15){
        i = 0;
        IE1 &= ~WDTIE;
    }
    else{
        TACCR0 -= 9;
        i++;
    }

}
