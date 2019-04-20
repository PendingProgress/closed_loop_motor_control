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

void TimerA_setup(void) {
    TACCR0 = 200;                      // 200 / 32768 Hz = 6.1 ms
    TACTL = TASSEL_1 + MC_1;            // ACLK, up mode
//    TACCTL0 = CCIE;                     // Enabled interrupt
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


void step_motor2(char direction){
    int i = 0;
    P2OUT &= ~(BIT0 | BIT3 | BIT6 | BIT7);
    if(direction){
        for(i = 0; i < 5; i++){
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
        }
    }
    else{
        for(i = 0; i < 5; i++){
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
        }
    }
}
void step_motor_cw(){
    static int i = 0;
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
void step_motor_ccw(){
    static int i = 4;
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
}
void main(void) {
    WDTCTL = WDT_ADLY_16; // 1 second interval
//    IE1 |= WDTIE;                             // Enable WDT interrupt
    TimerA_setup();
//    TimerB_setup();
    ADC_setup();                        // Setup ADC
    P2DIR |= BIT0 | BIT3 | BIT6 | BIT7;
    P2OUT &= ~(BIT0 | BIT3 | BIT6 | BIT7);
//    UART_setup();                       // Setup UART for RS-232
    float deltaAngle = 0;
    float accel_y, accel_z,angle;
    float setPoint = 0;
    int steps;
    _EINT();


    while (1){
        ADC12CTL0 |= ADC12SC;               // Start conversions
        __bis_SR_register(LPM0_bits); // Enter LPM0

//        accel_x = ((ADC_x*3.3/4095-1.65)/0.3);
        accel_y = ((ADC_y*3.3/4095-1.65)/0.3);
        accel_z = ((ADC_z*3.3/4095-1.65)/0.3);

        angle = (atan2f(accel_z,accel_y)*180/M_PI) + 180; // process variable PV(t)

        deltaAngle = setPoint - angle; // manipulated variable MV(t)

        // 1 full rotation is 2048 "steps". One step is 4 pulses to the motor. function uses "full step" mode
        // steps is calculated using angle. 2048 steps = 360 degrees.
        // for 1 degree resolution. we step 2048/360 = 5.689 ~= 5
        steps = 2048/360*deltaAngle;
//        steps = 2048;
        TACCTL0 = CCIE;     // Enable timer A interrupt for PWM
        TACCR0 = 200;
        IE1 |= WDTIE;
        //PID control loop
        while(steps){
            if(steps < 0){
                // move motor clockwise?
                step_motor_cw();
//                rotate_by_angle(deltaAngle);
                steps++;
            }
            else{
                step_motor_ccw();
                steps--;
                // move motor counterclockwise?
            }
            __bis_SR_register(LPM0_bits);

//            __bis_SR_register(LPM0_bits + GIE); // Enter LPM0
        }
        //disable timer A interrupt until next PID loop
        TACCTL0 &= ~CCIE;
//        __no_operation();
//        __no_operation();
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
    LPM0_EXIT;
}

#pragma vector = WDT_VECTOR
__interrupt void watchdog_timer(void) {
    // accelerates motor over time to avoid stalling
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

