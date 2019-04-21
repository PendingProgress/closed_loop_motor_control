/*
 * TODO:
 *  - remove or re-implement acceleration ramp up using WDT_interrupt
 */

#include  <msp430xG46x.h>
#include <math.h>
#include <stdlib.h>
//#include  <stdio.h>
#define POS4 BIT7
char direction = 0; // sets spin direction of motor
volatile long int ADC_x, ADC_y, ADC_z;

float deltaAngle = 0;
float MV;
float accel_y, accel_z,angle;
float setPoint = 90;
float p_gain = 10;
int steps;
int motor_speed=0;
int min_period = 100;
int max_period = 1000;

void TimerA_setup(void) {
    TACCR0 = min_period;                      // 200 / 32768 Hz = 6.1 ms
    TACTL = TASSEL_1 + MC_1;            // ACLK, up mode
//    TACCTL0 = CCIE;                     // Enabled interrupt
}
void TimerB_setup(void) {

    TB0CCR0 = 3276;                // Set TB0 (and maximum) count value
    TB0CTL = TBSSEL_1 | MC_1;     // ACLK is clock source, UP mode

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

float round_one_decimal(float var)
{
    // returns float rounded to 1 decimal place
    float value = (int)(var * 10 + .5);
    return (float)value / 10;
}

void main(void) {
    WDTCTL = WDT_ADLY_16; // 1 second interval
//    IE1 |= WDTIE;                             // Enable WDT interrupt
    TimerA_setup();
    TimerB_setup();
    ADC_setup();                        // Setup ADC
    P2DIR |= BIT0 | BIT3 | BIT6 | BIT7;
    P2OUT &= ~(BIT0 | BIT3 | BIT6 | BIT7);
//    UART_setup();                       // Setup UART for RS-232

    TACCTL0 = CCIE;
//    _EINT();


    while (1){
        ADC12CTL0 |= ADC12SC;               // Start conversions
        __bis_SR_register(LPM0_bits+GIE); // Enter LPM0

        accel_y = round_one_decimal(((ADC_y*3.3/4095-1.65)/0.3));
        accel_z = round_one_decimal(((ADC_z*3.3/4095-1.65)/0.3));
//        accel_y = (((ADC_y*3.3/4095-1.65)/0.3));
//        accel_z = (((ADC_z*3.3/4095-1.65)/0.3));

        angle = (atan2f(accel_z,accel_y)*180/M_PI); // process variable PV(t)

        //debug if statement to set break point for larger than unexpected angles
    //    if(angle > 400){
    //        __no_operation();
    //    }
        MV = p_gain*(setPoint - angle); // manipulated variable MV(t)
        if (MV < 0){
            direction = 0;
        }
        else{
            direction = 1;
        }
        // 1 full rotation is 2048 "steps". One step is 4 pulses to the motor. function uses "full step" mode
        // steps is calculated using angle. 2048 steps = 360 degrees.
        // for 1 degree resolution. we step 2048/360 = 5.689 ~= 5
    //        steps = 2048/360*deltaAngle;

        motor_speed = abs(min_period * 180 / MV);
        if (motor_speed < min_period){
            motor_speed = min_period;
        }
        else if (motor_speed > 30000){
            motor_speed = 0;
        }
        else if (motor_speed > max_period){
            motor_speed = max_period;
//            __no_operation();
        }

    //        steps = 2048;

        TACCR0 = motor_speed;
//        TB0CCTL0 &= ~CCIE;
//        __bis_SR_register(LPM0_bits); // Enter LPM0
    }
}

#pragma vector = ADC12_VECTOR
__interrupt void ADC12ISR(void) {
    ADC_x = ADC12MEM0;      // Move results, IFG is cleared
    ADC_y = ADC12MEM1;
    ADC_z = ADC12MEM2;
    TB0CCTL0 = CCIE;
//    __bic_SR_register_on_exit(LPM0_bits); // Exit LPM0
}

#pragma vector = TIMERB0_VECTOR
__interrupt void timerB_isr() {
    TB0CCTL0 &= ~CCIE;
    LPM0_EXIT;
}
#pragma vector = TIMERA0_VECTOR
__interrupt void timerA_isr() {
    //PID control loop
    if(MV != 0){
        if(direction == 0){
            // move motor clockwise?
            step_motor_cw();
        }
        else{
            // move motor counterclockwise?
            step_motor_ccw();
        }
    }
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
