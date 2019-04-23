/*--------------------------------------------------------
* File: Final Project (main.c)
* Description: Closed loop control of a 28BYJ-89 Stepper Motor using an accelerometer
* Input: Accelerometer
* Output: PWM signal to ULN2003 that drives the motor
* Author: Jonathan Falwell
* Lab Section: 01
* Date: 04/23/2019
*--------------------------------------------------------*/
#include  <msp430xG46x.h>
#include <math.h>
#include <stdlib.h>

char direction = 0; // sets spin direction of motor
volatile long int ADC_x, ADC_z;

float deltaAngle = 0;
float MV;
float accel_x, accel_z,angle;
float setPoint = 90;
float delta_t = 0;
float last_t = 0;
float current_t = 0;
float millis = 0;
float last_millis = 0;
int motor_speed=0;
int min_period = 100;
int max_period = 1000;

void TimerA_setup(void) {
    TACCR0 = min_period;
    TACTL = TASSEL_1 + MC_1; // ACLK, up mode
    TACCTL0 = CCIE;          // enable masked interrupt
}
void TimerB_setup(void) {
    TB0CCTL0 = CCIE;           // enable masked interrupt
    TB0CCR0 = 32767;           // Set TB0 (and maximum) count value
    TB0CTL = TBSSEL_1 | MC_1;  // ACLK is clock source, UP mode

    TB0CCTL5 = CM_3 | CCIS_3 | SCS | CAP | CCIE; // TB5 setup for software triggered capture mode
}
void ADC_setup(void) {
    int i =0;

    P6DIR &= ~BIT4 + ~BIT6;              // Configure P6.3 and P6.7 as input pins
    P6SEL |= BIT4 + BIT6;                 // Configure P6.3 and P6.7 as analog pins
    ADC12CTL0 = ADC12ON + SHT0_6 + MSC;          // configure ADC converter
    ADC12CTL1 = SHP + CONSEQ_1;                  // Use sample timer, single sequence
    ADC12MCTL0 = INCH_4 + SREF_0;                // ADC A4 pin - Accel X-axis
    ADC12MCTL1 = INCH_6 + + SREF_0 + EOS;        // ADC A6 pin - Accel Z-axis
                                                 // EOS - End of Sequence for Conversions
    ADC12IE |= 0x02;                    // Enable ADC12IFG.1
    for (i = 0; i < 0x3600; i++);       // Delay for reference start-up
    ADC12CTL0 |= ENC;                   // Enable conversions
}

void step_motor_cw(){
    // function to step motor cw by sending pulses to ULN2003
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
    // function to step motor ccw by sending pulses to ULN2003
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

    WDTCTL = WDTPW + WDTHOLD;   // Stop WDT
    TimerA_setup();
    TimerB_setup();
    ADC_setup();

    // Setup Motor Pins
    P2DIR |= BIT0 | BIT3 | BIT6 | BIT7;
    P2OUT &= ~(BIT0 | BIT3 | BIT6 | BIT7);

    float error = 0;
    float integral = 0;
    float derivative = 0;
    float Kp = 10;
    float Ki = 0;
    float Kd = 0;

    float previous_error = 0;

    while (1){
        ADC12CTL0 |= ADC12SC;               // Start conversions

        __bis_SR_register(LPM0_bits+GIE); // Enter LPM0

        //time keeping for Integral and Derivative terms
        current_t = current_t + millis; // add millis to current_t
        delta_t = current_t - last_t;
        last_t = current_t;

        // calculate angle from accelerometer
        accel_x = round_one_decimal(((ADC_x*3.3/4095-1.65)/0.3));
        accel_z = round_one_decimal(((ADC_z*3.3/4095-1.65)/0.3));
        angle = (atan2f(accel_z,accel_x)*180/M_PI); // process variable PV(t)

        // calculate terms for MV
        error = setPoint - angle;
        integral = integral + error * delta_t;
        derivative = (error - previous_error) / delta_t;
        previous_error = error;

        MV = Kp * error + Ki * integral + Kd * derivative; // manipulated variable MV(t)

        //set motor direction based on sign of MV
        if (MV < 0){
            direction = 0;
        }
        else{
            direction = 1;
        }


        motor_speed = abs(min_period * 180 / MV);
        // set limit based on experimentation
        if (motor_speed < min_period){
            motor_speed = min_period;
        }
        // for very large values(slow motor speed) set motor speed to 0
        else if (motor_speed > 30000 || motor_speed < -30000){
            motor_speed = 0;
        }
        // set limit based on experimentation
        else if (motor_speed > max_period){
            motor_speed = max_period;
        }

        // set period of the PWM signal
        TACCR0 = motor_speed;

    }
}

#pragma vector = ADC12_VECTOR
__interrupt void ADC12ISR(void) {
    // get ADC12 data and toggle TB5 software capture
    ADC_x = ADC12MEM0;      // Move results, IFG is cleared
    ADC_z = ADC12MEM1;
    TB0CCTL5 ^= CCIS_1; // toggle CCIS0 bit to initiate software capture

}

#pragma vector = TIMERB0_VECTOR
__interrupt void timerB_isr() {
    // seconds
    static int i = 1;
    current_t = i;
    i++;
}
#pragma vector = TIMERB1_VECTOR
__interrupt void timerB1_isr() {
    // milliseconds
    millis = (float)TBCCR5/32767 - last_millis;
    if(millis < 0) // crude method of handling when CCR5 rolls over and millis goes negative
    {
        millis = 0;
    }
    last_millis = (float)TBCCR5/32767;
    TB0CCTL5 &= ~CCIFG;

    __bic_SR_register_on_exit(LPM0_bits);  // Exit LPMx, interrupts enabled
}

#pragma vector = TIMERA0_VECTOR
__interrupt void timerA_isr() {
    // Timer A controls motor speed and direction.
    // Timer A frequency is dynamically controlled by PID loop
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

