/*
  This program is the basis for all of the wheel labs
 */

#include <p30f4011.h>

#include <libpic30.h>
#include <delay.h>
#include <adc10.h>
#include <pwm.h>
#include <uart.h>
#include <string.h>
#include <stdio.h>
#include <qei.h>
#include <timer.h>
#include <ports.h>

// Configuration Bits
#pragma config FPR = FRC_PLL16   // 117.92 MHz
#pragma config FOS = PRI
#pragma config FCKSMEN = CSW_FSCM_OFF
#pragma config WDT = WDT_OFF
#pragma config FPWRT = PWRT_16
#pragma config BODENV = BORV27
#pragma config BOREN = PBOR_OFF
#pragma config MCLRE = MCLR_EN
#pragma config GWRP = GWRP_OFF


#define max(A ,B) ((A) > (B) ? (A) : (B))
#define min(A, B) ((A) < (B) ? (A) : (B))

#define PERIOD 14739 // for 1000 Hz pwm frequency
#define MAX_DUTY 2*PERIOD
#define MAX_COUNT 1
#define PI 3.14159265
#define MAXCNT 719  // maximum count for QEI encoders before interrupt, 720 counts per revolution
#define MAX_ISUM 1130

// define some variables

unsigned int AD_value;
unsigned int GO;
int encindex, p1, p2, r1, r2;

/***************************************************************/

// external input interrupt handler

void __attribute__((interrupt, no_auto_psv)) _INT1Interrupt(void) {
    unsigned int dutycyclereg, dutycycle;
    char updatedisable;

    // turn off the pwm signal

    dutycycle = (unsigned int) 0;
    dutycyclereg = 3;
    updatedisable = 0;
    SetDCMCPWM(dutycyclereg, dutycycle, updatedisable); // duty cycle set to low

    // turn off the LED to indicate power if off

    LATFbits.LATF6 = 0; // signal the power is off

    // Disable the interrupt

    DisableINT1;

    // now just wait

    while (1);
}

/************************************************************/

// Initialize external interrupt 1

void Init_INT1(void) {
    unsigned int config;

    config = FALLING_EDGE_INT & // interrupt on a falling edge
            EXT_INT_ENABLE & // enable the interrupts
            //EXT_INT_PRI_0 ;
            GLOBAL_INT_ENABLE;

    ConfigINT1(config);


    // turn on the LED to show interrupt is set

    TRISFbits.TRISF6 = 0;
    LATFbits.LATF6 = 1; // signal the interrupt is set

    // prepare for an input on RD0

    TRISDbits.TRISD0 = 1;

    // enable the interrupt

    DisableINT1;
    EnableINT1;

    return;
}

/**********************************************************/

// timer 1 interrupt handler

void __attribute__((interrupt, auto_psv)) _T1Interrupt(void) {
    unsigned int ReadQEI(void);
    extern unsigned int AD_value;
    extern unsigned int GO;
    extern int p2, r2, encindex;

    // read from the A/D channel

    ADCON1bits.SAMP = 1; // start the sampling
    while (!ADCON1bits.DONE);
    AD_value = ReadADC10(0);

    // update the position variables

    p2 = (int) ReadQEI();
    r2 = encindex;

    // reset Timer 1 interrupt flag 

    IFS0bits.T1IF = 0;

    // if GO is 1 we are not done before the next interrupt!

    if (GO == 1)
        LATEbits.LATE1 = 1;

    GO = 1;
}

/***********************************************************/

// Initialize timer 1

void Init_Timer1(unsigned int period) {
    unsigned int config;

    config = T1_INT_PRIOR_4 & // set interrupt priority to 2
            T1_INT_ON; // enable the interrupts

    ConfigIntTimer1(config);

    config = T1_ON & // turn on the timer
            T1_IDLE_CON & // operate during sleep
            T1_GATE_OFF & // timer gate accumulation is disabled
            T1_PS_1_256 & // timer prescale is 256
            T1_SYNC_EXT_OFF & // don't synch with external clock
            T1_SOURCE_INT; // use the internal clock

    OpenTimer1(config, period);

    TRISEbits.TRISE1 = 0; // prepare for the overrun LED indicator
    LATEbits.LATE1 = 0; // the LED should be off

    return;
}

/***************************************************/

// QEI interrupt handler

void __attribute__((interrupt, no_auto_psv)) _QEIInterrupt(void) {
    extern int encindex;

    // update the encoder count every time the counter POSCNT gets to MAXCNT

    if (QEICONbits.UPDN) {
        encindex++;
    }
    else {
        encindex--;
    }

    IFS2bits.QEIIF = 0; // 
}

/***********************************************/

// setup the QEI encoder

void encoder_init(void) {

    unsigned int config1, config2;

    config1 = QEI_DIR_SEL_QEB &
            QEI_INT_CLK &
            QEI_INDEX_RESET_DISABLE & // QEI index pulse resets postion counter 
            QEI_CLK_PRESCALE_1 &
            QEI_GATED_ACC_DISABLE &
            QEI_NORMAL_IO &
            QEI_INPUTS_NOSWAP &
            QEI_MODE_x2_MATCH & // reset on match
            QEI_DOWN_COUNT & // count up
            QEI_IDLE_CON; // continue on idle

    config2 = POS_CNT_ERR_INT_DISABLE & // disable error interrupts
            QEI_QE_CLK_DIVIDE_1_1 & //1_256
            QEI_QE_OUT_ENABLE & // enable digital filter
            MATCH_INDEX_INPUT_PHASEA &
            MATCH_INDEX_INPUT_LOW;

    OpenQEI(config1, config2);

    config1 = QEI_INT_ENABLE & // enable the interrupts
            QEI_INT_PRI_2; // set the priority to two

    WriteQEI((unsigned int) MAXCNT);

    ConfigIntQEI(config1);
}

/********************************************************/

// setup ADC10

void adc_init(void) {

    unsigned int config1, config2, config3, configport, configscan;

    ADCON1bits.ADON = 0; // turn off ADC

    SetChanADC10(
            ADC_CH0_NEG_SAMPLEA_VREFN & // negative reference for channel 0 is VREF negative
            ADC_CH0_POS_SAMPLEA_AN2 // 
            // ADC_CHX_NEG_SAMPLEA_VREFN &  // negative reference for channel 1 is VREF negative
            // ADC_CHX_POS_SAMPLEA_AN3AN4AN5
            );

    ConfigIntADC10(ADC_INT_DISABLE); // disable the interrupts

    config1 =
            ADC_MODULE_ON & //turn on ADC module
            ADC_IDLE_CONTINUE & // let it idle if not in use
            ADC_FORMAT_INTG & // unsigned integer format
            ADC_CLK_AUTO & // manual trigger source
            ADC_AUTO_SAMPLING_OFF & // do not continue sampling
            ADC_SAMPLE_SIMULTANEOUS & // sample both channels at the same time
            ADC_SAMP_ON; // enable sampling

    config2 =
            ADC_VREF_AVDD_AVSS & // voltage reference
            ADC_SCAN_OFF & // don't scan
            ADC_CONVERT_CH0 & // convert channel 0
            ADC_SAMPLES_PER_INT_1 & // 1 samples per interrupt
            ADC_ALT_BUF_OFF & // don't use the alternate buffer
            ADC_ALT_INPUT_OFF; // don't use an alternate input

    config3 =
            ADC_SAMPLE_TIME_2 & // auto sample time bits
            ADC_CONV_CLK_SYSTEM & // use the system clock
            ADC_CONV_CLK_13Tcy; // conversion clock speed (coeff of TCY is ADCS)

    configport =
            ENABLE_AN2_ANA; // parameters to be configured in the ADPCFG 

    configscan =
            SCAN_NONE; // scan select parameter for the ADCSSL register

    OpenADC10(config1, config2, config3, configport, configscan);
}

/*********************************************************/

// setup pwm

void pwm_init(void) {

    unsigned int config1, config2, config3;
    unsigned int sptime;

    config1 = PWM_INT_DIS & // disable the interrupt
            PWM_FLTA_DIS_INT; // disable the interrupt on fault

    ConfigIntMCPWM(config1);

    config1 = PWM_EN & //  enable the PWM module
            PWM_IPCLK_SCALE1 & // input prescaler set to 1
            PWM_OP_SCALE1 & // post scalar set to 1
            PWM_MOD_UPDN; // free running mode

    config2 = PWM_MOD1_IND & // pwm modules run independently
            PWM_MOD2_IND &
            PWM_MOD3_IND &
            PWM_PDIS1H & // disable 1 high
            PWM_PDIS2H & // disable 2 high
            PWM_PEN3H & // enable 3 high
            PWM_PDIS1L & // disable 1 low
            PWM_PDIS2L & // disable 2 low
            PWM_PDIS3L; // disable 3 low

    config3 = PWM_UEN; // enable updates

    sptime = 0x0;

    OpenMCPWM(PERIOD, sptime, config1, config2, config3);

}

/********************************************************/

// setup the UART

void uart1_init(void) {

    unsigned int config1, config2, ubrg;

    config1 = UART_EN & // enable the UART
            UART_IDLE_CON & // set idle mode
            UART_DIS_WAKE & // disable wake-up on start
            UART_DIS_LOOPBACK & // disable loopback
            UART_DIS_ABAUD & // disable autobaud rate detect
            UART_NO_PAR_8BIT & // no parity, 8 bits
            UART_1STOPBIT; // one stop bit

    config2 = UART_INT_TX_BUF_EMPTY & // interrupt anytime a buffer is empty
            UART_TX_PIN_NORMAL & // set transmit break pin
            UART_TX_ENABLE & // enable UART transmission
            UART_INT_RX_CHAR & // receive interrupt mode selected
            UART_ADR_DETECT_DIS & // disable address detect
            UART_RX_OVERRUN_CLEAR; // overrun bit clear

    ubrg = 15; // 115200 baud

    OpenUART1(config1, config2, ubrg);
}

/*****************************************************************/
//
// update an array
//

void update_array(double arr[], int N) {
    int k;
    for (k = N - 2; k >= 0; k--) arr[k + 1] = arr[k];
}

/*****************************************************************/
//
// implement an IIR filter
//

void filter(double A[], double B[], double fin[], double fout[], int N) {
    int i;
    fout[0] = B[0] * fin[0];
    for (i = 1; i < N; i++) {
        fout[0] += B[i] * fin[i] - A[i] * fout[i];
    }
}

/*****************************************************************/

int main(void) {
    extern unsigned int AD_value;
    extern unsigned int GO;
    extern int r1, r2, p1, p2;
    unsigned int dutycyclereg, dutycycle, period;
    char updatedisable;
    double convert_to_duty, time, dt;
    int count, int_time, int_Rin, int_Rout, int_Y, int_R, int_u;
    double arg, speed, speed_scale;
    double error, u;
    double Y, Yin[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
    double Yout[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
    double R, Rin[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
    double Rout[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
    double A[5] = {1.0, -2.3695, 2.3140, -1.0547, 0.1874}, B[5] = {0.0048, 0.0193, 0.0289, 0.0193, 0.0048};
    double Ac[5] = {1.0000, -0.6000, 0.0, 0.0, -0.4000};
    double Bc[5] = {18.9722, -18.6022, 0.0, 0.0, 0.0};
    double error_in[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
    double u_out[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
    int Nr = 5, Ny = 5, Nu = 5;
    double AD_scale = 0.1688;
    double kp = 2;
    double reference_scaling = 1;
    double MAX_DELTA_U = 1000.0;
    double last_u = 0.0;
    double ki = 0.075, kd = -10, Isum = 0;
    double Derror = 0, last_error = 0;

    // set up the external interrupt

    Init_INT1();

    //  set up the A/D parameters

    adc_init();

    // set up the pwm

    pwm_init();

    // set up the uart

    uart1_init();

    // disable updates

    updatedisable = 0;

    // get some scaling out of the way

    convert_to_duty = ((double) MAX_DUTY) / 1023.0;

    r1 = 0;
    r2 = 0;
    p1 = 0;
    p2 = 0;
    encindex = 0;

    // initialize timer1
    // dt can be no larger than 0.25 seconds

    dt = 0.05; // the sampling interval in seconds

    // dt = N*256/29,480,000;  assuming a 256 prescaler.
    // so N = dt* 115156

    period = (unsigned int) (dt * 115156.0);

    if (period > 32768) {
        period = 32768;
    }
    printf("....period is %6u (should be < 32768) \n ", period);

    Init_Timer1(period);

    AD_value = 0;
    time = -dt;

    // set up the encoder to read the speed of the wheel

    // enable the input for QEI

    TRISBbits.TRISB3 = 1;
    TRISBbits.TRISB4 = 1;
    TRISBbits.TRISB5 = 1;

    // we also need to set these bits for the QEI 

    ADPCFGbits.PCFG3 = 1;
    ADPCFGbits.PCFG4 = 1;
    ADPCFGbits.PCFG5 = 1;

    encoder_init();

    // enable the QEI interrupt

    EnableIntQEI;

    // convert QEI encoder readings to radians

    speed_scale = (2.0 * PI / 720.0) / dt;


    TRISEbits.TRISE1 = 0; // output when sampling too fast
    TRISEbits.TRISE3 = 0; // pwm for servo

    count = MAX_COUNT;
    GO = 0;

    /********************* MAIN LOOP **********************/

    while (1) {

        while (!GO);

        // update the time

        time = time + dt;

        /*********************************************/
        //  implement the PREFILTER (Gpf) functions
        //
        //  the reference signal is the value in AD_value, and is
        //  read every time the Timer1 interrupt happens.
        //
        /*********************************************/

        // convert value read from pot [0-1023] to a speed [rad/sec]

        update_array(Rout, Nr);
        update_array(Rin, Nr);
        Rin[0] = AD_value;
        filter(A, B, Rin, Rout, Nr);
        Rout[0] = max(0.0, Rout[0]);
        Rout[0] = min(Rout[0], 1023);
        R = Rout[0] * AD_scale;
//        if (time < 0.5) R = 0.0;
//        else if ((time >= 0.5) & (time < 5.5)) R = 10.0;
//        else if ((time >= 5.5) & (time < 10.5)) R = 25.0;
//        else if (time >= 10.5) R = 40.0;
//        R = 75;
        R = R * reference_scaling;

        /*********************************************/
        //  implement the FEEDBACK (H) functions
        //
        //  Even if H is not explicitly written, we still need to
        //  sample the output and convert it to the correct units.       
        //  For the wheel system, the units are radians/second
        // 
        /*********************************************/

        // get the raw speed of the wheel from the QEI data
        // the if statement prevents overflow

        if ((r2 - r1) < 0)
            arg = ((double) ((r2 - r1) + 65536))*720.0 + (double) (p2 - p1);
        else
            arg = ((double) (r2 - r1))*720.0 + (double) (p2 - p1);

        speed = arg*speed_scale;
        update_array(Yout, Nr);
        update_array(Yin, Nr);
        Yin[0] = speed;
        filter(A, B, Yin, Yout, Nr);
        Yout[0] = max(0.0, Yout[0]);
        Yout[0] = min(Yout[0], 1023);
        Y = Yout[0];

        /*********************************************/
        //  implement the ERROR computation
        //
        //  The error is the difference between the (possibly)
        //  modified reference signal and the (possibly modified)
        //  output
        //
        /*********************************************/

//        error = R; // use this for open loop control
        error = R - Y;  // use this for closed loop control

        /*********************************************/
        //  implement the CONTROLLER (Gc) functions
        //
        //  
        /*********************************************/

//        update_array(u_out, Nu);
//        update_array(error_in, Nu);
//        error_in[0] = error;
//        filter(Ac, Bc, error_in, u_out, Nu);
//        u = u_out[0];
        Derror = error-last_error;
        last_error = error;
        Isum = Isum + error;
        Isum = max(0.0, Isum);
        Isum = min(MAX_ISUM, Isum);
        u = kp * error + ki * Isum + kd * Derror;

        /*********************************************/
        // implement CONYTROl EFFORT CONVERSION
        //
        //  convert the control effort to the correct units for
        //  motor, be sure the control effort is within
        //  the allowable range
        //  
        /*********************************************/

        // scale = MAX_DUTY/1023
        // u/AD_scale corresponds to R/AD_scale
        // so u has units of MAX_DUTY/1023 * [0-1023]

        u = u * convert_to_duty / AD_scale; // convert back to a pwm signal
//        if (error > 0.0) u = 0.2 * MAX_DUTY;
//        else u = 0.0;
        u = min(u, last_u+MAX_DELTA_U);
        u = min(u, MAX_DUTY);
        u = max(u,0.0);
        last_u = u;

        dutycycle = (unsigned int) u;

        dutycyclereg = 3;
        SetDCMCPWM(dutycyclereg, dutycycle, updatedisable);

        /*********************************************/
        // 
        // prepare to print out. All values should be scaled by
        // 100 and converted to integers. 
        // COUNT indicates how many time periods to wait before
        // printing out, usually it is set to 1
        // 
        /*********************************************/

        if (--count == 0) {
            int_time = (int) (100.0 * time); // convert for printout
            int_R = (int) 100*R/reference_scaling;
            int_Y = (int) 100*Y;
            int_u = (int) u;

            printf("%8d %8d %8d %8d %8d\n", int_time, int_R, int_u, int_Y, (int) Isum);
            count = MAX_COUNT;
        }   
        // save the current positions

        r1 = r2;
        p1 = p2;

        GO = 0; // all done
    }

}
