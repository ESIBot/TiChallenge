#include "basics.h"



//*****************************************************************
// config()
//*****************************************************************

void config(){

// Parar Watchdog
  WDTCTL = WDTPW | WDTHOLD;

//Configura los pines PWM de los motores

  P1DIR |= BIT7;                       // P1.7 Como salida
  P2DIR |= BIT0;                       // P2.0 Como salida
  P1SEL |= BIT7;                       // Modo timer
  P2SEL |= BIT0;                       // Modo timer

  TA1CCR0 = 512-1;                          // PWM Period
  TA1CCTL1 = OUTMOD_7;                      // CCR1 reset/set
  TA1CCTL2 = OUTMOD_7;                      // CCR2 reset/set
  TA1CTL = TASSEL_2 | MC_1 | TACLR;         // SMCLK, up mode, clear TAR

//Configura el ADC y los pines de los emisores y receptores de IR
//usando adc10_10 de los ejemplos de code composer como referencia

  P1DIR |= BIT0 | BIT1 | BIT2 | BIT3 | BIT4 | BIT5 | BIT6 ;    //ATENCION: Los emisores conectados a P1_0 y P1_6 puede que cambien de pin
  P5DIR &= ((~BIT0) & (~BIT1) & (~BIT2) & (~BIT3));
  P6DIR &= ((~BIT0) & (~BIT1));


  //Comrpobar que con 16 ciclos le da tiempo a terminar la conversión
  ADC10CTL0 = ADC10SHT_2 | ADC10MSC | ADC10ON;  // 16ADCclks, MSC, ADC ON
  ADC10CTL1 = ADC10SHP | ADC10CONSEQ_1;  //ADC10 Sample/Hold Pulse Mode ; Conversión de los canales uno tras otro
  ADC10CTL2 &= ~ADC10RES; //Resolución 8 bits

  ADC10MCTL0 = ADC10INCH_9;  //Hay que indicar el canal máximo al que llega la secuencia de conversiones. ¿Qué pasa con los que no se usan?

  DMACTL0 = DMA0TSEL_24;                    // ADC10IFG trigger
  __data16_write_addr((unsigned short) &DMA0SA,(unsigned long) &ADC10MEM0);
                                            // Source single address
  __data16_write_addr((unsigned short) &DMA0DA,(unsigned long) &ADC_Result[0]);
                                            // Destination array address
  DMA0SZ = 0x0a;                            // 10 conversions
  DMA0CTL = DMADT_4 | DMADSTINCR_3 | DMASRCBYTE | DMADSTBYTE | DMAEN | DMAIE;
                                            // Rpt, inc dest, byte access,

//Configurar el bus i2c en uscib1

  P4SEL |= BIT1 | BIT2;
  UCB1CTL0 = UCMST | UCMODE_3 | UCSYNC;     // I2C Master, synchronous mode
  UCB1CTL1 = UCSSEL_2 | UCSWRST;            // Use SMCLK, enable SW reset
  UCB1BR0 = 12;                             // fSCL = SMCLK/12 = ~100kHz
  UCB1BR1 = 0;

  UCB1I2CSA = 0x02;                         // Slave Address is 048h
  UCB1CTL1 &= ~UCSWRST;                     // Clear SW reset, resume operation
  UCB1IE |= UCRXIE;                         // Enable RX interrupt


 }

//***********************************************************************
// applySensorGeometry()
//***********************************************************************

void applySensorGeometry(){

    int ir_rf_x[NUMSENSORES];
    int ir_rf_y[NUMSENSORES];
    int xs,ys,thetas;
    int i =0;

    for(i = 0; i<NUMSENSORES;i++){
        xs = sensor_placement_x[i];
        ys = sensor_placement_y[i];
        thetas = sensor_placement_theta[i];

        ir_rf_x[i] = ir_distances[i]*cos(thetas)+xs;   // ir_rf = R*(ir_distances,0,1);
        ir_rf_y[i] = ir_distances[i]*sin(thetas)+ys;
    }


    for(i = 0; i<NUMSENSORES;i++){
        ir_wf_x[i] = (cos(position[2])*ir_rf_x[i]) - (sin(position[2])* ir_rf_y[i]) + position[0];
        ir_wf_y[i] = (sin(position[2])*ir_rf_x[i]) + (cos(position[2])* ir_rf_y[i]) + position[1];
    }
}
