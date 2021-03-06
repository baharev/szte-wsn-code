// $Id: MicaBusC.nc,v 1.7 2010-06-15 21:24:16 mmaroti Exp $
/*
 * Copyright (c) 2005-2006 Intel Corporation
 * All rights reserved.
 *
 * This file is distributed under the terms in the attached INTEL-LICENSE     
 * file. If you do not find these files, copies can be found by writing to
 * Intel Research Berkeley, 2150 Shattuck Avenue, Suite 1300, Berkeley, CA, 
 * 94704.  Attention:  Intel License Inquiry.
 */
/**
 * A simplistic beginning to providing a standard interface to the
 * mica-family 51-pin bus. Just provides the PW0-PW7 and Int0-3 digital
 * I/O pins and returns the ADC channel number for the ADC pins.
 * @author David Gay
 */

configuration MicaBusC {
  provides {
    interface GeneralIO as PW0;
    interface GeneralIO as PW1;
    interface GeneralIO as PW2;
    interface GeneralIO as PW3;
    interface GeneralIO as PW4;
    interface GeneralIO as PW5;
    interface GeneralIO as PW6;
    interface GeneralIO as PW7;

    interface GeneralIO as Int0;
    interface GeneralIO as Int1;
    interface GeneralIO as Int2;
    interface GeneralIO as Int3;

    /* INT lines used as interrupt source */
    interface GpioInterrupt as Int0_Interrupt;
    interface GpioInterrupt as Int1_Interrupt;
    interface GpioInterrupt as Int2_Interrupt;
    interface GpioInterrupt as Int3_Interrupt;
    
    interface GeneralIO as USART1_CLK;
    interface GeneralIO as USART1_RXD;
    interface GeneralIO as USART1_TXD;

    /* Separate interfaces to allow inlining to occur */
    interface MicaBusAdc as Adc0;
    interface MicaBusAdc as Adc1;
    interface MicaBusAdc as Adc2;
    interface MicaBusAdc as Adc3;
    interface MicaBusAdc as Adc4;
    interface MicaBusAdc as Adc5;
    interface MicaBusAdc as Adc6;
    interface MicaBusAdc as Adc7;
    
    /*other pins as GPIO*/
    interface GeneralIO as UART0_RXD;
    interface GeneralIO as UART0_TXD;
    interface GeneralIO as SPI_SCK;
    interface GeneralIO as SPI_MOSI;
    interface GeneralIO as SPI_MISO;
    interface GeneralIO as I2C_CLK;
    interface GeneralIO as I2C_DATA;
    interface GeneralIO as Adc0_IO;
    interface GeneralIO as Adc1_IO;
    interface GeneralIO as Adc2_IO;
    interface GeneralIO as Adc3_IO;
    interface GeneralIO as Adc4_IO;
    interface GeneralIO as Adc5_IO;
    interface GeneralIO as Adc6_IO;
    interface GeneralIO as Adc7_IO;
  }
}
implementation {
  components AtmegaGeneralIOC as Pins, MicaBusP;
  components new NoPinC() as NoPW1, new NoPinC() as NoPW7, new NoPinC() as NoAdc;
  
  PW0 = Pins.PortB6;
  PW1 = NoPW1;
  PW2 = Pins.PortB7;
  PW3 = Pins.PortG0;
  PW4 = Pins.PortG1;
  PW5 = Pins.PortG2;
  PW6 = Pins.PortG5;
  PW7 = NoPW7;
  
  Int0 = Pins.PortE4;
  Int1 = Pins.PortE5;
  Int2 = Pins.PortE6;
  Int3 = Pins.PortE7;
  
  #if UCDUAL_REV==1
  USART1_CLK = Pins.PortE2;//hardware bug, this is the clk of usart0
  #else
  USART1_CLK = Pins.PortD5;
  #endif
  USART1_RXD = Pins.PortD3;
  USART1_TXD = Pins.PortD2;
  
  UART0_RXD = Pins.PortE0;
  UART0_TXD = Pins.PortE1;
  SPI_SCK = Pins.PortB1;
  SPI_MOSI = Pins.PortB2;
  SPI_MISO = Pins.PortB3;
  I2C_CLK = Pins.PortD0;
  I2C_DATA = Pins.PortD1;
  Adc0_IO = NoAdc;
  Adc1_IO = Pins.PortF1;
  Adc2_IO = Pins.PortF2;
  Adc3_IO = Pins.PortF3;
  Adc4_IO = Pins.PortF4;
  Adc5_IO = Pins.PortF5;
  Adc6_IO = Pins.PortF6;
  Adc7_IO = Pins.PortF7;

  components AtmegaExtInterruptC;
  Int0_Interrupt=AtmegaExtInterruptC.GpioInterrupt[4];
  Int1_Interrupt=AtmegaExtInterruptC.GpioInterrupt[5];
  Int2_Interrupt=AtmegaExtInterruptC.GpioInterrupt[6];
  Int3_Interrupt=AtmegaExtInterruptC.GpioInterrupt[7];

  #if UCDUAL_REV==1
  Adc0 = MicaBusP.Adc7;//not connected
  Adc1 = MicaBusP.Adc6;
  Adc2 = MicaBusP.Adc5;
  Adc3 = MicaBusP.Adc4;
  Adc4 = MicaBusP.Adc3;
  Adc5 = MicaBusP.Adc2;
  Adc6 = MicaBusP.Adc1;
  Adc7 = MicaBusP.Adc0;
  #else
  Adc0 = MicaBusP.Adc0;
  Adc1 = MicaBusP.Adc1;
  Adc2 = MicaBusP.Adc2;
  Adc3 = MicaBusP.Adc3;
  Adc4 = MicaBusP.Adc4;
  Adc5 = MicaBusP.Adc5;
  Adc6 = MicaBusP.Adc6;
  Adc7 = MicaBusP.Adc7;//not connected
  #endif
}
