/*
  MIT License

  Copyright (c) 2018 Antonio Alexander Brewer (tonton81) - https://github.com/tonton81

  Contributors:
  Tim - https://github.com/Defragster
  Mike - https://github.com/mjs513

  Designed and tested for PJRC Teensy 3.2, 3.5, 3.6, and LC boards.

  Forum link : https://forum.pjrc.com/threads/54548-TSPISlave-Library

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and / or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.
*/

#include "Arduino.h"
#include "TSPISlave.h"
#include "SPI.h"
TSPISlave* TSPISlave::spi_slaves[3] = { nullptr, nullptr, nullptr };
void tspi0_isr(void);
void tspi1_isr(void);
void tspi2_isr(void);

int TSPISlave::getPortNo(SPIClass &_port) {
  if (&_port == &SPI) return 0;
#if defined(KINETISL) || defined(__MK64FX512__) || defined(__MK66FX1M0__)
  if (&_port == &SPI1) return 1;
#endif
#if defined(__MK64FX512__) || defined(__MK66FX1M0__)
  if (&_port == &SPI2) return 2;
#endif
  return -1;
}

TSPISlave::TSPISlave(SPIClass& _port, uint8_t _miso, uint8_t _mosi, uint8_t _sck, uint8_t _cs, uint8_t _fmsz):
  port(&_port),
  portno(getPortNo(_port)),
  mosi(setSlaveMOSI(_mosi)),
  miso(setSlaveMISO(_miso)),
  cs(setSlaveCS(_cs)),
  sck(setSlaveSCK(_sck))
  {
    ( _fmsz <= 8 ) ? _fmsz = 8 : _fmsz = 16;

    if (!valid()) return;

    switch (portno) {
      case 0: {
#if defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__)
        SIM_SCGC6 |= SIM_SCGC6_SPI0; // enable slave clock
        spi_map = 0x4002C000;
#elif defined(KINETISL)
        SIM_SCGC4 |= SIM_SCGC4_SPI0; // enable slave clock
        SIM_SCGC5 |= SIM_SCGC5_PORTA | SIM_SCGC5_PORTC; // enable ports
        spi_map = 0x40076000;
#endif
        spi_irq = IRQ_SPI0;
        _VectorsRam[16 + IRQ_SPI0] = tspi0_isr;
        spi_slaves[0] = this;
        break;
      }
      case 1: {
#if defined(__MK64FX512__) || defined(__MK66FX1M0__)
        SIM_SCGC6 |= SIM_SCGC6_SPI1; // enable slave clock
        spi_map = 0x4002D000;
#elif defined(KINETISL)
        SIM_SCGC4 |= SIM_SCGC4_SPI1; // enable slave clock
        SIM_SCGC5 |= SIM_SCGC5_PORTE; // enable port
        spi_map = 0x40077000;
#endif
        spi_irq = IRQ_SPI1;
        _VectorsRam[16 + IRQ_SPI1] = tspi1_isr;
        spi_slaves[1] = this;
        break;
      }
      case 2: {
        SIM_SCGC3 |= SIM_SCGC3_SPI2; // enable slave clock
        spi_map = 0x400AC000;
        spi_irq = IRQ_SPI2;
        _VectorsRam[16 + IRQ_SPI2] = tspi2_isr;
        spi_slaves[2] = this;
        break;
      }
    }

  #if defined(KINETISL)
    (*(KINETISL_SPI_t *)spi_map).C1 = ((uint8_t)0b00000000); // disable spi
    (*(KINETISL_SPI_t *)spi_map).C2 = ((uint8_t)(( _fmsz == 8 ) ? 0x00 : 0x40));  //((uint8_t)0b01000000);
    (*(KINETISL_SPI_t *)spi_map).BR = ((uint8_t)0b00000000);
    (*(KINETISL_SPI_t *)spi_map).C1 = ((uint8_t)0b11101100);
  #endif

  #if defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__)
    (*(KINETISK_SPI_t *)spi_map).MCR |= SPI_MCR_HALT | SPI_MCR_MDIS;
    (*(KINETISK_SPI_t *)spi_map).MCR = 0x00000000;
    (*(KINETISK_SPI_t *)spi_map).MCR &= ~SPI_MCR_HALT & ~SPI_MCR_MDIS;
    (*(KINETISK_SPI_t *)spi_map).CTAR0 = 0;
    (*(KINETISK_SPI_t *)spi_map).MCR |= SPI_MCR_HALT | SPI_MCR_MDIS;
    (*(KINETISK_SPI_t *)spi_map).CTAR0 = SPI_CTAR_FMSZ(_fmsz - 1);
    (*(KINETISK_SPI_t *)spi_map).MCR &= ~SPI_MCR_HALT & ~SPI_MCR_MDIS;
    (*(KINETISK_SPI_t *)spi_map).MCR |= SPI_MCR_HALT | SPI_MCR_MDIS;
    (*(KINETISK_SPI_t *)spi_map).CTAR0 = (*(KINETISK_SPI_t *)spi_map).CTAR0 & ~(SPI_CTAR_CPOL | SPI_CTAR_CPHA);
    (*(KINETISK_SPI_t *)spi_map).MCR &= ~SPI_MCR_HALT & ~SPI_MCR_MDIS;
    (*(KINETISK_SPI_t *)spi_map).RSER = 0x00020000;
  #endif

    NVIC_SET_PRIORITY(spi_irq, 1); // set priority
    NVIC_ENABLE_IRQ(spi_irq); // enable CS IRQ
  }

int TSPISlave::setSlaveMISO(uint8_t pin) {
  switch (portno) {
    case 0: {
      switch (pin) {
        case 8:
          CORE_PIN8_CONFIG = PORT_PCR_MUX(2);
          return pin;
        case 12:
          CORE_PIN12_CONFIG = PORT_PCR_MUX(2);
          return pin;
        case 39:
          CORE_PIN39_CONFIG = PORT_PCR_MUX(2);
          return pin;

      }
      break;
    }
    case 1: {
      switch (pin) {
        case 1:
          CORE_PIN1_CONFIG = PORT_PCR_MUX(2);
          return pin;
        case 5:
          CORE_PIN5_CONFIG = PORT_PCR_MUX(6);
          return pin;
      }
      break;
    }
    case 2: {
      switch (pin) {
        case 45:
          CORE_PIN45_CONFIG = PORT_PCR_MUX(2);
          return pin;
        case 51:
          CORE_PIN51_CONFIG = PORT_PCR_MUX(2);
          return pin;
      }
      break;
    }
  }
  return -1;
}

int TSPISlave::setSlaveMOSI(uint8_t pin) {
  switch (portno) {
    case 0: {
      switch (pin) {
        case 7:
          CORE_PIN7_CONFIG = PORT_PCR_MUX(2);
          return pin;
        case 11:
          CORE_PIN11_CONFIG = PORT_PCR_MUX(2);
          return pin;
      }
      break;
    }
    case 1: {
      switch (pin) {
        case 0:
          CORE_PIN0_CONFIG = PORT_PCR_MUX(2);
          return pin;
        case 21:
          CORE_PIN21_CONFIG = PORT_PCR_MUX(7);
          return pin;
      }
      break;
    }
    case 2: {
      switch (pin) {
        case 44:
          CORE_PIN44_CONFIG = PORT_PCR_MUX(2);
          return pin;
        case 52:
          CORE_PIN52_CONFIG = PORT_PCR_MUX(2);
          return pin;
      }
      break;
    }
  }
  return -1;
}

int TSPISlave::setSlaveCS(uint8_t pin) {
  switch (portno) {
    case 0: {
      switch (pin) {
        case 2:
          CORE_PIN2_CONFIG = PORT_PCR_MUX(2);
          return pin;
        case 10:
          CORE_PIN10_CONFIG = PORT_PCR_MUX(2);
          return pin;
      }
      break;
    }
    case 1: {
      switch (pin) {
        case 6:
          CORE_PIN6_CONFIG = PORT_PCR_MUX(2);
          return pin;
        case 31:
          CORE_PIN31_CONFIG = PORT_PCR_MUX(7);
          return pin;
      }
      break;
    }
    case 2: {
      switch (pin) {
        case 43:
          CORE_PIN43_CONFIG = PORT_PCR_MUX(2);
          return pin;
        case 55:
          CORE_PIN55_CONFIG = PORT_PCR_MUX(2);
          return pin;
      }
      break;
    }
  }
  return -1;
}

int TSPISlave::setSlaveSCK(uint8_t pin) {
  switch (portno) {
    case 0: {
      switch (pin) {
        case 13:
          CORE_PIN13_CONFIG = PORT_PCR_MUX(2);
          return pin;
        case 14:
          CORE_PIN14_CONFIG = PORT_PCR_MUX(2);
          return pin;
      }
      break;
    }
    case 1: {
      switch (pin) {
        case 20:
          CORE_PIN20_CONFIG = PORT_PCR_MUX(7);
          return pin;
        case 32:
          CORE_PIN32_CONFIG = PORT_PCR_MUX(2);
          return pin;
      }
      break;
    }
    case 2: {
      switch (pin) {
        case 46:
          CORE_PIN46_CONFIG = PORT_PCR_MUX(2);
          return pin;
        case 53:
          CORE_PIN53_CONFIG = PORT_PCR_MUX(2);
          return pin;
      }
      break;
    }
  }
  return -1;
}

void TSPISlave::onReceive(_spi_ptr handler) {
  _spihandler = handler;
}

bool TSPISlave::valid() const {
  return !(
    mosi < 0 ||
    miso < 0 ||
    sck < 0 ||
    cs < 0 ||
    portno < 0
  );
}

bool TSPISlave::available() {
#if defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__)
  return (*(KINETISK_SPI_t *)spi_map).SR & 0xF0;
#elif defined(KINETISL)
  (*(KINETISL_SPI_t *)spi_map).C1 &= ~SPI_C1_SPIE;
  return ((*(KINETISL_SPI_t *)spi_map).S & SPI_S_SPRF || (*(KINETISL_SPI_t *)spi_map).S & SPI_S_SPTEF);
#endif
}

bool TSPISlave::active() {
  return !digitalRead(cs);
}

void TSPISlave::pushr(uint16_t data) {
#if defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__)
  (*(KINETISK_SPI_t *)spi_map).PUSHR = data;
#elif defined(KINETISL)
  (*(KINETISL_SPI_t *)spi_map).DH = data >> 8;
  (*(KINETISL_SPI_t *)spi_map).DL = data;
  (*(KINETISL_SPI_t *)spi_map).C1 &= ~SPI_C1_SPTIE;
#endif
}

uint16_t TSPISlave::popr() {
#if defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__)
  return (*(KINETISK_SPI_t *)spi_map).POPR;
#elif defined(KINETISL)
  return (((*(KINETISL_SPI_t *)spi_map).DH) << 8 | ((*(KINETISL_SPI_t *)spi_map).DL));
#endif
  return 1;
}

void tspi0_isr(void) {
  if ( TSPISlave::spi_slaves[0] && TSPISlave::spi_slaves[0]->_spihandler ) {
    TSPISlave::spi_slaves[0]->_spihandler();
  }

#if defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__)
  else {
    SPI0_PUSHR_SLAVE = 0;
    SPI0_POPR;
  }
  SPI0_SR |= SPI_SR_RFDF;

#elif defined(KINETISL)
  else {
    KINETISL_SPI0.DH;
    KINETISL_SPI0.DL;
    KINETISL_SPI0.DH = 0;
    KINETISL_SPI0.DL = 0;
  }
  KINETISL_SPI0.C1 &= ~SPI_C1_SPTIE;
  KINETISL_SPI0.C1 |= SPI_C1_SPIE;
#endif
}

void tspi1_isr(void) {
  if ( TSPISlave::spi_slaves[1] && TSPISlave::spi_slaves[1]->_spihandler ) {
    TSPISlave::spi_slaves[1]->_spihandler();
  }
#if defined(__MK64FX512__) || defined(__MK66FX1M0__)
  else {
    SPI1_PUSHR_SLAVE = 0;
    SPI1_POPR;
  }
  SPI1_SR |= SPI_SR_RFDF;

#elif defined(KINETISL)
  else {
    KINETISL_SPI1.DH;
    KINETISL_SPI1.DL;
    KINETISL_SPI1.DH = 0;
    KINETISL_SPI1.DL = 0;
  }
  KINETISL_SPI1.C1 &= ~SPI_C1_SPTIE;
  KINETISL_SPI1.C1 |= SPI_C1_SPIE;
#endif
}

#if defined(__MK64FX512__) || defined(__MK66FX1M0__)
void tspi2_isr(void) {
  if ( TSPISlave::spi_slaves[2] && TSPISlave::spi_slaves[2]->_spihandler ) {
    TSPISlave::spi_slaves[2]->_spihandler();
  }
  else {
    SPI2_PUSHR_SLAVE = 0;
    SPI2_POPR;
  }
  SPI2_SR |= SPI_SR_RFDF;
}
#endif
