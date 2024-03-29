/*
 * Copyright (C) 2005-2013 The Paparazzi Team
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */

/**
 * @file arch/stm32/mcu_periph/spi_arch.c
 * @ingroup stm32_arch
 *
 * Handling of SPI hardware for STM32.
 * SPI Master code.
 *
 * When a transaction is submitted:
 * - The transaction is added to the queue if there is space,
 *   otherwise it returns false
 * - The pending state is set
 * - SPI Interrupts (in this case the DMA interrupts) are disabled
 *   to prevent race conditions
 * - The slave is selected if required, then the before_cb callback is run
 * - The spi and dma registers are set up for the specific transaction
 * - Spi, DMA and interrupts are enabled and the transaction starts
 *
 * Obviously output_length and input_length will never both be 0 at the same time.
 * In this case, spi_submit will just return false.
 *
 * For the DMA and interrupts:
 * - If the output_len != input_len, a dummy DMA transfer is triggered for
 *   the remainder so the same amount of data is moved in and out.
 *   This simplifies keeping the clock going if output_len is greater and allows
 *   the rx dma interrupt to represent that the transaction has fully completed.
 * - The dummy DMA transfer is initiated at the transaction setup if length is 0,
 *   otherwise after the first dma interrupt completes in the ISR directly.
 * - The rx DMA transfer completed interrupt marks the end of a complete transaction.
 * - The after_cb callback happens BEFORE the slave is unselected as configured.
 */

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/dma.h>

#include "spi.h"
#include "gpio_arch.h"

#include "lisa_m_2.0.h"

#ifdef SPI_MASTER

/**
 * Libopencm3 specifc communication parameters for a SPI peripheral in master mode.
 */
struct locm3_spi_comm {
  uint32_t br;       ///< baudrate (clock divider)
  uint32_t cpol;     ///< clock polarity
  uint32_t cpha;     ///< clock phase
  uint32_t dff;      ///< data frame format 8/16 bits
  uint32_t lsbfirst; ///< frame format lsb/msb first
};

/**
 * This structure keeps track of specific config for each SPI bus,
 * which allows for more code reuse.
 */
struct spi_periph_dma {
  uint32_t spi;                    ///< SPI peripheral identifier
  uint32_t spidr;                  ///< SPI DataRegister address for DMA
  uint32_t dma;                    ///< DMA controller base address (DMA1 or DMA2)
  uint8_t  rx_chan;                ///< receive DMA channel number
  uint8_t  tx_chan;                ///< transmit DMA channel number
  uint8_t  rx_nvic_irq;            ///< receive interrupt
  uint8_t  tx_nvic_irq;            ///< transmit interrupt
  uint16_t tx_dummy_buf;           ///< dummy tx buffer for receive only cases
  bool_t tx_extra_dummy_dma;  ///< extra tx dummy dma flag for tx_len < rx_len
  uint16_t rx_dummy_buf;           ///< dummy rx buffer for receive only cases
  bool_t rx_extra_dummy_dma;  ///< extra rx dummy dma flag for tx_len > rx_len
  struct locm3_spi_comm comm; ///< current communication paramters
  uint8_t  comm_sig;               ///< comm config signature used to check for changes
};


#if USE_SPI0
#error "The STM32 doesn't have SPI0"
#endif
#if USE_SPI1
static struct spi_periph_dma spi1_dma;
#endif
#if USE_SPI2
static struct spi_periph_dma spi2_dma;
#endif
#if USE_SPI3
static struct spi_periph_dma spi3_dma;
#endif

static void spi_start_dma_transaction(struct spi_periph* periph, struct spi_transaction* _trans);
static void spi_next_transaction(struct spi_periph* periph);
static void spi_configure_dma(uint32_t dma, uint8_t chan, uint32_t periph_addr, uint32_t buf_addr,
                              uint16_t len, enum SPIDataSizeSelect dss, bool_t increment);
static void process_rx_dma_interrupt(struct spi_periph* periph);
static void process_tx_dma_interrupt(struct spi_periph* periph);
static void spi_arch_int_enable(struct spi_periph *spi);
static void spi_arch_int_disable(struct spi_periph *spi);


/******************************************************************************
 *
 * Handling of Slave Select outputs
 *
 *****************************************************************************/

static inline void SpiSlaveUnselect(uint8_t slave) {
  switch(slave) {
#if USE_SPI_SLAVE0
    case 0:
      gpio_set(SPI_SELECT_SLAVE0_PORT, SPI_SELECT_SLAVE0_PIN);
      break;
#endif // USE_SPI_SLAVE0
#if USE_SPI_SLAVE1
    case 1:
      gpio_set(SPI_SELECT_SLAVE1_PORT, SPI_SELECT_SLAVE1_PIN);
      break;
#endif //USE_SPI_SLAVE1
#if USE_SPI_SLAVE2
    case 2:
      gpio_set(SPI_SELECT_SLAVE2_PORT, SPI_SELECT_SLAVE2_PIN);
      break;
#endif //USE_SPI_SLAVE2
#if USE_SPI_SLAVE3
    case 3:
      gpio_set(SPI_SELECT_SLAVE3_PORT, SPI_SELECT_SLAVE3_PIN);
      break;
#endif //USE_SPI_SLAVE3
#if USE_SPI_SLAVE4
    case 4:
      gpio_set(SPI_SELECT_SLAVE4_PORT, SPI_SELECT_SLAVE4_PIN);
      break;
#endif //USE_SPI_SLAVE4
#if USE_SPI_SLAVE5
    case 5:
      gpio_set(SPI_SELECT_SLAVE5_PORT, SPI_SELECT_SLAVE5_PIN);
      break;
#endif //USE_SPI_SLAVE5
    default:
      break;
  }
}

static inline void SpiSlaveSelect(uint8_t slave) {
  switch(slave) {
#if USE_SPI_SLAVE0
    case 0:
      gpio_clear(SPI_SELECT_SLAVE0_PORT, SPI_SELECT_SLAVE0_PIN);
      break;
#endif // USE_SPI_SLAVE0
#if USE_SPI_SLAVE1
    case 1:
      gpio_clear(SPI_SELECT_SLAVE1_PORT, SPI_SELECT_SLAVE1_PIN);
      break;
#endif //USE_SPI_SLAVE1
#if USE_SPI_SLAVE2
    case 2:
      gpio_clear(SPI_SELECT_SLAVE2_PORT, SPI_SELECT_SLAVE2_PIN);
      break;
#endif //USE_SPI_SLAVE2
#if USE_SPI_SLAVE3
    case 3:
      gpio_clear(SPI_SELECT_SLAVE3_PORT, SPI_SELECT_SLAVE3_PIN);
      break;
#endif //USE_SPI_SLAVE3
#if USE_SPI_SLAVE4
    case 4:
      gpio_clear(SPI_SELECT_SLAVE4_PORT, SPI_SELECT_SLAVE4_PIN);
      break;
#endif //USE_SPI_SLAVE4
#if USE_SPI_SLAVE5
    case 5:
      gpio_clear(SPI_SELECT_SLAVE5_PORT, SPI_SELECT_SLAVE5_PIN);
      break;
#endif //USE_SPI_SLAVE5
    default:
      break;
  }
}

void spi_slave_select(uint8_t slave) {
  SpiSlaveSelect(slave);
}

void spi_slave_unselect(uint8_t slave) {
  SpiSlaveUnselect(slave);
}

void spi_init_slaves(void) {

#if USE_SPI_SLAVE0
  gpio_setup_output(SPI_SELECT_SLAVE0_PORT, SPI_SELECT_SLAVE0_PIN);
  SpiSlaveUnselect(0);
#endif

#if USE_SPI_SLAVE1
  gpio_setup_output(SPI_SELECT_SLAVE1_PORT, SPI_SELECT_SLAVE1_PIN);
  SpiSlaveUnselect(1);
#endif

#if USE_SPI_SLAVE2
  gpio_setup_output(SPI_SELECT_SLAVE2_PORT, SPI_SELECT_SLAVE2_PIN);
  SpiSlaveUnselect(2);
#endif

#if USE_SPI_SLAVE3
  gpio_setup_output(SPI_SELECT_SLAVE3_PORT, SPI_SELECT_SLAVE3_PIN);
  SpiSlaveUnselect(3);
#endif

#if USE_SPI_SLAVE4
  gpio_setup_output(SPI_SELECT_SLAVE4_PORT, SPI_SELECT_SLAVE4_PIN);
  SpiSlaveUnselect(4);
#endif

#if USE_SPI_SLAVE5
  gpio_setup_output(SPI_SELECT_SLAVE5_PORT, SPI_SELECT_SLAVE5_PIN);
  SpiSlaveUnselect(5);
#endif
}


/******************************************************************************
 *
 * Implementation of the generic SPI functions
 *
 *****************************************************************************/
bool_t spi_submit(struct spi_periph* p, struct spi_transaction* t)
{
  uint8_t idx;
  idx = p->trans_insert_idx + 1;
  if (idx >= SPI_TRANSACTION_QUEUE_LEN) idx = 0;
  if ((idx == p->trans_extract_idx) || ((t->input_length == 0) && (t->output_length == 0))) {
    t->status = SPITransFailed;
    return FALSE; /* queue full or input_length and output_length both 0 */
    // TODO can't tell why it failed here if it does
  }

  t->status = SPITransPending;

  //Disable interrupts to avoid race conflict with end of DMA transfer interrupt
  //FIXME
  spi_arch_int_disable(p);

  // GT: no copy?  There's a queue implying a copy here...
  p->trans[p->trans_insert_idx] = t;
  p->trans_insert_idx = idx;

  /* if peripheral is idle, start the transaction */
  if (p->status == SPIIdle && !p->suspend) {
    spi_start_dma_transaction(p, p->trans[p->trans_extract_idx]);
  }
  //FIXME
  spi_arch_int_enable(p);
  return TRUE;
}

bool_t spi_lock(struct spi_periph* p, uint8_t slave) {
  spi_arch_int_disable(p);
  if (slave < 254 && p->suspend == 0) {
    p->suspend = slave + 1; // 0 is reserved for unlock state
    spi_arch_int_enable(p);
    return TRUE;
  }
  spi_arch_int_enable(p);
  return FALSE;
}

bool_t spi_resume(struct spi_periph* p, uint8_t slave) {
  spi_arch_int_disable( p );
  if (p->suspend == slave + 1) {
    // restart fifo
    p->suspend = 0;
    if (p->trans_extract_idx != p->trans_insert_idx && p->status == SPIIdle) {
      spi_start_dma_transaction(p, p->trans[p->trans_extract_idx]);
    }
    spi_arch_int_enable(p);
    return TRUE;
  }
  spi_arch_int_enable(p);
  return FALSE;
}


/******************************************************************************
 *
 * Transaction configuration helper functions
 *
 *****************************************************************************/
static void set_default_comm_config(struct locm3_spi_comm* c) {
  c->br = SPI_CR1_BAUDRATE_FPCLK_DIV_64;
  c->cpol = SPI_CR1_CPOL_CLK_TO_1_WHEN_IDLE;
  c->cpha = SPI_CR1_CPHA_CLK_TRANSITION_2;
  c->dff = SPI_CR1_DFF_8BIT;
  c->lsbfirst = SPI_CR1_MSBFIRST;
}

static inline uint8_t get_transaction_signature(struct spi_transaction* t) {
  return ((t->dss << 6) | (t->cdiv << 3) | (t->bitorder << 2) |
          (t->cpha << 1) | (t->cpol));
}

static uint8_t get_comm_signature(struct locm3_spi_comm* c) {
  uint8_t sig = 0;
  if (c->cpol == SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE) {
    sig |= SPICpolIdleLow;
  } else {
    sig |= SPICpolIdleHigh;
  }
  if (c->cpha == SPI_CR1_CPHA_CLK_TRANSITION_1) {
    sig |= (SPICphaEdge1 << 1);
  } else {
    sig |= (SPICphaEdge2 << 1);
  }
  if (c->lsbfirst == SPI_CR1_MSBFIRST) {
    sig |= (SPIMSBFirst << 2);
  } else {
    sig |= (SPILSBFirst << 2);
  }
  uint8_t cdiv = SPIDiv256;
  switch (c->br) {
    case SPI_CR1_BAUDRATE_FPCLK_DIV_2:
      cdiv = SPIDiv2;
      break;
    case SPI_CR1_BAUDRATE_FPCLK_DIV_4:
      cdiv = SPIDiv4;
      break;
    case SPI_CR1_BAUDRATE_FPCLK_DIV_8:
      cdiv = SPIDiv8;
      break;
    case SPI_CR1_BAUDRATE_FPCLK_DIV_16:
      cdiv = SPIDiv16;
      break;
    case SPI_CR1_BAUDRATE_FPCLK_DIV_32:
      cdiv = SPIDiv32;
      break;
    case SPI_CR1_BAUDRATE_FPCLK_DIV_64:
      cdiv = SPIDiv64;
      break;
    case SPI_CR1_BAUDRATE_FPCLK_DIV_128:
      cdiv = SPIDiv128;
      break;
    case SPI_CR1_BAUDRATE_FPCLK_DIV_256:
      cdiv = SPIDiv256;
      break;
    default:
      break;
  }
  sig |= (cdiv << 3);
  if (c->dff == SPI_CR1_DFF_8BIT) {
    sig |= (SPIDss8bit << 6);
  } else {
    sig |= (SPIDss16bit << 6);
  }
  return sig;
}

/** Update SPI communication conf from generic paparazzi SPI transaction */
static void set_comm_from_transaction(struct locm3_spi_comm* c, struct spi_transaction* t) {
  if (t->dss == SPIDss8bit) {
    c->dff = SPI_CR1_DFF_8BIT;
  } else {
    c->dff = SPI_CR1_DFF_16BIT;
  }
  if (t->bitorder == SPIMSBFirst) {
    c->lsbfirst = SPI_CR1_MSBFIRST;
  } else {
    c->lsbfirst = SPI_CR1_LSBFIRST;
  }
  if (t->cpha == SPICphaEdge1) {
    c->cpha = SPI_CR1_CPHA_CLK_TRANSITION_1;
  } else {
    c->cpha = SPI_CR1_CPHA_CLK_TRANSITION_2;
  }
  if (t->cpol == SPICpolIdleLow) {
    c->cpol = SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE;
  } else {
    c->cpol = SPI_CR1_CPOL_CLK_TO_1_WHEN_IDLE;
  }

  switch (t->cdiv) {
    case SPIDiv2:
      c->br = SPI_CR1_BAUDRATE_FPCLK_DIV_2;
      break;
    case SPIDiv4:
      c->br = SPI_CR1_BAUDRATE_FPCLK_DIV_4;
      break;
    case SPIDiv8:
      c->br = SPI_CR1_BAUDRATE_FPCLK_DIV_8;
      break;
    case SPIDiv16:
      c->br = SPI_CR1_BAUDRATE_FPCLK_DIV_16;
      break;
    case SPIDiv32:
      c->br = SPI_CR1_BAUDRATE_FPCLK_DIV_32;
      break;
    case SPIDiv64:
      c->br = SPI_CR1_BAUDRATE_FPCLK_DIV_64;
      break;
    case SPIDiv128:
      c->br = SPI_CR1_BAUDRATE_FPCLK_DIV_128;
      break;
    case SPIDiv256:
      c->br = SPI_CR1_BAUDRATE_FPCLK_DIV_256;
      break;
    default:
      break;
  }
}


/******************************************************************************
 *
 * Helpers for SPI transactions with DMA
 *
 *****************************************************************************/
static void spi_configure_dma(uint32_t dma, uint8_t chan, uint32_t periph_addr, uint32_t buf_addr,
                              uint16_t len, enum SPIDataSizeSelect dss, bool_t increment)
{
  dma_channel_reset(dma, chan);
  dma_set_peripheral_address(dma, chan, periph_addr);
  dma_set_memory_address(dma, chan, buf_addr);
  dma_set_number_of_data(dma, chan, len);

  /* Set the dma transfer size based on SPI transaction DSS */
  if (dss == SPIDss8bit) {
    dma_set_peripheral_size(dma, chan, DMA_CCR_PSIZE_8BIT);
    dma_set_memory_size(dma, chan, DMA_CCR_MSIZE_8BIT);
  } else {
    dma_set_peripheral_size(dma, chan, DMA_CCR_PSIZE_16BIT);
    dma_set_memory_size(dma, chan, DMA_CCR_MSIZE_16BIT);
  }

  if (increment)
    dma_enable_memory_increment_mode(dma, chan);
  else
    dma_disable_memory_increment_mode(dma, chan);
}

/// Enable DMA channel interrupts
static void spi_arch_int_enable(struct spi_periph *spi) {
  /// @todo fix priority levels if necessary
  // enable receive interrupt
  nvic_set_priority( ((struct spi_periph_dma *)spi->init_struct)->rx_nvic_irq, 0);
  nvic_enable_irq( ((struct spi_periph_dma *)spi->init_struct)->rx_nvic_irq );
  // enable transmit interrupt
  nvic_set_priority( ((struct spi_periph_dma *)spi->init_struct)->tx_nvic_irq, 0);
  nvic_enable_irq( ((struct spi_periph_dma *)spi->init_struct)->tx_nvic_irq );
}

/// Disable DMA channel interrupts
static void spi_arch_int_disable(struct spi_periph *spi) {
  nvic_disable_irq( ((struct spi_periph_dma *)spi->init_struct)->rx_nvic_irq );
  nvic_disable_irq( ((struct spi_periph_dma *)spi->init_struct)->tx_nvic_irq );
}

/// start next transaction if there is one in the queue
static void spi_next_transaction(struct spi_periph* periph) {
  /* Increment the transaction to handle */
  periph->trans_extract_idx++;

  /* wrap read index of circular buffer */
  if (periph->trans_extract_idx >= SPI_TRANSACTION_QUEUE_LEN)
    periph->trans_extract_idx = 0;

  /* Check if there is another pending SPI transaction */
  if ((periph->trans_extract_idx == periph->trans_insert_idx) || periph->suspend)
    periph->status = SPIIdle;
  else
    spi_start_dma_transaction(periph, periph->trans[periph->trans_extract_idx]);
}


/**
 * Start a new transaction with DMA.
 */
static void spi_start_dma_transaction(struct spi_periph* periph, struct spi_transaction* trans)
{
  struct spi_periph_dma *dma;
  uint8_t sig = 0x00;

  /* Store local copy to notify of the results */
  trans->status = SPITransRunning;
  periph->status = SPIRunning;

  dma = periph->init_struct;

  /*
   * Check if we need to reconfigure the spi peripheral for this transaction
   */
  sig = get_transaction_signature(trans);
  if (sig != dma->comm_sig) {
    /* A different config is required in this transaction... */
    set_comm_from_transaction(&(dma->comm), trans);

    /* remember the new conf signature */
    dma->comm_sig = sig;

    /* apply the new configuration */
    spi_disable((uint32_t)periph->reg_addr);
    spi_init_master((uint32_t)periph->reg_addr, dma->comm.br, dma->comm.cpol,
                    dma->comm.cpha, dma->comm.dff, dma->comm.lsbfirst);
    spi_enable_software_slave_management((uint32_t)periph->reg_addr);
    spi_set_nss_high((uint32_t)periph->reg_addr);
    spi_enable((uint32_t)periph->reg_addr);
  }

  /*
   * Select the slave after reconfiguration of the peripheral
   */
  if (trans->select == SPISelectUnselect || trans->select == SPISelect) {
    SpiSlaveSelect(trans->slave_idx);
  }

  /* Run the callback AFTER selecting the slave */
  if (trans->before_cb != 0) {
    trans->before_cb(trans);
  }

  /*
   * Receive DMA channel configuration ----------------------------------------
   *
   * We always run the receive DMA until the very end!
   * This is done so we can use the transfer complete interrupt
   * of the RX DMA to signal the end of the transaction.
   *
   * If we want to receive less than we transmit, a dummy buffer
   * for the rx DMA is used after for the remaining data.
   *
   * In the transmit only case (input_length == 0),
   * the dummy is used right from the start.
   */
  if (trans->input_length == 0) {
    /* run the dummy rx dma for the complete transaction length */
    spi_configure_dma(dma->dma, dma->rx_chan, (uint32_t)dma->spidr,
                      (uint32_t)&(dma->rx_dummy_buf), trans->output_length, trans->dss, FALSE);
  } else {
    /* run the real rx dma for input_length */
    spi_configure_dma(dma->dma, dma->rx_chan, (uint32_t)dma->spidr,
                      (uint32_t)trans->input_buf, trans->input_length, trans->dss, TRUE);
    /* use dummy rx dma for the rest */
    if (trans->output_length > trans->input_length) {
      /* Enable use of second dma transfer with dummy buffer (cleared in ISR) */
      dma->rx_extra_dummy_dma = TRUE;
    }
  }
  dma_set_read_from_peripheral(dma->dma, dma->rx_chan);
  dma_set_priority(dma->dma, dma->rx_chan, DMA_CCR_PL_VERY_HIGH);


  /*
   * Transmit DMA channel configuration ---------------------------------------
   *
   * We always run the transmit DMA!
   * To receive data, the clock must run, so something has to be transmitted.
   * If needed, use a dummy DMA transmitting zeros for the remaining length.
   *
   * In the reveive only case (output_length == 0),
   * the dummy is used right from the start.
   */
  if (trans->output_length == 0) {
    spi_configure_dma(dma->dma, dma->tx_chan, (uint32_t)dma->spidr,
                      (uint32_t)&(dma->tx_dummy_buf), trans->input_length, trans->dss, FALSE);
  } else {
    spi_configure_dma(dma->dma, dma->tx_chan, (uint32_t)dma->spidr,
                      (uint32_t)trans->output_buf, trans->output_length, trans->dss, TRUE);
    if (trans->input_length > trans->output_length) {
      /* Enable use of second dma transfer with dummy buffer (cleared in ISR) */
      dma->tx_extra_dummy_dma = TRUE;
    }
  }
  dma_set_read_from_memory(dma->dma, dma->tx_chan);
  dma_set_priority(dma->dma, dma->tx_chan, DMA_CCR_PL_MEDIUM);


  /* Enable DMA transfer complete interrupts. */
  dma_enable_transfer_complete_interrupt(dma->dma, dma->rx_chan);
  dma_enable_transfer_complete_interrupt(dma->dma, dma->tx_chan);

  /* Enable DMA channels */
  dma_enable_channel(dma->dma, dma->rx_chan);
  dma_enable_channel(dma->dma, dma->tx_chan);

  /* Enable SPI transfers via DMA */
  spi_enable_rx_dma((uint32_t)periph->reg_addr);
  spi_enable_tx_dma((uint32_t)periph->reg_addr);
}



/******************************************************************************
 *
 * Initialization of each SPI peripheral
 *
 *****************************************************************************/
#if USE_SPI1
void spi1_arch_init(void) {

  // set dma options
  spi1_dma.spidr = (uint32_t)&SPI1_DR;
  spi1_dma.dma = DMA1;
  spi1_dma.rx_chan = DMA_CHANNEL2;
  spi1_dma.tx_chan = DMA_CHANNEL3;
  spi1_dma.rx_nvic_irq = NVIC_DMA1_CHANNEL2_IRQ;
  spi1_dma.tx_nvic_irq = NVIC_DMA1_CHANNEL3_IRQ;
  spi1_dma.tx_dummy_buf = 0;
  spi1_dma.tx_extra_dummy_dma = FALSE;
  spi1_dma.rx_dummy_buf = 0;
  spi1_dma.rx_extra_dummy_dma = FALSE;

  // set the default configuration
  set_default_comm_config(&spi1_dma.comm);
  spi1_dma.comm_sig = get_comm_signature(&spi1_dma.comm);

  // set init struct, indices and status
  spi1.reg_addr = (void *)SPI1;
  spi1.init_struct = &spi1_dma;
  spi1.trans_insert_idx = 0;
  spi1.trans_extract_idx = 0;
  spi1.status = SPIIdle;


  // Enable SPI1 Periph and gpio clocks
  rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_SPI1EN);

  // Configure GPIOs: SCK, MISO and MOSI
  gpio_set_mode(GPIO_BANK_SPI1_SCK, GPIO_MODE_OUTPUT_50_MHZ,
                GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
                GPIO_SPI1_SCK | GPIO_SPI1_MOSI);

  gpio_set_mode(GPIO_BANK_SPI1_MISO, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT,
                GPIO_SPI1_MISO);

  // reset SPI
  spi_reset(SPI1);

  // Disable SPI peripheral
  spi_disable(SPI1);

  // Force SPI mode over I2S.
  SPI1_I2SCFGR = 0;

  // configure master SPI.
  spi_init_master(SPI1, spi1_dma.comm.br, spi1_dma.comm.cpol, spi1_dma.comm.cpha,
                  spi1_dma.comm.dff, spi1_dma.comm.lsbfirst);
  /*
   * Set NSS management to software.
   *
   * Note:
   * Setting nss high is very important, even if we are controlling the GPIO
   * ourselves this bit needs to be at least set to 1, otherwise the spi
   * peripheral will not send any data out.
   */
  spi_enable_software_slave_management(SPI1);
  spi_set_nss_high(SPI1);

  // Enable SPI_1 DMA clock
  rcc_peripheral_enable_clock(&RCC_AHBENR, RCC_AHBENR_DMA1EN);

  // Enable SPI1 periph.
  spi_enable(SPI1);

  spi_arch_int_enable(&spi1);
}
#endif

#if USE_SPI2
void spi2_arch_init(void) {

  // set dma options
  spi2_dma.spidr = (uint32_t)&SPI2_DR;
  spi2_dma.dma = DMA1;
  spi2_dma.rx_chan = DMA_CHANNEL4;
  spi2_dma.tx_chan = DMA_CHANNEL5;
  spi2_dma.rx_nvic_irq = NVIC_DMA1_CHANNEL4_IRQ;
  spi2_dma.tx_nvic_irq = NVIC_DMA1_CHANNEL5_IRQ;
  spi2_dma.tx_dummy_buf = 0;
  spi2_dma.tx_extra_dummy_dma = FALSE;
  spi2_dma.rx_dummy_buf = 0;
  spi2_dma.rx_extra_dummy_dma = FALSE;

  // set the default configuration
  set_default_comm_config(&spi2_dma.comm);
  spi2_dma.comm_sig = get_comm_signature(&spi2_dma.comm);

  // set init struct, indices and status
  spi2.reg_addr = (void *)SPI2;
  spi2.init_struct = &spi2_dma;
  spi2.trans_insert_idx = 0;
  spi2.trans_extract_idx = 0;
  spi2.status = SPIIdle;


  // Enable SPI2 Periph and gpio clocks
  rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_SPI2EN);

  // Configure GPIOs: SCK, MISO and MOSI
  gpio_set_mode(GPIO_BANK_SPI2_SCK, GPIO_MODE_OUTPUT_50_MHZ,
                GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
                GPIO_SPI2_SCK | GPIO_SPI2_MOSI);

  gpio_set_mode(GPIO_BANK_SPI2_MISO, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT,
                GPIO_SPI2_MISO);

  // reset SPI
  spi_reset(SPI2);

  // Disable SPI peripheral
  spi_disable(SPI2);

  // Force SPI mode over I2S.
  SPI2_I2SCFGR = 0;

  // configure master SPI.
  spi_init_master(SPI2, spi2_dma.comm.br, spi2_dma.comm.cpol, spi2_dma.comm.cpha,
                  spi2_dma.comm.dff, spi2_dma.comm.lsbfirst);

  /*
   * Set NSS management to software.
   * Setting nss high is very important, even if we are controlling the GPIO
   * ourselves this bit needs to be at least set to 1, otherwise the spi
   * peripheral will not send any data out.
   */
  spi_enable_software_slave_management(SPI2);
  spi_set_nss_high(SPI2);

  // Enable SPI_2 DMA clock
  rcc_peripheral_enable_clock(&RCC_AHBENR, RCC_AHBENR_DMA1EN);

  // Enable SPI2 periph.
  spi_enable(SPI2);

  spi_arch_int_enable(&spi2);
}
#endif

#if USE_SPI3
void spi3_arch_init(void) {

  // set the default configuration
  spi3_dma.spidr = (uint32_t)&SPI3_DR;
  spi3_dma.dma = DMA2;
  spi3_dma.rx_chan = DMA_CHANNEL1;
  spi3_dma.tx_chan = DMA_CHANNEL2;
  spi3_dma.rx_nvic_irq = NVIC_DMA2_CHANNEL1_IRQ;
  spi3_dma.tx_nvic_irq = NVIC_DMA2_CHANNEL2_IRQ;
  spi3_dma.tx_dummy_buf = 0;
  spi3_dma.tx_extra_dummy_dma = FALSE;
  spi3_dma.rx_dummy_buf = 0;
  spi3_dma.rx_extra_dummy_dma = FALSE;

  // set the default configuration
  set_default_comm_config(&spi3_dma.comm);
  spi3_dma.comm_sig = get_comm_signature(&spi3_dma.comm);

  // set init struct, indices and status
  spi3.reg_addr = (void *)SPI3;
  spi3.init_struct = &spi3_dma;
  spi3.trans_insert_idx = 0;
  spi3.trans_extract_idx = 0;
  spi3.status = SPIIdle;


  // Enable SPI3 Periph and gpio clocks
  rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_SPI3EN);

  // Configure GPIOs: SCK, MISO and MOSI
  gpio_set_mode(GPIO_BANK_SPI3_SCK, GPIO_MODE_OUTPUT_50_MHZ,
                GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
                GPIO_SPI3_SCK | GPIO_SPI3_MOSI);

  gpio_set_mode(GPIO_BANK_SPI3_MISO, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT,
                GPIO_SPI3_MISO);

  /// @todo disable JTAG so the pins can be used?

  // reset SPI
  spi_reset(SPI3);

  // Disable SPI peripheral
  spi_disable(SPI3);

  // Force SPI mode over I2S.
  SPI3_I2SCFGR = 0;

  // configure master SPI.
  spi_init_master(SPI3, spi3_dma.comm.br, spi3_dma.comm.cpol, spi3_dma.comm.cpha,
                  spi3_dma.comm.dff, spi3_dma.comm.lsbfirst);

  /*
   * Set NSS management to software.
   * Setting nss high is very important, even if we are controlling the GPIO
   * ourselves this bit needs to be at least set to 1, otherwise the spi
   * peripheral will not send any data out.
   */
  spi_enable_software_slave_management(SPI3);
  spi_set_nss_high(SPI3);

  // Enable SPI_3 DMA clock
  rcc_peripheral_enable_clock(&RCC_AHBENR, RCC_AHBENR_DMA2EN);

  // Enable SPI3 periph.
  spi_enable(SPI3);

  spi_arch_int_enable(&spi3);
}
#endif




/******************************************************************************
 *
 * DMA Interrupt service routines
 *
 *****************************************************************************/
#ifdef USE_SPI1
/// receive transferred over DMA
void dma1_channel2_isr(void)
{
  if ((DMA1_ISR & DMA_ISR_TCIF2) != 0) {
    // clear int pending bit
    DMA1_IFCR |= DMA_IFCR_CTCIF2;
  }
  process_rx_dma_interrupt(&spi1);
}

/// transmit transferred over DMA
void dma1_channel3_isr(void)
{
  if ((DMA1_ISR & DMA_ISR_TCIF3) != 0) {
    // clear int pending bit
    DMA1_IFCR |= DMA_IFCR_CTCIF3;
  }
  process_tx_dma_interrupt(&spi1);
}

#endif

#ifdef USE_SPI2
/// receive transferred over DMA
void dma1_channel4_isr(void)
{
  if ((DMA1_ISR & DMA_ISR_TCIF4) != 0) {
    // clear int pending bit
    DMA1_IFCR |= DMA_IFCR_CTCIF4;
  }
  process_rx_dma_interrupt(&spi2);
}

/// transmit transferred over DMA
void dma1_channel5_isr(void)
{
  if ((DMA1_ISR & DMA_ISR_TCIF5) != 0) {
    // clear int pending bit
    DMA1_IFCR |= DMA_IFCR_CTCIF5;
  }
  process_tx_dma_interrupt(&spi2);
}

#endif

#if USE_SPI3
/// receive transferred over DMA
void dma2_channel1_isr(void)
{
  if ((DMA2_ISR & DMA_ISR_TCIF1) != 0) {
    // clear int pending bit
    DMA2_IFCR |= DMA_IFCR_CTCIF1;
  }
  process_rx_dma_interrupt(&spi3);
}

/// transmit transferred over DMA
void dma2_channel2_isr(void)
{
  if ((DMA2_ISR & DMA_ISR_TCIF2) != 0) {
    // clear int pending bit
    DMA2_IFCR |= DMA_IFCR_CTCIF2;
  }
  process_tx_dma_interrupt(&spi3);
}

#endif

/// Processing done after rx completes.
void process_rx_dma_interrupt(struct spi_periph *periph) {
  struct spi_periph_dma *dma = periph->init_struct;
  struct spi_transaction *trans = periph->trans[periph->trans_extract_idx];

  /* Disable DMA Channel */
  dma_disable_transfer_complete_interrupt(dma->dma, dma->rx_chan);

  /* Disable SPI Rx request */
  spi_disable_rx_dma((uint32_t)periph->reg_addr);

  /* Disable DMA rx channel */
  dma_disable_channel(dma->dma, dma->rx_chan);


  if (dma->rx_extra_dummy_dma) {
    /*
     * We are finished the first part of the receive with real data,
     * but still need to run the dummy to get a transfer complete interrupt
     * after the complete transaction is done.
     */

    /* Reset the flag so this only happens once in a transaction */
    dma->rx_extra_dummy_dma = FALSE;

    /* Use the difference in length between rx and tx */
    uint16_t len_remaining = trans->output_length - trans->input_length;

    spi_configure_dma(dma->dma, dma->rx_chan, (uint32_t)dma->spidr,
                      (uint32_t)&(dma->rx_dummy_buf), len_remaining, trans->dss, FALSE);
    dma_set_read_from_peripheral(dma->dma, dma->rx_chan);
    dma_set_priority(dma->dma, dma->rx_chan, DMA_CCR_PL_HIGH);

    /* Enable DMA transfer complete interrupts. */
    dma_enable_transfer_complete_interrupt(dma->dma, dma->rx_chan);
    /* Enable DMA channels */
    dma_enable_channel(dma->dma, dma->rx_chan);
    /* Enable SPI transfers via DMA */
    spi_enable_rx_dma((uint32_t)periph->reg_addr);
  }
  else {
    /*
     * Since the receive DMA is always run until the very end
     * and this interrupt is triggered after the last data word was read,
     * we now know that this transaction is finished.
     */

    /* Run the callback */
    trans->status = SPITransSuccess;
    if (trans->after_cb != 0) {
      trans->after_cb(trans);
    }

    /* AFTER the callback, then unselect the slave if required */
    if (trans->select == SPISelectUnselect || trans->select == SPIUnselect) {
      SpiSlaveUnselect(trans->slave_idx);
    }

    spi_next_transaction(periph);
  }
}

/// Processing done after tx completes
void process_tx_dma_interrupt(struct spi_periph *periph) {
  struct spi_periph_dma *dma = periph->init_struct;
  struct spi_transaction *trans = periph->trans[periph->trans_extract_idx];

  /* Disable DMA Channel */
  dma_disable_transfer_complete_interrupt(dma->dma, dma->tx_chan);

  /* Disable SPI TX request */
  spi_disable_tx_dma((uint32_t)periph->reg_addr);

  /* Disable DMA tx channel */
  dma_disable_channel(dma->dma, dma->tx_chan);

  if (dma->tx_extra_dummy_dma) {
    /*
     * We are finished the first part of the transmit with real data,
     * but still need to clock in the rest of the receive data.
     * Set up a dummy dma transmit transfer to accomplish this.
     */

    /* Reset the flag so this only happens once in a transaction */
    dma->tx_extra_dummy_dma = FALSE;

    /* Use the difference in length between tx and rx */
    uint16_t len_remaining = trans->input_length - trans->output_length;

    spi_configure_dma(dma->dma, dma->tx_chan, (uint32_t)dma->spidr,
                      (uint32_t)&(dma->tx_dummy_buf), len_remaining, trans->dss, FALSE);
    dma_set_read_from_memory(dma->dma, dma->tx_chan);
    dma_set_priority(dma->dma, dma->tx_chan, DMA_CCR_PL_MEDIUM);

    /* Enable DMA transfer complete interrupts. */
    dma_enable_transfer_complete_interrupt(dma->dma, dma->tx_chan);
    /* Enable DMA channels */
    dma_enable_channel(dma->dma, dma->tx_chan);
    /* Enable SPI transfers via DMA */
    spi_enable_tx_dma((uint32_t)periph->reg_addr);

  }
}

#endif /** SPI_MASTER */


/*
 *
 * SPI Slave code
 *
 * FIXME implement it
 *
 */
#ifdef SPI_SLAVE

#warning SPI_SLAVE mode currently not implemented for STM32.

#endif /* SPI_SLAVE */
