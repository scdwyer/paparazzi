/*
 * Copyright (C) 2008-2009 Antoine Drouin <poinix@gmail.com>
 * Copyright (C) 2012 Gautier Hattenberger
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
 */

/**
 * @file peripherals/ms2100.c
 * Driver for the ms2100 magnetic sensor from PNI
 */

#include "peripherals/ms2100.h"
#include "mcu_periph/spi.h"

#include <stdlib.h>  // for abs


#define MS2100_DIVISOR_128  2
#define MS2100_DIVISOR_256  3
#define MS2100_DIVISOR_512  4
#define MS2100_DIVISOR_1024 5

#ifndef MS2100_DIVISOR
#define MS2100_DIVISOR MS2100_DIVISOR_1024
#endif

// keep stupid global variable for now...
struct Ms2100 ms2100;

/* callback function to lock the spi fifo
 * after the first transaction
 */
void ms2100_lock_cb(struct spi_transaction * t);
static void ms2100_req_meas(struct Ms2100 *ms, uint8_t axis);


void ms2100_init(struct Ms2100 *ms, struct spi_periph *spi_p, uint8_t slave_idx) {

  /* set spi_peripheral */
  ms->spi_p = spi_p;

  /* configure spi transaction for the request */
  ms->req_trans.cpol = SPICpolIdleLow;
  ms->req_trans.cpha = SPICphaEdge1;
  ms->req_trans.dss = SPIDss8bit;
  ms->req_trans.bitorder = SPIMSBFirst;
  ms->req_trans.cdiv = SPIDiv64;

  ms->req_trans.slave_idx = slave_idx;
  ms->req_trans.select = SPISelect;
  ms->req_trans.output_buf = ms->req_buf;
  ms->req_trans.output_length = 1;
  ms->req_trans.input_buf = NULL;
  ms->req_trans.input_length = 0;
  // ms2100 has to be reset before each measurement: implemented in ms2100_arch.c
  ms->req_trans.before_cb = ms2100_reset_cb;
  ms->req_trans.after_cb = ms2100_lock_cb;
  ms->req_trans.status = SPITransDone;

  /* configure spi transaction to read the result */
  ms->read_trans.cpol = SPICpolIdleLow;
  ms->read_trans.cpha = SPICphaEdge1;
  ms->read_trans.dss = SPIDss8bit;
  ms->read_trans.bitorder = SPIMSBFirst;
  ms->read_trans.cdiv = SPIDiv64;

  ms->read_trans.slave_idx = slave_idx;
  ms->read_trans.select = SPIUnselect;
  ms->read_trans.output_buf = NULL;
  ms->read_trans.output_length = 0;
  ms->read_trans.input_buf = ms->read_buf;
  ms->read_trans.input_length = 2;
  ms->read_trans.before_cb = NULL;
  ms->read_trans.after_cb = NULL;
  ms->read_trans.status = SPITransDone;

  ms2100_arch_init();

  INT_VECT3_ZERO(ms->data.vect);
  ms->cur_axe = 0;

  ms->status = MS2100_IDLE;
}

/// send request to read given axis
static void ms2100_req_meas(struct Ms2100 *ms, uint8_t axis) {
  ms->req_buf[0] = axis << 0 | MS2100_DIVISOR << 4;
  spi_submit(ms->spi_p, &(ms->req_trans));
  spi_submit(ms->spi_p, &(ms->read_trans));
  ms->status = MS2100_SENDING_REQ;
}

/// send request to read next axis
void ms2100_read(struct Ms2100 *ms) {
  /* schedule read on first axis */
  ms2100_req_meas(ms, 0);
}

#define Int16FromBuf(_buf,_idx) ((int16_t)((_buf[_idx]<<8) | _buf[_idx+1]))

void ms2100_event(struct Ms2100 *ms) {

  // handle request transaction
  if (ms->req_trans.status == SPITransSuccess) {
    ms->req_trans.status = SPITransDone;
  }
  else if (ms->req_trans.status == SPITransFailed) {
    ms->status = MS2100_IDLE;
    spi_slave_unselect(ms->req_trans.slave_idx);
    spi_resume(ms->spi_p, ms->req_trans.slave_idx);
    ms->cur_axe = 0;
    ms->req_trans.status = SPITransDone;
  }

  // handle reading transaction
  if (ms->read_trans.status == SPITransSuccess) {
    if (ms->status == MS2100_READING_RES) {
      // store value
      int16_t new_val = Int16FromBuf(ms->read_buf,0);
      // what is this check about?
      if (abs(new_val) < 2000) {
        ms->data.value[ms->cur_axe] = new_val;
      }
      ms->cur_axe++;
      if (ms->cur_axe > 2) {
        ms->cur_axe = 0;
        ms->status = MS2100_DATA_AVAILABLE;
      }
      else {
        ms->status = MS2100_IDLE;
      }
      ms->read_trans.status = SPITransDone;
    }
  }
  else if (ms->read_trans.status == SPITransDone) {
    /* request next axis if not all have been read in this cycle */
    if (ms->cur_axe == 1 || ms->cur_axe == 2) {
      ms2100_req_meas(ms, ms->cur_axe);
    }
  }
  else if (ms->read_trans.status == SPITransFailed) {
    ms->status = MS2100_IDLE;
    spi_slave_unselect(ms->req_trans.slave_idx);
    ms->cur_axe = 0;
    ms->read_trans.status = SPITransDone;
  }

  /* check for end of conversion */
  if (ms->status == MS2100_GOT_EOC) {
    // eoc occurs, unlock SPI
    spi_resume(ms->spi_p, ms->req_trans.slave_idx);
    ms->status = MS2100_READING_RES;
  }
}

void ms2100_lock_cb(struct spi_transaction * trans) {
  spi_lock(ms2100.spi_p, trans->slave_idx);
}
