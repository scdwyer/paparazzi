/*
 * $Id$
 *
 * Copyright (C) 2011 Stephen Dwyer
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

/** \file send_datalink_time.c
 *
 *   This sends a message with the datalink_time value for debugging the datalink
 */


#include "datalink/send_datalink_time.h"

#ifndef DOWNLINK_DEVICE
#define DOWNLINK_DEVICE DOWNLINK_AP_DEVICE
#endif

#include "mcu_periph/uart.h"
#include "messages.h"
#include "downlink.h"


void send_downlink_time_periodic( void ) {
    uint8_t ac_id = AC_ID;
    extern uint16_t datalink_time;

    DOWNLINK_SEND_AC_DATALINK_TIME_(DefaultChannel,
                &ac_id,
                &datalink_time );
}

