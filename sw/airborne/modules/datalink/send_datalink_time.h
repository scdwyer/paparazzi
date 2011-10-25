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

/** \file send_datalink_time.h
 *
 *   This sends a message with the datalink_time value for debugging the datalink
 */

#ifndef SEND_DATALINK_TIME_H
#define SEND_DATALINK_TIME_H

void send_datalink_time_periodic( void );

#endif


