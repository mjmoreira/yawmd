/*
 *	yawmd, wireless medium simulator for the Linux module mac80211_hwsim
 *	Copyright (c) 2021 Miguel Moreira
 *
 *	Author: Miguel Moreira  <mmoreira@tutanota.com>
 *
 *	This program is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU General Public License
 *	as published by the Free Software Foundation; either version 2
 *	of the License, or (at your option) any later version.
 *
 *	This program is distributed in the hope that it will be useful,
 *	but WITHOUT ANY WARRANTY; without even the implied warranty of
 *	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *	GNU General Public License for more details.
 *
 *	You should have received a copy of the GNU General Public License
 *	along with this program; if not, write to the Free Software
 *	Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 *	02110-1301, USA.
 */

#ifndef YAWMD_CONFIG_H_
#define YAWMD_CONFIG_H_

#include <stdbool.h>
#include "yawmd.h"

static const int 	DEFAULT_CCA_THRESHOLD = -90;
static const int 	DEFAULT_NOISE_LEVEL = -91;
static const int 	DEFAULT_SNR = 20 - DEFAULT_NOISE_LEVEL;
static const int 	CFG_DEFAULT_SNR = -100; // dBm
static const double 	CFG_DEFAULT_PROB = 1.0; // [0.0,1.0]
static const int 	CFG_DEFAULT_NOISE_LEVEL = -91; // dBm
static const int 	CFG_DEFAULT_FADING_COEFFICIENT = 0;
static const double 	CFG_DEFAULT_MOVE_INTERVAL = 5.0; // seconds
static const int 	CFG_DEFAULT_ANTENNA_GAIN = 0; // dBm
static const bool 	CFG_DEFAULT_SIMULATE_INTERFERENCE = false;
static const bool 	CFG_DEFAULT_ISNODEAPS = false;

bool configure(char *file_name, struct yawmd *ctx);
void delete_mediums(struct yawmd *mediums);

int get_fading_signal(struct medium *medium);

void dump_medium_info(struct medium* info);

#endif // YAWMD_CONFIG_H_
