/*
 *	yawmd, wireless medium simulator for the Linux module mac80211_hwsim
 *	Copyright (c) 2011 cozybit Inc.
 *	Copyright (c) 2021 Miguel Moreira
 *
 *	Author:	Javier Lopez	<jlopex@cozybit.com>
 *		Javier Cardona	<javier@cozybit.com>
 *		Miguel Moreira	<mmoreira@tutanota.com>
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

#ifndef YAWMD_H_
#define YAWMD_H_

#define YAWMD_VERSION_MAJOR 2
#define YAWMD_VERSION_MINOR 0

/* Version of netlink communication protocol with mac80211_hwsim. */
#define YAWMD_HWSIM_PROTO_VERSION 2

#define YAWMD_DEFAULT_LOG_LEVEL	6

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <time.h>
#include <event2/event.h>
#include <event2/event_struct.h>
#include <pthread.h>
#include "list.h"
#include "ieee80211.h"

#define HWSIM_TX_CTL_REQ_TX_STATUS	1
#define HWSIM_TX_CTL_NO_ACK		(1 << 1)
#define HWSIM_TX_STAT_ACK		(1 << 2)

/* Netlink message identifier */
enum {
	HWSIM_CMD_UNSPEC,
	HWSIM_CMD_REGISTER,
	HWSIM_CMD_FRAME,
	HWSIM_CMD_TX_INFO_FRAME,
	HWSIM_CMD_NEW_RADIO,
	HWSIM_CMD_DEL_RADIO,
	HWSIM_CMD_GET_RADIO,
	HWSIM_YAWMD_TX_INFO,
	HWSIM_YAWMD_RX_INFO,
	__HWSIM_CMD_MAX,
};


/**
 * enum hwsim_attrs - hwsim netlink attributes
 *
 * @HWSIM_ATTR_UNSPEC: unspecified attribute to catch errors
 *
 * @HWSIM_ATTR_ADDR_RECEIVER: MAC address of the radio device that
 *	the frame is broadcasted to
 * @HWSIM_ATTR_ADDR_TRANSMITTER: MAC address of the radio device that
 *	the frame was broadcasted from
 * @HWSIM_ATTR_FRAME: Data array
 * @HWSIM_ATTR_FLAGS: mac80211 transmission flags, used to process
	properly the frame at user space
 * @HWSIM_ATTR_RX_RATE: estimated rx rate index for this frame at user
	space
 * @HWSIM_ATTR_SIGNAL: estimated RX signal for this frame at user
	space
 * @HWSIM_ATTR_TX_INFO: ieee80211_tx_rate array
 * @HWSIM_ATTR_COOKIE: sk_buff cookie to identify the frame
 * @HWSIM_ATTR_CHANNELS: u32 attribute used with the %HWSIM_CMD_CREATE_RADIO
 *	command giving the number of channels supported by the new radio
 * @HWSIM_ATTR_RADIO_ID: u32 attribute used with %HWSIM_CMD_DESTROY_RADIO
 *	only to destroy a radio
 * @HWSIM_ATTR_REG_HINT_ALPHA2: alpha2 for regulatoro driver hint
 *	(nla string, length 2)
 * @HWSIM_ATTR_REG_CUSTOM_REG: custom regulatory domain index (u32 attribute)
 * @HWSIM_ATTR_REG_STRICT_REG: request REGULATORY_STRICT_REG (flag attribute)
 * @HWSIM_ATTR_SUPPORT_P2P_DEVICE: support P2P Device virtual interface (flag)
 * @HWSIM_ATTR_USE_CHANCTX: used with the %HWSIM_CMD_CREATE_RADIO
 *	command to force use of channel contexts even when only a
 *	single channel is supported
 * @HWSIM_ATTR_DESTROY_RADIO_ON_CLOSE: used with the %HWSIM_CMD_CREATE_RADIO
 *	command to force radio removal when process that created the radio dies
 * @HWSIM_ATTR_RADIO_NAME: Name of radio, e.g. phy666
 * @HWSIM_ATTR_NO_VIF:  Do not create vif (wlanX) when creating radio.
 * @HWSIM_ATTR_FREQ: Frequency at which packet is transmitted or received.
 * @HWSIM_ATTR_TX_INFO_FLAGS: additional flags for corresponding
 *	rates of %HWSIM_ATTR_TX_INFO
 * @HWSIM_ATTR_PERM_ADDR: permanent mac address of new radio
 * @HWSIM_ATTR_IFTYPE_SUPPORT: u32 attribute of supported interface types bits
 * @HWSIM_ATTR_CIPHER_SUPPORT: u32 array of supported cipher types
 * @HWSIM_ATTR_FRAME_HEADER: frame header, used by yawmd
 * @HWSIM_ATTR_FRAME_LENGTH: frame length in bytes, used by yawmd
 * @HWSIM_ATTR_FRAME_ID: u64 unique identifier of a frame, used with yawmd
 * @HWSIM_ATTR_RECEIVER_INFO: array of struct itf_recv_info/hwsim_itf_recv_info
 * @__HWSIM_ATTR_MAX: enum limit
 */
enum {
	HWSIM_ATTR_UNSPEC,
	HWSIM_ATTR_ADDR_RECEIVER,
	HWSIM_ATTR_ADDR_TRANSMITTER,
	HWSIM_ATTR_FRAME,
	HWSIM_ATTR_FLAGS,
	HWSIM_ATTR_RX_RATE,
	HWSIM_ATTR_SIGNAL,
	HWSIM_ATTR_TX_INFO,
	HWSIM_ATTR_COOKIE,
	HWSIM_ATTR_CHANNELS,
	HWSIM_ATTR_RADIO_ID,
	HWSIM_ATTR_REG_HINT_ALPHA2,
	HWSIM_ATTR_REG_CUSTOM_REG,
	HWSIM_ATTR_REG_STRICT_REG,
	HWSIM_ATTR_SUPPORT_P2P_DEVICE,
	HWSIM_ATTR_USE_CHANCTX,
	HWSIM_ATTR_DESTROY_RADIO_ON_CLOSE,
	HWSIM_ATTR_RADIO_NAME,
	HWSIM_ATTR_NO_VIF,
	HWSIM_ATTR_FREQ,
	HWSIM_ATTR_PAD,
	HWSIM_ATTR_TX_INFO_FLAGS,
	HWSIM_ATTR_PERM_ADDR,
	HWSIM_ATTR_IFTYPE_SUPPORT,
	HWSIM_ATTR_CIPHER_SUPPORT,
	HWSIM_ATTR_FRAME_HEADER,
	HWSIM_ATTR_FRAME_LENGTH,
	HWSIM_ATTR_FRAME_ID,
	HWSIM_ATTR_RECEIVER_INFO,
	__HWSIM_ATTR_MAX,
};
#define HWSIM_ATTR_MAX (__HWSIM_ATTR_MAX - 1)

#define TIME_FMT "%lld.%06lld"
#define TIME_ARGS(a) ((unsigned long long)(a)->tv_sec), ((unsigned long long)(a)->tv_nsec/1000)

#define MAC_FMT "%02x:%02x:%02x:%02x:%02x:%02x"
#define MAC_ARGS(a) a[0],a[1],a[2],a[3],a[4],a[5]

#ifndef min
#define min(x,y) ((x) < (y) ? (x) : (y))
#endif


typedef uint8_t u8;
typedef uint32_t u32;
typedef uint64_t u64;


struct wqueue {
	struct list_head frames;
	int cw_min;
	int cw_max;
};


/* General information regarding yawmd. */
struct yawmd {
	// list of struct medium
	struct list_head 	medium_list;
	unsigned int		n_mediums;
	int 			log_level;
	struct nl_sock 		*socket;
	struct nl_cb 		*cb;
	int 			family_id;
	int			timer_fd;
	struct event_base	*ev_base;
	bool			threads;
};

/* Each medium is an isolated transmission environment. */
struct medium {
	struct list_head 	list;
	struct list_head	frame_queue;
	struct yawmd		*ctx;
	pthread_mutex_t		queue_mutex;
	pthread_t		thread;
	int 			id;
	unsigned 		n_interfaces;
	struct interface 	*interfaces;
	// row transmitter x column receiver
	int 			*snr_matrix;
	double 			*prob_matrix;
	double 			move_interval;
	int 			fading_coefficient; // int??
	int 			noise_level;
	bool 			sim_interference;
	int 			model_index; // enum model_name
	int			move_timerfd;
	int			delivery_timerfd;
	int			queue_timerfd;
	struct event		move_event;
	struct event		delivery_event;
	struct event		queue_event;
	struct itimerspec 	move_time;

	// The medium stores information for medium access. It is stored the
	// frame being transmitted at the moment (.current_transmission) and
	// the timestamp at which it will end being transmitted
	// (.end_transmission), which is also the time the .delivery_timerfd is
	// set to trigger.
	struct wqueue		qos_queues[IEEE80211_NUM_ACS];
	struct frame		*current_transmission;
	struct timespec		end_transmission;

	union {
		struct {
			// free_space, log_norm_shad, two_ray
			int 		system_loss;
			// log_norm_shad, log_dist
			double		path_loss_exponent;
			// log_dist
			double 		xg;
		};
		struct { // itu
			unsigned int 	n_floors; 
			int 		floor_pen_factor; 
			int 		power_loss_coeff;
		};
	};
	int 	(*get_link_snr)		(struct medium *medium,
					 struct interface *transmitter,
			    	 	 struct interface *receiver);
	double	(*get_error_prob)	(struct medium *medium, double snr,
					 unsigned int rate_idx, u32 freq,
					 int frame_len,
					 struct interface *transmitter,
					 struct interface *receiver);
	int	(*path_loss_func)	(struct medium *medium,
					 struct interface *transmitter,
					 struct interface *receiver);
	void	(*move_interfaces)	(struct medium *medium);
};

struct interface
{
	unsigned int	index;
	unsigned char 	addr[ETH_ALEN];
	unsigned char 	hwaddr[ETH_ALEN];
	bool 		isap;
	double 		position_x, position_y, position_z;
	double 		direction_x, direction_y, direction_z;
	int 		antenna_gain;
	int 		tx_power;
	u32		frequency;
	struct medium	*medium;
};

struct hwsim_tx_rate {
	signed char idx;
	unsigned char count;
};

struct frame {
	// list node
	struct list_head	list;
	bool			acked;
	u64			cookie;
	u32			freq;
	int			flags;
	int			signal;
	int			duration;
	int			tx_rates_count;
	struct interface	*sender;
	struct hwsim_tx_rate	tx_rates[IEEE80211_TX_MAX_RATES];
	// Frame length (MAC header + IP Header + Transport Header + Payload)
	size_t			frame_len;
	// Frame header. Includes space for QoS data.
	struct ieee80211_hdr	header;
};

struct log_distance_model_param {
	double path_loss_exponent;
	double Xg;
};

struct itu_model_param {
	int nFLOORS;
	int lF;
	int pL;
};

struct log_normal_shadowing_model_param {
	int sL;
	double path_loss_exponent;
};

struct free_space_model_param {
	int sL;
};

struct two_ray_ground_model_param {
	int sL;
};

struct intf_info {
	int signal;
	int duration;
	double prob_col;
};


/* itf_recv_info - interface receive information

One of the blocks of information sent to mac80211_hwsim to indicate which
interfaces should receive a copy of the frame and with which signal
intensity (as was determined by the simulation).
Sent as an array of struct itf_recv_info. */
struct itf_recv_info {
 	u8 mac_addr[ETH_ALEN];
 	u32 signal;
} __attribute__((__packed__)) __attribute__((__aligned__(1)));


/* Keeps track of the reception information of a frame. Instead of using
directly itf_recv_info, the operations of adding the interface information
are handled using procedures. */
struct recv_container {
	struct itf_recv_info *recv_info;
	int size;
};

int w_logf(struct yawmd *ctx, u8 level, const char *format, ...);
int w_flogf(struct yawmd *ctx, u8 level, FILE *stream, const char *format, ...);
int index_to_rate(size_t index, u32 freq);

#endif /* YAWMD_H_ */
