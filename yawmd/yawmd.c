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

#include <netlink/netlink.h>
#include <netlink/genl/genl.h>
#include <netlink/genl/ctrl.h>
#include <netlink/genl/family.h>
#include <stdint.h>
#include <getopt.h>
#include <signal.h>
#include <math.h>
#include <sys/timerfd.h>
#include <errno.h>
#include <limits.h>
#include <unistd.h>
#include <pthread.h>
#include <stdio.h>
#include <syslog.h>

#include "yawmd.h"
#include "ieee80211.h"
#include "config.h"
// #include "yserver.h"
// #include "config_dynamic.h"
// #include "yserver_messages.h"

// See main()
struct itimerspec it_1ns;

/* Integer round up division. For example 1.1 gets rounded to 2. */
static inline int div_round(int a, int b)
{
	return (a + b - 1) / b;
}

/* Calculate frame transmission duration. */
static inline int pkt_duration(int len, int rate)
{
	/* preamble + signal + t_sym * n_sym, rate in 100 kbps */
	return 16 + 4 + 4 * div_round((16 + 8 * len + 6) * 10, 4 * rate);
}

/* Log to stdout if level <= ctx->log_lvl. */
int w_logf(struct yawmd *ctx, u8 level, const char *format, ...)
{
	va_list(args);
	va_start(args, format);
	if (ctx->log_level >= level) {
		return vprintf(format, args);
	}
	return -1;
}

/* Log to file if level <= ctx->log_lvl. */
int w_flogf(struct yawmd *ctx, u8 level, FILE *stream, const char *format, ...)
{
	va_list(args);
	va_start(args, format);
	if (ctx->log_level >= level) {
		return vfprintf(stream, format, args);
	}
	return -1;
}

/* Is t1 < t2. False if t1 >= t2. */
static bool timespec_before(struct timespec *t1, struct timespec *t2)
{
	return t1->tv_sec < t2->tv_sec ||
	       (t1->tv_sec == t2->tv_sec && t1->tv_nsec < t2->tv_nsec);
}

/* t + usec */
static void timespec_add_usec(struct timespec *t, int usec)
{
	t->tv_nsec += usec * 1000;
	if (t->tv_nsec >= 1000000000) {
		t->tv_sec++;
		t->tv_nsec -= 1000000000;
	}
}

/* t + seconds */
static void timespec_add_seconds(struct timespec *t, double seconds) {
	long long ns = (long long) t->tv_nsec + (long long) (seconds * 1E9);
	
	t->tv_sec += (long) (ns / (long long) 1E9);
	t->tv_nsec = (long) (ns % (long) 1E9);
}

/* c = a - b */
static int timespec_sub(struct timespec *a, struct timespec *b,
			struct timespec *c)
{
	c->tv_sec = a->tv_sec - b->tv_sec;

	if (a->tv_nsec < b->tv_nsec) {
		c->tv_sec--;
		c->tv_nsec = 1000000000 + a->tv_nsec - b->tv_nsec;
	} else {
		c->tv_nsec = a->tv_nsec - b->tv_nsec;
	}

	return 0;
}


//------------------------------------------------------------------------------
/* struct recv_container manipulation procedures */

/* Create or initialize a recv_container. */
struct recv_container* create_recv_container(struct recv_container *container,
                                             struct medium *medium)
{
	if (container == NULL) {
		container = calloc(1, sizeof(struct recv_container));
	}
	container->size = 0;
	container->recv_info =
		malloc(sizeof(struct recv_container) * medium->n_interfaces);
	return container;
}

/* Add new entry to the container. */
inline void add_recv_info(struct recv_container *container, u8 *mac, int signal)
{
	memcpy(container->recv_info[container->size].mac_addr, mac, ETH_ALEN);
	container->recv_info[container->size].signal = signal;
	container->size++;
}

inline struct itf_recv_info* get_recv_info(struct recv_container *container) {
	return container->recv_info;
}

inline size_t recv_info_byte_len(struct recv_container *container)
{
	return container->size * sizeof(struct itf_recv_info);
}	

/* Frees dynamically allocated structures in the container but not the
struct recv_container itself. */
inline void delete_container(struct recv_container *container) {
	if (container != NULL && container->recv_info != NULL) {
		free(container->recv_info);
	}
}


//------------------------------------------------------------------------------

static inline bool frame_has_a4(struct frame *frame)
{
	return (frame->header.frame_control[1] & (FCTL_TODS | FCTL_FROMDS)) ==
		(FCTL_TODS | FCTL_FROMDS);
}

static inline bool frame_is_mgmt(struct frame *frame)
{
	return (frame->header.frame_control[0] & FCTL_FTYPE) == FTYPE_MGMT;
}

static inline bool frame_is_data(struct frame *frame)
{
	return (frame->header.frame_control[0] & FCTL_FTYPE) == FTYPE_DATA;
}

static inline bool frame_is_data_qos(struct frame *frame)
{
	return (frame->header.frame_control[0] & (FCTL_FTYPE | STYPE_QOS_DATA))
	       == (FTYPE_DATA | STYPE_QOS_DATA);
}

static inline u8 *frame_get_qos_ctl(struct frame *frame)
{
	if (frame_has_a4(frame))
		return (u8 *)&(frame->header) + 30;
	else
		return (u8 *)&(frame->header) + 24;
}

/* Determine QoS queue the frame belongs to. */
static enum ieee80211_ac_number frame_select_queue_80211(struct frame *frame)
{
	u8 *p;
	int priority;

	if (!frame_is_data(frame))
		return IEEE80211_AC_VO;

	if (!frame_is_data_qos(frame))
		return IEEE80211_AC_BE;

	p = frame_get_qos_ctl(frame);
	priority = *p & QOS_CTL_TAG1D_MASK;

	return ieee802_1d_to_ac[priority];
}

// static double dBm_to_milliwatt(int decibel_intf)
// {
// #define INTF_LIMIT (31)
// 	int intf_diff = DEFAULT_NOISE_LEVEL - decibel_intf;

// 	if (intf_diff >= INTF_LIMIT)
// 		return 0.001;

// 	if (intf_diff <= -INTF_LIMIT)
// 		return 1000.0;

// 	return pow(10.0, -intf_diff / 10.0);
// }

// static double milliwatt_to_dBm(double value)
// {
// 	return 10.0 * log10(value);
// }

// static int set_interference_duration(struct yawmd *ctx, int src_idx,
// 				     int duration, int signal)
// {
// 	int i;

// 	if (!ctx->intf)
// 		return 0;

// 	if (signal >= CCA_THRESHOLD)
// 		return 0;

// 	for (i = 0; i < ctx->num_stas; i++) {
// 		ctx->intf[ctx->num_stas * src_idx + i].duration += duration;
// 		// use only latest value
// 		ctx->intf[ctx->num_stas * src_idx + i].signal = signal;
// 	}

// 	return 1;
// }

// static int get_signal_offset_by_interference(struct yawmd *ctx, int src_idx,
// 					     int dst_idx)
// {
// 	int i;
// 	double intf_power;

// 	if (!ctx->intf)
// 		return 0;

// 	intf_power = 0.0;
// 	for (i = 0; i < ctx->num_stas; i++) {
// 		if (i == src_idx || i == dst_idx)
// 			continue;
// 		if (drand48() < ctx->intf[i * ctx->num_stas + dst_idx].prob_col)
// 			intf_power += dBm_to_milliwatt(
// 				ctx->intf[i * ctx->num_stas + dst_idx].signal);
// 	}

// 	if (intf_power <= 1.0)
// 		return 0;

// 	return (int)(milliwatt_to_dBm(intf_power) + 0.5);
// }

static bool is_multicast_ether_addr(const u8 *addr)
{
	return 0x01 & addr[0];
}

/* Get struct interface searching by mac address in the list of interfaces of a
   medium. */
static struct interface *get_interface_medium(struct medium *medium, u8 *addr)
{
	for (unsigned int i = 0; i < medium->n_interfaces; i++) {
		if (memcmp(medium->interfaces[i].addr, addr, ETH_ALEN) == 0)
			return &medium->interfaces[i];
	}
	return NULL;
}

/* Get struct interface, searching by mac address all mediums. */
static struct interface *get_interface(struct yawmd *ctx, u8 *mac_addr)
{
	struct interface *i;
	struct medium *m;
	list_for_each_entry(m, &(ctx->medium_list), list) {
		i = get_interface_medium(m, mac_addr);
		if (i != NULL) {
			return i;
		}
	}
	return NULL;
}

/* Find appropriate QoS queue, determine delivery timestamp of the frame and
reset timer. */
static void queue_frame(struct frame *frame)
{
	u8 *dest = frame->header.addr1;
	struct timespec now;
	struct wqueue *queue;
	struct interface *sender = frame->sender;
	struct interface *receiver;
	struct medium *medium = sender->medium;
	int send_time;
	int cw;
	double error_prob;
	bool is_acked = false;
	bool noack = false;
	int i, j;
	int rate_idx;
	int ac;

	/* TODO configure phy parameters */
	int slot_time = 9;
	int sifs = 16;
	int difs = 2 * slot_time + sifs;

	int retries = 0;

	clock_gettime(CLOCK_MONOTONIC, &now);

	int ack_time_usec =
		pkt_duration(14, index_to_rate(0, frame->freq)) + sifs;

	/*
	 * To determine a frame's expiration time, we compute the
	 * number of retries we might have to make due to radio conditions
	 * or contention, and add backoff time accordingly.  To that, we
	 * add the expiration time of the previous frame in the queue.
	 */

	ac = frame_select_queue_80211(frame);
	queue = &medium->qos_queues[ac];

	/* try to "send" this frame at each of the rates in the rateset */
	send_time = 0;
	cw = queue->cw_min;

	int snr = DEFAULT_SNR;

	if (is_multicast_ether_addr(dest)) {
		receiver = NULL;
	} else {
		receiver = get_interface_medium(medium, dest);
		if (receiver) {
			snr = medium->get_link_snr(medium, sender, receiver);
			// snr -= get_signal_offset_by_interference(medium,
			// 		sender->index, receiver->index);
			snr += get_fading_signal(medium);
		}
	}
	frame->signal = snr + medium->noise_level;

	noack = frame_is_mgmt(frame) || is_multicast_ether_addr(dest);

	// Find appropriate transmission rate by applying error fails until the
	// frame is acked. Also simulates the transmission time.
	for (i = 0; i < frame->tx_rates_count && !is_acked; i++) {
		rate_idx = frame->tx_rates[i].idx;

		/* no more rates in MRR */
		if (rate_idx < 0)
			break;

		error_prob =
			medium->get_error_prob(medium, snr, rate_idx,
					       frame->freq, frame->frame_len,
					       sender, receiver);
		for (j = 0; j < frame->tx_rates[i].count && !is_acked; j++) {
			send_time += difs + pkt_duration(frame->frame_len,
				index_to_rate(rate_idx, frame->freq));

			retries++;

			/* skip ack/backoff/retries for noack frames */
			if (noack) {
				is_acked = true;
				break;
			}

			/* TODO TXOPs */

			/* backoff */
			if (j > 0) {
				send_time += (cw * slot_time) / 2;
				cw = (cw << 1) + 1;
				if (cw > queue->cw_max)
					cw = queue->cw_max;
			}

			if (drand48() > error_prob)
				is_acked = true;

			send_time += ack_time_usec;
		}
	}
	if (is_acked) {
		frame->tx_rates[i-1].count = j + 1;
		for (; i < frame->tx_rates_count; i++) {
			frame->tx_rates[i].idx = -1;
			frame->tx_rates[i].count = 0;
		}
		frame->flags |= HWSIM_TX_STAT_ACK;
	}

	frame->duration = send_time;
	/* See deliver_queued_frames() for more details. */
	// If there is no current transmission, start sending now.
	if (medium->current_transmission == NULL) {
		medium->end_transmission = now;
		medium->current_transmission = frame;
		timespec_add_usec(&medium->end_transmission, frame->duration);

		/* Frames are only sent to mac80211_hwsim after they finish
		being transmitted in the medium. */
		struct itimerspec timer;
		memset(&timer, 0, sizeof(timer));
		timer.it_value = medium->end_transmission;
		timerfd_settime(medium->delivery_timerfd, TFD_TIMER_ABSTIME,
				&timer, NULL);
	}
	else {
		list_add_tail(&frame->list, &queue->frames);
	}
}

/* Report frame reception details for mac80211_hwsim. It sends information to
the transmitter and to the receiver interfaces. */
static int send_rx_info_nl(struct yawmd *ctx, struct frame *frame,
			    u32 rate_idx, struct recv_container *recv_info)
{
	struct nl_msg *msg;
	struct nl_sock *sock = ctx->socket;
	int ret;

	msg = nlmsg_alloc();
	if (!msg) {
		w_logf(ctx, LOG_ERR, "Error allocating new message MSG!\n");
		return -1;
	}

	if (genlmsg_put(msg, NL_AUTO_PID, NL_AUTO_SEQ, ctx->family_id, 0,
			NLM_F_REQUEST, HWSIM_YAWMD_RX_INFO,
			YAWMD_HWSIM_PROTO_VERSION) == NULL) {
		w_logf(ctx, LOG_ERR, "%s: genlmsg_put failed\n", __func__);
		ret = -1;
		goto out;
	}

	if (nla_put(msg, HWSIM_ATTR_ADDR_TRANSMITTER, ETH_ALEN,
	            frame->sender->hwaddr) ||
	    nla_put_u64(msg, HWSIM_ATTR_FRAME_ID, frame->cookie) ||
	    nla_put_u32(msg, HWSIM_ATTR_RX_RATE, rate_idx) ||
	    nla_put_u32(msg, HWSIM_ATTR_FREQ, frame->freq) ||
	    nla_put_u32(msg, HWSIM_ATTR_SIGNAL, frame->signal) ||
	    nla_put_u32(msg, HWSIM_ATTR_FLAGS, frame->flags) ||
	    nla_put(msg, HWSIM_ATTR_TX_INFO,
	            frame->tx_rates_count * sizeof(struct hwsim_tx_rate),
	            frame->tx_rates) ||
	    nla_put(msg, HWSIM_ATTR_RECEIVER_INFO,
	            recv_info_byte_len(recv_info), get_recv_info(recv_info))) {
		w_logf(ctx, LOG_ERR, "%s: Failed to fill a payload\n",
		       __func__);
		ret = -1;
		goto out;
	}

	w_logf(ctx, LOG_DEBUG,
	       "frame info sent from " MAC_FMT " to %d radios\n",
	       MAC_ARGS(frame->sender->hwaddr), recv_info->size);

	ret = nl_send_auto_complete(sock, msg);
	if (ret < 0) {
		w_logf(ctx, LOG_ERR, "%s: nl_send_auto failed\n", __func__);
		ret = -1;
		goto out;
	}
	ret = 0;


out:
	nlmsg_free(msg);

	return ret;
}

/* Fill the frame receiver's list. */
static void deliver_frame(struct medium *medium, struct frame *frame)
{
	struct yawmd *ctx = medium->ctx;
	u8 *dest = frame->header.addr1;
	u8 *src = frame->sender->addr;
	int rate_idx = 0;
	struct recv_container recv_info;
	create_recv_container(&recv_info, medium);

	// if simulation determined that this frame was successfully delivered
	if (frame->flags & HWSIM_TX_STAT_ACK) {
		/* rx the frame on the dest interface */
		for (unsigned int i = 0; i < medium->n_interfaces; i++) {
			struct interface *itf = &medium->interfaces[i];
			if (is_multicast_ether_addr(dest) &&
			    memcmp(src, itf->addr, ETH_ALEN) != 0) {
				int snr, signal;
				double error_prob;
				/*
				 * we may or may not receive this based on
				 * reverse link from sender -- check for
				 * each receiver.
				 */
				snr = medium->get_link_snr(medium, frame->sender,
							   itf);
				snr += get_fading_signal(medium);
				signal = snr + medium->noise_level;
				if (signal < DEFAULT_CCA_THRESHOLD)
					continue;

				// // always returns 0 because of the test
				// // signal >= CCA_THRESHOLD
				// if (set_interference_duration(
				// 	    ctx, frame->sender->index,
				// 	    frame->duration, signal))
				// 	continue;

				// snr -= get_signal_offset_by_interference(
				// 	ctx, frame->sender->index,
				// 	station->index);

				rate_idx = frame->tx_rates[0].idx;
				error_prob = medium->get_error_prob(medium,
					(double)snr, rate_idx, frame->freq,
					frame->frame_len, frame->sender, itf);

				if (drand48() <= error_prob) {
					w_logf(ctx, LOG_INFO,
					       "Dropped mcast from " MAC_FMT
					       " to " MAC_FMT " at receiver\n",
					       MAC_ARGS(src),
					       MAC_ARGS(itf->addr));
					continue;
				}

				add_recv_info(&recv_info, itf->hwaddr,
				              frame->signal);
			}
			// if current station is destination of frame
			else if (memcmp(dest, itf->addr, ETH_ALEN) == 0) {

				// // if TRUE: signal < CCA_THRESHOLD
				// // no transmission is sent
				// // if FALSE: interference is off or
				// // signal >= CCA_THRESHOLD
				// if (set_interference_duration(
				// 	    ctx, frame->sender->index,
				// 	    frame->duration, frame->signal))
				// 	continue;

				rate_idx = frame->tx_rates[0].idx;

				add_recv_info(&recv_info, itf->hwaddr,
				              frame->signal);
			}
		}
	}
	// else { // if !(frame->flags & HWSIM_TX_STAT_ACK)
	// 	// frame is not sent
	// 	set_interference_duration(ctx, frame->sender->index,
	// 				  frame->duration, frame->signal);
	// }

	send_rx_info_nl(ctx, frame, rate_idx, &recv_info);

	delete_container(&recv_info);
}

/* Find the highest priority frame queued and remove it from the queue. */
static inline struct frame * next_frame(struct medium *medium)
{
	struct frame *frame = NULL;
	for (unsigned int i = 0; i < IEEE80211_NUM_ACS; i++) {
		frame = list_first_entry_or_null(&medium->qos_queues[i].frames,
						 struct frame, list);
		if (frame != NULL) {
			list_del(&frame->list);
			break;
		}
	}
	return frame;
}

/* Deliver the frame that finished being transmitted and all the frames that
should already have been transmitted. Set the timer for the end of transmission
of the next frame. */
static void deliver_queued_frames(struct medium *medium)
{
	/* Frames are only sent to mac80211_hwsim after they finish being
	transmitted.*/
	/* This procedure is only called after ctx.timerfd is set and expires. */
	/* There are two cases when a frame is received. Either there is no
	current transmission, which means that the ctx.qos_queues are also
	empty, or there is a current transmission.
	When there is no current transmission (ctx.current_transmission == NULL)
	the frame is placed there, and the timer (ctx.timerfd) is set for
	$Now + frame.duration. If there was a ctx.current_transmission when the
	frame arrived, it is just placed at ctx.qos_queues. (This happens at
	queue_frame()).

	When ctx.timerfd expires, this procedure is called. The frame
	ctx.current_transmission is sent to mac80211_hwsim. After this, there
	can be two cases: ctx.qos_queues contains frames, or ctx.qos_queues are
	empty. If ctx.qos_queues contain frames, one is selected and set as the
	ctx.current_transmission, and the timer is set as ctx.end_transmission.
	If there are no frames, the timer (ctx.timerfd) is not set, and this
	procedure is only called after a frame arrives and queue_frame() sets
	the timer again.
	In the case where the ctx.qos_queues contain frames, it is important to
	make up for lost time, because, there is an interval between the
	timestamp of the timer (ctx.timerfd) and when the process actually
	executes again. (So, to prevent unintentional transmission gaps, we go
	into the past). In this case, the timestamp ctx.end_transmission of the
	current frame is always calculated by adding the frame.duration to the
	last frame's ctx.end_transmission, and all the frames that end up with
	a ctx.end_transmission before the current timestamp (now), are sent
	without making use of the timer (ctx.timerfd). The timer is only set,
	when ctx.end_transmission is later than the variable now.
	*/
	
	struct timespec now;
	clock_gettime(CLOCK_MONOTONIC, &now);

	// Deliver the frame that finished being transmitted.
	deliver_frame(medium, medium->current_transmission);
	free(medium->current_transmission);

	medium->current_transmission = next_frame(medium);

	if (medium->current_transmission == NULL)
		return;

	// Transmit all the frames delayed.
	do {
		timespec_add_usec(&medium->end_transmission,
				  medium->current_transmission->duration);
		// If the end of delivery time is in the future set timer instead
		if (!timespec_before(&medium->end_transmission, &now))
			break;
		deliver_frame(medium, medium->current_transmission);
		free(medium->current_transmission);
		medium->current_transmission = next_frame(medium);
	} while (medium->current_transmission != NULL
		 && timespec_before(&medium->end_transmission, &now));

	if (medium->current_transmission == NULL)
		return;
	
	struct itimerspec timer;
	memset(&timer, 0, sizeof(timer));
	timer.it_value = medium->end_transmission;
	timerfd_settime(medium->delivery_timerfd, TFD_TIMER_ABSTIME, &timer,
			NULL);
}

// static void deliver_expired_frames(struct medium *medium)
//	... (replaced)
//	Remaining code was to update interference

// 	// int duration;
// 	// struct timespec _diff;
// 	// if (!ctx->intf)
// 	// 	return;

// 	// timespec_sub(&now, &ctx->intf_updated, &_diff);
// 	// duration = (_diff.tv_sec * 1000000) + (_diff.tv_nsec / 1000);
// 	// if (duration < 10000) // calc per 10 msec
// 	// 	return;

// 	// // update interference
// 	// for (int i = 0; i < ctx->num_stas; i++)
// 	// 	for (int j = 0; j < ctx->num_stas; j++) {
// 	// 		if (i == j)
// 	// 			continue;
// 	// 		// probability is used for next calc
// 	// 		ctx->intf[i * ctx->num_stas + j].prob_col =
// 	// 			ctx->intf[i * ctx->num_stas + j].duration /
// 	// 			(double)duration;
// 	// 		ctx->intf[i * ctx->num_stas + j].duration = 0;
// 	// 	}

// 	// clock_gettime(CLOCK_MONOTONIC, &ctx->intf_updated);
// }

static int nl_err_cb(struct sockaddr_nl *nla, struct nlmsgerr *nlerr, void *arg)
{
	struct genlmsghdr *gnlh = nlmsg_data(&nlerr->msg);
	struct yawmd *ctx = arg;

	w_flogf(ctx, LOG_ERR, stderr, "nl: cmd %d, seq %d: %s\n", gnlh->cmd,
			nlerr->msg.nlmsg_seq, strerror(abs(nlerr->error)));

	return NL_SKIP;
}

/* Handle messages from mac80211_hwsim. Process CMD_FRAME events and queue them
for later delivery with the scheduler. */
static int process_messages_cb(struct nl_msg *msg, void *arg)
{
	struct yawmd *ctx = arg;
	struct nlattr *attrs[HWSIM_ATTR_MAX+1];
	/* netlink header */
	struct nlmsghdr *nlh = nlmsg_hdr(msg);
	/* generic netlink header*/
	struct genlmsghdr *gnlh = nlmsg_data(nlh);

	struct interface *sender;
	struct frame *frame;
	struct ieee80211_hdr *hdr;
	u8 *src;

	if (gnlh->cmd == HWSIM_YAWMD_TX_INFO) {
		// pthread_rwlock_rdlock(&snr_lock);
		/* we get the attributes*/
		genlmsg_parse(nlh, 0, attrs, HWSIM_ATTR_MAX, NULL);
		if (attrs[HWSIM_ATTR_ADDR_TRANSMITTER]) {
			u8 *hwaddr = (u8 *)nla_data(
				attrs[HWSIM_ATTR_ADDR_TRANSMITTER]);

			unsigned int data_len =
				nla_get_u32(attrs[HWSIM_ATTR_FRAME_LENGTH]);
			char *data =
				(char *)nla_data(attrs[HWSIM_ATTR_FRAME_HEADER]);
			unsigned int flags =
				nla_get_u32(attrs[HWSIM_ATTR_FLAGS]);
			unsigned int tx_rates_len =
				nla_len(attrs[HWSIM_ATTR_TX_INFO]);
			struct hwsim_tx_rate *tx_rates =
				(struct hwsim_tx_rate *)
				nla_data(attrs[HWSIM_ATTR_TX_INFO]);
			u64 cookie = nla_get_u64(attrs[HWSIM_ATTR_FRAME_ID]);
			u32 freq = nla_get_u32(attrs[HWSIM_ATTR_FREQ]);

			hdr = (struct ieee80211_hdr *)data;
			src = hdr->addr2;

			if (data_len < 6 + 6 + 4)
				goto out;

			sender = get_interface(ctx, src);
			if (sender == NULL) {
				w_flogf(ctx, LOG_ERR, stderr, "Unable to find sender station " MAC_FMT "\n", MAC_ARGS(src));
				goto out;
			}
			memcpy(sender->hwaddr, hwaddr, ETH_ALEN);

			frame = malloc(sizeof(struct frame));
			if (!frame)
				goto out;

			memcpy(&frame->header, data, sizeof(struct ieee80211_hdr));
			frame->frame_len = data_len;
			frame->flags = flags;
			frame->cookie = cookie;
			frame->freq = freq;
			frame->sender = sender;
			sender->frequency = freq;
			frame->tx_rates_count =
				tx_rates_len / sizeof(struct hwsim_tx_rate);
			memcpy(frame->tx_rates, tx_rates,
			       min(tx_rates_len, sizeof(frame->tx_rates)));
			
			if (ctx->threads) {
				struct medium *medium = sender->medium;
				pthread_mutex_lock(&medium->queue_mutex);
				list_add_tail(&frame->list, &medium->frame_queue);
				timerfd_settime(medium->queue_timerfd, 0,
						&it_1ns, NULL);
				pthread_mutex_unlock(&medium->queue_mutex);
			}
			else {
				queue_frame(frame);
			}
		}
out:
		//pthread_rwlock_unlock(&snr_lock);
		return 0;

	}
	return 0;
}

static void thread_queue_frame(int fd, short what, void *data) {
	struct medium *medium = (struct medium *) data;

	uint64_t u;
	read(fd, &u, sizeof(u));

	// Limit the amount of frames processed in one go.
	struct frame *frame;
	for (unsigned i = 0; i < 5; i++) {
		pthread_mutex_lock(&medium->queue_mutex);
		frame = list_first_entry_or_null(&medium->frame_queue,
						 struct frame, list);
		if (frame != NULL)
			list_del(&frame->list);
		pthread_mutex_unlock(&medium->queue_mutex);
		if (frame != NULL)
			queue_frame(frame);
		else
			break;
	}

	// If the queue is not empty make libevent call again, but allow to
	// process other events.
	if (frame != NULL) {
		pthread_mutex_lock(&medium->queue_mutex);
		frame = list_first_entry_or_null(&medium->frame_queue,
							struct frame, list);
		if (frame != NULL)
			timerfd_settime(medium->queue_timerfd, 0, &it_1ns, NULL);
		pthread_mutex_unlock(&medium->queue_mutex);
	}
}

/* Register with the kernel to start receiving new frames. */
int send_register_msg(struct yawmd *ctx)
{
	struct nl_sock *sock = ctx->socket;
	struct nl_msg *msg;
	int ret;

	msg = nlmsg_alloc();
	if (!msg) {
		w_logf(ctx, LOG_ERR, "Error allocating new message MSG!\n");
		return -1;
	}

	if (genlmsg_put(msg, NL_AUTO_PID, NL_AUTO_SEQ, ctx->family_id,
			0, NLM_F_REQUEST, HWSIM_CMD_REGISTER,
			YAWMD_HWSIM_PROTO_VERSION) == NULL) {
		w_logf(ctx, LOG_ERR, "%s: genlmsg_put failed\n", __func__);
		ret = -1;
		goto out;
	}

	ret = nl_send_auto_complete(sock, msg);
	if (ret < 0) {
		w_logf(ctx, LOG_ERR, "%s: nl_send_auto failed\n", __func__);
		ret = -1;
		goto out;
	}
	ret = 0;

out:
	nlmsg_free(msg);
	return ret;
}

static void sock_event_cb(int fd, short what, void *data)
{
	struct yawmd *ctx = data;

	nl_recvmsgs_default(ctx->socket);
}

/* Setup netlink socket and callbacks. */
static int init_netlink(struct yawmd *ctx)
{
	struct nl_sock *sock;
	int ret;

	ctx->cb = nl_cb_alloc(NL_CB_CUSTOM);
	if (!ctx->cb) {
		w_logf(ctx, LOG_ERR, "Error allocating netlink callbacks\n");
		return -1;
	}

	sock = nl_socket_alloc_cb(ctx->cb);
	if (!sock) {
		w_logf(ctx, LOG_ERR, "Error allocating netlink socket\n");
		return -1;
	}

	ctx->socket = sock;

	ret = genl_connect(sock);
	if (ret < 0) {
		w_logf(ctx, LOG_ERR, "Error connecting netlink socket ret=%d\n", ret);
		return -1;
	}

	ctx->family_id = genl_ctrl_resolve(sock, "MAC80211_HWSIM");
	if (ctx->family_id < 0) {
		w_logf(ctx, LOG_ERR, "Family MAC80211_HWSIM not registered\n");
		return -1;
	}

	nl_cb_set(ctx->cb, NL_CB_MSG_IN, NL_CB_CUSTOM, process_messages_cb, ctx);
	nl_cb_err(ctx->cb, NL_CB_CUSTOM, nl_err_cb, ctx);

	return 0;
}

/* Print the CLI help */
static void print_help(int exval)
{
	printf("yawmd (version %d.%d) - a wireless medium simulator\n",
	       YAWMD_VERSION_MAJOR, YAWMD_VERSION_MINOR);
	printf("yawmd [-h] [-V] [-s] [-l LOG_LVL] [-x FILE] -c FILE\n\n");

	printf("  -h              print this help and exit\n");
	printf("  -V              print version and exit\n\n");

	printf("  -l LOG_LVL      set the logging level\n");
	printf("                  LOG_LVL: RFC 5424 severity, values 0 - 7\n");
	printf("                  >= 3: errors are logged\n");
	printf("                  >= 5: startup msgs are logged\n");
	printf("                  >= 6: dropped packets are logged (default)\n");
	printf("                  == 7: all packets will be logged\n");
	printf("  -c FILE         set input config file\n");
	printf("  -t              simulate mediums in different threads\n");
	// printf("  -x FILE         set input PER file\n");
	// printf("  -s              start the server on a socket\n");
	// printf("  -d              use the dynamic complex mode\n");
	// printf("                  (server only with matrices for each connection)\n");

	exit(exval);
}

static void movement_timer_cb(int fd, short what, void *data) {
	struct medium *medium = data;
	uint64_t u;

	read(fd, &u, sizeof(u));

	//printf("movement_timer_cb for medium id=%d\n", medium->id);

	medium->move_interfaces(medium);
	timespec_add_seconds(&medium->move_time.it_value, medium->move_interval);
	timerfd_settime(medium->move_timerfd, TFD_TIMER_ABSTIME,
			&medium->move_time, NULL);
	//dump_medium_info(medium);
	return;
}

static void delivery_timer_cb(int fd, short what, void *data)
{
	struct medium *medium = data;
	uint64_t u;

	read(fd, &u, sizeof(u));

	deliver_queued_frames(medium);
}

/* Initialize event timers when running with only one thread */
static void init_event_timers(struct yawmd *ctx) {
	struct medium *m;
	struct timespec now;
	struct itimerspec it;

	clock_gettime(CLOCK_MONOTONIC, &now);
	it.it_interval.tv_sec = 0;
	it.it_interval.tv_nsec = 0;
	// FIXME: 20sec delay before starting movement
	// TODO: make it a configuration from the file?
	it.it_value.tv_sec = now.tv_sec + 20;
	it.it_value.tv_nsec = now.tv_nsec;

	list_for_each_entry(m, &ctx->medium_list, list) {
		m->delivery_timerfd = timerfd_create(CLOCK_MONOTONIC, 0);
		event_assign(&m->delivery_event, ctx->ev_base,
			     m->delivery_timerfd, EV_READ | EV_PERSIST,
			     delivery_timer_cb, m);
		event_add(&m->delivery_event, NULL);

		if (m->move_interfaces == NULL)
			continue;
		m->move_timerfd = timerfd_create(CLOCK_MONOTONIC, 0);
		event_assign(&m->move_event, ctx->ev_base, m->move_timerfd,
			     EV_READ | EV_PERSIST, movement_timer_cb, m);
		event_add(&m->move_event, NULL);
		timerfd_settime(m->move_timerfd, TFD_TIMER_ABSTIME, &it, NULL);
		m->move_time = it;
	}
	return;
}

/* Initialize event timers when running with multiple threads. */
static void init_threads_event_timers(struct medium *medium,
				      struct event_base *ev_base)
{
	struct timespec now;
	struct itimerspec it;

	medium->queue_timerfd = timerfd_create(CLOCK_MONOTONIC, 0);
	event_assign(&medium->queue_event, ev_base, medium->queue_timerfd,
		     EV_READ | EV_PERSIST, thread_queue_frame, medium);
	event_add(&medium->queue_event, NULL);

	medium->delivery_timerfd = timerfd_create(CLOCK_MONOTONIC, 0);
	event_assign(&medium->delivery_event, ev_base,
		     medium->delivery_timerfd, EV_READ | EV_PERSIST,
		     delivery_timer_cb, medium);
	event_add(&medium->delivery_event, NULL);

	if (medium->move_interfaces != NULL) {
		clock_gettime(CLOCK_MONOTONIC, &now);
		it.it_interval.tv_sec = 0;
		it.it_interval.tv_nsec = 0;
		// FIXME: 20sec delay before starting move
		// TODO: make it a configuration from the file?
		it.it_value.tv_sec = now.tv_sec + 20;
		it.it_value.tv_nsec = now.tv_nsec;
		medium->move_timerfd = timerfd_create(CLOCK_MONOTONIC, 0);
		event_assign(&medium->move_event, ev_base, medium->move_timerfd,
			     EV_READ | EV_PERSIST, movement_timer_cb, medium);
		event_add(&medium->move_event, NULL);
		timerfd_settime(medium->move_timerfd, TFD_TIMER_ABSTIME, &it,
				NULL);
		medium->move_time = it;
	}
}

void *thread_main(void *arg)
{
	struct medium *medium = (struct medium *) arg;
	struct event_base *ev_base;	

	ev_base = event_base_new();
	if (ev_base == NULL) {
		printf("Error creating event_base in medium id=%d.\n",
		       medium->id);
		exit(1);
	}

	INIT_LIST_HEAD(&medium->frame_queue);
	init_threads_event_timers(medium, ev_base);
	pthread_mutex_init(&medium->queue_mutex, NULL);

	event_base_dispatch(ev_base);

	event_base_free(ev_base);
	pthread_mutex_destroy(&medium->queue_mutex);

	return NULL;
}

int main(int argc, char *argv[])
{
	int opt;
	struct event ev_cmd;
	struct yawmd ctx;
	char *config_file = NULL;
	// char *per_file = NULL;

	setvbuf(stdout, NULL, _IOLBF, BUFSIZ);

	if (argc == 1) {
		fprintf(stderr, "This program needs arguments....\n\n");
		print_help(EXIT_FAILURE);
	}

	ctx.log_level = YAWMD_DEFAULT_LOG_LEVEL;
	unsigned long int parse_log_lvl;
	char* parse_end_token;
	// bool start_server = false;
	// bool full_dynamic = false;
	ctx.threads = false;

	//while ((opt = getopt(argc, argv, ":hVc:l:x:sd:t")) != -1) {
	while ((opt = getopt(argc, argv, ":hVc:l:t")) != -1) {
		switch (opt) {
		case 'h':
			print_help(EXIT_SUCCESS);
			break;
		case 'V':
			printf("yawmd version %d.%d - a wireless medium simulator for mac80211_hwsim\n"
				"Communication protocol with mac80211_hwsim version %d.\n",
				YAWMD_VERSION_MAJOR, YAWMD_VERSION_MINOR,
				YAWMD_HWSIM_PROTO_VERSION);
			exit(EXIT_SUCCESS);
			break;
		case 'c':
			config_file = optarg;
			break;
		// case 'x':
		// 	printf("Input packet error rate file: %s\n", optarg);
		// 	per_file = optarg;
		// 	break;
		case ':':
			printf("yawmd: Error - Option `%c' "
			       "needs a value\n\n", optopt);
			print_help(EXIT_FAILURE);
			break;
		case 'l':
			parse_log_lvl = strtoul(optarg, &parse_end_token, 10);
			if ((parse_log_lvl == ULONG_MAX && errno == ERANGE) ||
			     optarg == parse_end_token || parse_log_lvl > 7) {
				printf("yawmd: Error - Invalid RFC 5424 severity level: "
							   "%s\n\n", optarg);
				print_help(EXIT_FAILURE);
			}
			ctx.log_level = parse_log_lvl;
			break;
		// case 'd':
		// 	full_dynamic = true;
		// 	break;
		// case 's':
		// 	start_server = true;
		// 	break;
		case 't':
			ctx.threads = true;
			break;
		case '?':
			printf("yawmd: Error - No such option: "
			       "`%c'\n\n", optopt);
			print_help(EXIT_FAILURE);
			break;
		}

	}

	if (optind < argc)
		print_help(EXIT_FAILURE);

	// if (full_dynamic) {
	// 	if (config_file) {
	// 		printf("%s: cannot use dynamic complex mode with config file\n", argv[0]);
	// 		print_help(EXIT_FAILURE);
	// 	}

	// 	if (!start_server) {
	// 		printf("%s: dynamic complex mode requires the server option\n", argv[0]);
	// 		print_help(EXIT_FAILURE);
	// 	}

	// 	w_logf(&ctx, LOG_NOTICE, "Using dynamic complex mode instead of config file\n");
	// } else {
		if (!config_file) {
			printf("%s: config file must be supplied\n", argv[0]);
			print_help(EXIT_FAILURE);
		}

		w_logf(&ctx, LOG_NOTICE, "Input configuration file: %s\n", config_file);
	//}

	INIT_LIST_HEAD(&ctx.medium_list);
	if (!configure(config_file, &ctx))
		return EXIT_FAILURE;

	// This will be used to set the timers that signals medium.queue_event
	// that a new frame for processing is available at medium.frame_queue.
	// FIXME: This a bit wasteful of resources, because it is being set a
	// timer to trigger 1ns later to write info in a file descriptor that
	// is being monitored by libevent. At least the timer can be bypassed.
	// Alternatively, event_active() (see 
	// http://www.wangafu.net/~nickm/libevent-book/Ref4_event.html)
	// can be used directly for the signal.
	it_1ns.it_interval.tv_nsec = 0;
	it_1ns.it_interval.tv_sec = 0;
	it_1ns.it_value.tv_sec = 0;
	it_1ns.it_value.tv_nsec = 1;

	/* init libevent */
	ctx.ev_base = event_base_new();
	if (ctx.ev_base == NULL) {
		printf("Error initializing libevent base.\n");
		return EXIT_FAILURE;
	}

	/* init netlink */
	if (init_netlink(&ctx) < 0)
		return EXIT_FAILURE;

	event_assign(&ev_cmd, ctx.ev_base, nl_socket_get_fd(ctx.socket),
		     EV_READ | EV_PERSIST, sock_event_cb, &ctx);
	event_add(&ev_cmd, NULL);

	/* setup timers */
	if (ctx.threads) {
	struct medium *m;
		list_for_each_entry(m, &ctx.medium_list, list) {
			if (pthread_create(&m->thread, NULL, thread_main, m) != 0) {
				printf("Error creating thread for medium id %d\n",
				m->id);
				exit(1); // FIXME: no cleanup
			}
		}
	}
	else {
		init_event_timers(&ctx);
	}

	/* register for new frames */
	if (send_register_msg(&ctx) == 0) {
		w_logf(&ctx, LOG_NOTICE, "REGISTER SENT!\n");
	}

	// if (start_server == true)
	// 	start_yserver(&ctx);

	/* enter libevent main loop */
	event_base_dispatch(ctx.ev_base);
	// FIXME: Add signal handler to event loop, so that it can be executed
	// code to perform the cleanup.
	// See "Constructing signal events" at 
	// http://www.wangafu.net/~nickm/libevent-book/Ref4_event.html.
	// See event_base_loopbreak() to break the loop in the signal handler.
	// It is necessary to look into terminating the loops of the threads and
	// the threads themselves.
	
	
	// if (start_server == true)
	// 	stop_yserver();

	event_base_free(ctx.ev_base);
	free(ctx.socket);
	free(ctx.cb);
	// free(ctx.intf);
	// free(ctx.per_matrix);
	
	delete_mediums(&ctx);

	return EXIT_SUCCESS;
}
