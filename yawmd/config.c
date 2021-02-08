/*
 *	yawmd, wireless medium simulator for the Linux module mac80211_hwsim
 *	Copyright (c) 2011 cozybit Inc.
 *	Copyright (c) 2021 Miguel Moreira
 *
 *	Author: Javier Lopez    <jlopex@cozybit.com>
 *		Javier Cardona  <javier@cozybit.com>
 *		Miguel Moreira  <mmoreira@tutanota.com>
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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <libconfig.h>
#include <math.h>
#include "config.h"

static const double FREQ_CH1 = 2.412e9; // [Hz]
static const double SPEED_LIGHT = 2.99792458e8; // [meter/sec]

static const char *setting_must_be_bool =
	"Setting %s (%s:%u) must be a boolean.\n";
static const char *setting_must_be_int =
	"Setting %s (%s:%u) must be an integer.\n";
static const char *setting_must_be_float =
	"Setting %s (%s:%u) must be a float.\n";
static const char *setting_must_be_list =
	"Setting %s (%s:%u) must be a list: ( ... ).\n";
static const char *setting_must_be_array =
	"Setting %s (%s:%u) must be an array: [ ... ].\n";
static const char *setting_must_be_group =
	"Setting %s (%s:%u) must be a group: { ... }.\n";
static const char *setting_must_be_string =
	"Setting %s (%s:%u) must be a string: \" ... \".\n";
static const char *setting_ignore_unknown =
	"Ignoring unknown setting \"%s\" (%s:%d).\n";
// TODO: setting required string - with full path

enum model_type { MODEL_SNR,
		  MODEL_PROB,
		  MODEL_PATH_LOSS };
static const char * const model_type_str[] = { "snr", 
					       "prob",
					       "path_loss" };

enum model_name { MN_FREE_SPACE,
		  MN_ITU,
		  MN_LOG_DISTANCE,
		  MN_TWO_RAY_GROUND, 
		  MN_LOG_NORMAL_SHADOWING,
		  MN_SNR,
		  MN_PROB };
static const char * const model_name_str[] = { "free_space",
					       "itu",
					       "log_distance",
					       "two_ray_ground",
					       "log_normal_shadowing",
					       "snr_matrix",
					       "prob_matrix" };

enum model_sett {
	MODEL_TYPE,
	MODEL_DEFAULT_SNR,
	MODEL_LINKS,
	MODEL_DEFAULT_PROBABILITY,
	MODEL_SIMULATE_INTERFERENCE,
	MODEL_NOISE_LEVEL,
	MODEL_FADING_COEFFICIENT,
	MODEL_POSITIONS,
	MODEL_MOVE_INTERVAL,
	MODEL_DIRECTIONS,
	MODEL_TX_POWERS,
	MODEL_ANTENNA_GAIN,
	MODEL_ISNODEAPS,
	MODEL_MODEL_NAME,
	MODEL_MODEL_PARAMETERS,
	__MODEL_SETTING_SIZE
};
static const char * const model_sett_str[] = {
	"type",
	"default_snr",
	"links",
	"default_probability",
	"simulate_interference",
	"noise_level",
	"fading_coefficient",
	"positions",
	"move_interval",
	"directions",
	"tx_powers",
	"antenna_gain",
	"isnodeaps",
	"model_name",
	"model_params"
};

enum { CONFIGURE_POSITIONS, CONFIGURE_DIRECTIONS };


static char *config_setting_path(config_setting_t *setting);
static unsigned int config_setting_length2(config_setting_t *setting);
static bool configure_medium(config_setting_t *medium, struct medium *info);
static bool configure_model(config_setting_t *model, struct medium *info);
static bool configure_model_snr(config_setting_t *model,
			       struct medium *info, bool *setting_present);
static bool configure_model_prob(config_setting_t *model,
			        struct medium *info,
			        bool *setting_present);
static bool configure_model_path_loss(config_setting_t *model,
			             struct medium *info,
			             bool *setting_present);
static bool configure_links(config_setting_t *links, void *value_matrix,
			    unsigned n_interfaces, int model_type);
static bool configure_positions_directions(config_setting_t *list,
					   struct medium *info,
					   int pos_dir);
static bool configure_model_type(config_setting_t *name,
				 config_setting_t *params,
				 struct medium *info);
static char *model_ignore_message(const char *parent_path,
				      const char *model_type);
static unsigned int number_char_length(unsigned int n);
static void print_matrix_int(int *matrix, unsigned int rows, unsigned int cols);
static void print_matrix_double(double *matrix, unsigned int rows,
				unsigned int cols);
//static void dump_medium_info(struct medium* info);
static struct medium* new_medium_info(void);
static void delete_medium_info(struct medium *mi);
static void medium_init_qos_queues(struct medium *medium);
static void wqueue_init(struct wqueue *wqueue, int cw_min, int cw_max);
static int get_link_snr_default(struct medium *medium, struct interface *sender,
				struct interface *receiver);
static int get_link_snr_from_snr_matrix(struct medium *medium,
					struct interface *sender,
					struct interface *receiver);
static double _get_error_prob_from_snr(struct medium *medium, double snr,
				       unsigned int rate_idx, u32 freq,
				       int frame_len, struct interface *src,
				       struct interface *dst);
static double get_error_prob_from_matrix(struct medium *medium, double snr,
					 unsigned int rate_idx, u32 freq,
					 int frame_len, struct interface *src,
					 struct interface *dst);
static void recalc_path_loss(struct medium *medium);
static void move_interfaces(struct medium *medium);
static int calc_path_loss_free_space(struct medium *medium,
				     struct interface *src,
				     struct interface *dst);
static int calc_path_loss_log_distance(struct medium *medium,
				       struct interface *src,
				       struct interface *dst);
static int calc_path_loss_itu(struct medium *medium, struct interface *src,
			      struct interface *dst);
static int calc_path_loss_log_normal_shadowing(struct medium *medium,
					       struct interface *src,
					       struct interface *dst);
static int calc_path_loss_two_ray_ground(struct medium *medium,
					 struct interface *src,
					 struct interface *dst);

extern double get_error_prob_from_snr(double snr, unsigned int rate_idx,
				      u32 freq, int frame_len);

/**
 * @brief Compare two mac addresses.
 * 
 * @param mac1 
 * @param mac2 
 * @param length address length in bytes
 * @return true if mac1 == mac2
 */
static bool compare_mac_addr(unsigned char *mac1, unsigned char *mac2,
		 	     unsigned int length)
{
	bool same = true;
	for (unsigned int j = 0; j < length; j++)
		same = same && (mac1[j] == mac2[j]);
	return same;
}


/**
 * @brief Check if mac_address is already present in array of interface.
 * 
 * @param mac_addr 
 * @param interface array of struct interface
 * @param mac_addr_len 
 * @param interface_length 
 * @return true if parameter mac_addr appears in parameter interface, false
 * otherwise.
 */
static bool check_repeated_mac_addr(unsigned char *mac_addr,
				    struct interface *interface,
				    unsigned int mac_addr_len,
				    unsigned int interface_length)
{
	for (unsigned int i = 0; i < interface_length; i++) {
		if (compare_mac_addr(interface[i].addr, mac_addr,
				     mac_addr_len)) {
			return true;
		}
	}
	return false;	
}

static bool check_id_mac_addr_repetitions(struct yawmd *mediums,
				          struct medium *info)
{
	struct medium *it = NULL;
	list_for_each_entry(it, &(mediums->medium_list), list) {
		if (it == info) continue;
		if (it->id == info->id) {
			fprintf(stderr,
				"Repeated medium id %d\n",
				info->id);
			return true;
		}
		for (unsigned k = 0; k < info->n_interfaces; k++) {
			if (check_repeated_mac_addr(
					info->interfaces[k].addr,
					it->interfaces, ETH_ALEN,
					info->n_interfaces)) {
				fprintf(stderr, "Repeated mac address: "
					MAC_FMT "\n",
					MAC_ARGS(info->interfaces[k].addr));
				return true;
			}
		}
	}
	return false;
}

/**
 * @brief Configure yawmd according to the parameter file.
 * 
 * @param file_name 
 * @param ctx 
 * @return true if configuration was successfully loaded, false otherwise.
 */
bool configure(char *file_name, struct yawmd *ctx)
{
	config_t cfg;
	config_setting_t *root = NULL, *medium = NULL;

	config_init(&cfg);

	// report errors in the file sintax
	if (!config_read_file(&cfg, file_name)) {
		fprintf(stderr, "%s:%d - %s\n", config_error_file(&cfg),
			config_error_line(&cfg), config_error_text(&cfg));
		config_destroy(&cfg);
		goto exit_failure;
	}

	// report unknown setting and check the existence of "medium" setting
	root = config_root_setting(&cfg);
	for (unsigned int i = 0; i < config_setting_length2(root); i++) {
		config_setting_t *s = config_setting_get_elem(root, i);
		char *s_name = config_setting_name(s);
		if (strcmp(s_name, "medium") == 0) {
			medium = s;
		} else {
			fprintf(stdout,
				"Ignoring unknown setting \"%s\" "
				"(%s:%d).\n",
				s_name, config_setting_source_file(s),
				config_setting_source_line(s));
		}
	}
	if (medium == NULL) {
		fprintf(stderr, "\"medium\" list not found in %s\n",
			config_setting_source_file(root));
		goto exit_failure;
	}
	if (config_setting_type(medium) != CONFIG_TYPE_LIST) {
		fprintf(stderr, "Setting \"medium\" (%s:%d) must be a list!\n",
			config_setting_source_file(medium),
			config_setting_source_line(medium));
		goto exit_failure;
	}

	if (config_setting_length2(medium) == 0) {
		fprintf(stderr,
			"Setting \"medium\" (%s:%d) must contain at "
			"least one element.\n",
			config_setting_source_file(medium),
			config_setting_source_line(medium));
		goto exit_failure;
	}

	ctx->n_mediums = 0;
	for (unsigned int i = 0; i < config_setting_length2(medium); i++) {
		printf("medium[%d]:\n", i); // FIXME: REMOVE ?
		config_setting_t *s = config_setting_get_elem(medium, i);

		if (!config_setting_is_group(s)) {
			fprintf(stderr,
				"Setting \"medium\" (%s:%d) list "
				"members must be of type group: { ... }\n",
				config_setting_source_file(s),
				config_setting_source_line(s));
			goto exit_mediums;
		}

		struct medium *info = new_medium_info();
		list_add_tail(&(info->list), &(ctx->medium_list));
		if (!configure_medium(s, info)) {
			fprintf(stdout, "Failure to configure medium.\n");
			goto exit_mediums;
		}
		dump_medium_info(info);
		// check repetitions of mac or medium id between mediums
		if (check_id_mac_addr_repetitions(ctx, info))
			goto exit_mediums;
		
		info->ctx = ctx;
		ctx->n_mediums++;
		fprintf(stdout, "Medium configuration loaded successfully.\n");
	}

	// guardar os meios no wmediumd

	fprintf(stdout, "Configuration successfully loaded!\n");
	return true;

exit_mediums:
	delete_mediums(ctx);
exit_failure:
	config_destroy(&cfg);
	return false;
}

/**
 * @brief Check and configure one member of the list "medium" from the
 * configuration file.
 * Should be called for each of the members of the list "medium".
 * 
 * @param medium - one of the elements of the list medium
 * @return int 
 */
static bool configure_medium(config_setting_t *medium, struct medium *info)
{
	bool id = false, interfaces = false, model = false;

	/* ---- Verify existence of settings and report unknowns */
	for (unsigned int i = 0; i < config_setting_length2(medium); i++) {
		config_setting_t *e = config_setting_get_elem(medium, i);
		char *name = config_setting_name(e);

		//printf("setting: %s\n", name);

		if (strcmp(name, "id") == 0) {
			id = true;
		} else if (strcmp(name, "interfaces") == 0) {
			interfaces = true;
		} else if (strcmp(name, "model") == 0) {
			model = true;
		} else {
			fprintf(stdout,
				"Ignoring unknown setting: \"%s\" (%s:%d).\n",
				name, config_setting_source_file(e),
				config_setting_source_line(e));
		}
	}

	/* ---- Verify presence of mandatory settings */
	if (!(id && interfaces && model)) {
		fprintf(stderr, "Not all configuration is present!\n");
		if (!id)
			fprintf(stderr, "Setting \"id\" required.\n");
		if (!interfaces)
			fprintf(stderr, "Setting \"interfaces\" required.\n");
		if (!model)
			fprintf(stderr, "Setting \"model\" required.\n");
		return false;
	}

	/* ---- check type and value range, and store them */

	// id
	if (config_setting_lookup_int(medium, "id", &(info->id)) == 
	    CONFIG_FALSE) {
		config_setting_t *s = config_setting_lookup(medium, "id");
		fprintf(stderr, "Setting \"id\" (%s:%d) must be an integer\n",
			config_setting_source_file(s),
			config_setting_source_line(s));
		return false;
	}

	// interfaces
	config_setting_t *itf = config_setting_lookup(medium, "interfaces");
	if (!config_setting_is_array(itf)) {
		fprintf(stderr,
			"Setting \"interfaces\" (%s:%d) must be an array: "
			"[ ... ]\n",
			config_setting_source_file(itf),
			config_setting_source_line(itf));
		return false;
	}

	// if a failure happens it is freed when delete_medium_info() is called.
	info->interfaces = calloc(config_setting_length2(itf),
			          sizeof(struct interface));
	info->n_interfaces = 0;
	for (unsigned int i = 0; i < config_setting_length2(itf); i++) {
		config_setting_t *e = config_setting_get_elem(itf, i);
		const char *mac = config_setting_get_string(e);
		if (mac == NULL) {
			fprintf(stderr,
				"Setting \"interfaces\" (%s:%d) array "
				"members must be strings: \" ... \"\n",
				config_setting_source_file(e),
				config_setting_source_line(e));
			return false;
		}
		// validate format
		unsigned int m[ETH_ALEN];
		if (sscanf(mac, "%x:%x:%x:%x:%x:%x", &m[0], &m[1], &m[2], &m[3],
			   &m[4], &m[5]) != ETH_ALEN) {
			fprintf(stderr, "Invalid mac address: \"%s\" (%s:%d)\n",
				mac, config_setting_source_file(e),
				config_setting_source_line(e));
			return false;
		}
		unsigned int p = info->n_interfaces;
		info->interfaces[p].index = p;
		info->interfaces[p].addr[0] = (unsigned char) m[0];
		info->interfaces[p].addr[1] = (unsigned char) m[1];
		info->interfaces[p].addr[2] = (unsigned char) m[2];
		info->interfaces[p].addr[3] = (unsigned char) m[3];
		info->interfaces[p].addr[4] = (unsigned char) m[4];
		info->interfaces[p].addr[5] = (unsigned char) m[5];

		info->interfaces[p].medium = info;

		info->n_interfaces++;
	}

	// check for mac_address repetitions in this medium
	for (unsigned int i = 0; i < info->n_interfaces; i++) {
		for (unsigned int j = i + 1; j < info->n_interfaces; j++) {
			if (compare_mac_addr(info->interfaces[i].addr,
					     info->interfaces[j].addr,
					     ETH_ALEN)) {
				fprintf(stderr, "Repeated mac address: "
					MAC_FMT "\n",
					MAC_ARGS(info->interfaces[j].addr));
				return false;
			}
		}
	}

	config_setting_t *mod = config_setting_lookup(medium, "model");
	if (config_setting_type(mod) != CONFIG_TYPE_GROUP) {
		fprintf(stderr,
			"Setting \"model\" (%s:%d) must be a group: "
			"{ ... }\n",
			config_setting_source_file(mod),
			config_setting_source_line(mod));
		return false;
	}

	return configure_model(mod, info);
}

static bool configure_model(config_setting_t *model, struct medium *info)
{
	bool set[__MODEL_SETTING_SIZE];
	memset(&set, false, sizeof(set) / sizeof(bool));
	const char * const *ms = model_sett_str;

	for (unsigned int i = 0; i < config_setting_length2(model); i++) {
		config_setting_t *e = config_setting_get_elem(model, i);
		const char *name = config_setting_name(e);
		if (strcmp(name, ms[MODEL_TYPE]) == 0)
			set[MODEL_TYPE] = true;
		else if (strcmp(name, ms[MODEL_DEFAULT_SNR]) == 0)
			set[MODEL_DEFAULT_SNR] = true;
		else if (strcmp(name, ms[MODEL_LINKS]) == 0)
			set[MODEL_LINKS] = true;
		else if (strcmp(name, ms[MODEL_DEFAULT_PROBABILITY]) == 0)
			set[MODEL_DEFAULT_PROBABILITY] = true;
		else if (strcmp(name, ms[MODEL_SIMULATE_INTERFERENCE]) == 0)
			set[MODEL_SIMULATE_INTERFERENCE] = true;
		else if (strcmp(name, ms[MODEL_NOISE_LEVEL]) == 0)
			set[MODEL_NOISE_LEVEL] = true;
		else if (strcmp(name, ms[MODEL_FADING_COEFFICIENT]) == 0)
			set[MODEL_FADING_COEFFICIENT] = true;
		else if (strcmp(name, ms[MODEL_POSITIONS]) == 0)
			set[MODEL_POSITIONS] = true;
		else if (strcmp(name, ms[MODEL_MOVE_INTERVAL]) == 0)
			set[MODEL_MOVE_INTERVAL] = true;
		else if (strcmp(name, ms[MODEL_DIRECTIONS]) == 0)
			set[MODEL_DIRECTIONS] = true;
		else if (strcmp(name, ms[MODEL_TX_POWERS]) == 0)
			set[MODEL_TX_POWERS] = true;
		else if (strcmp(name, ms[MODEL_ANTENNA_GAIN]) == 0)
			set[MODEL_ANTENNA_GAIN] = true;
		else if (strcmp(name, ms[MODEL_ISNODEAPS]) == 0)
			set[MODEL_ISNODEAPS] = true;
		else if (strcmp(name, ms[MODEL_MODEL_NAME]) == 0)
			set[MODEL_MODEL_NAME] = true;
		else if (strcmp(name, ms[MODEL_MODEL_PARAMETERS]) == 0)
			set[MODEL_MODEL_PARAMETERS] = true;
		else
			fprintf(stdout, setting_ignore_unknown, e->name,
				config_setting_source_file(e),
				config_setting_source_line(e));
	}

	/* setting model.type */

	// check presence of required setting model.type
	if (!set[MODEL_TYPE]) {
		char *p = config_setting_path(model);
		fprintf(stderr, "Setting %s.type required!\n", p);
		free(p);
		return false;
	}
	
	// check type and value
	config_setting_t *type = config_setting_get_member(model, "type");
	const char *type_val = config_setting_get_string(type);
	if (type_val == NULL) {
		fprintf(stderr, setting_must_be_string, type->name,
			config_setting_source_file(type),
			config_setting_source_line(type));
		return false;
	}
	
	if (strcmp(type_val, model_type_str[MODEL_SNR]) == 0)
		return configure_model_snr(model, info, (bool *)&set);
	else if (strcmp(type_val, model_type_str[MODEL_PROB]) == 0)
		return configure_model_prob(model, info, (bool *)&set);
	else if (strcmp(type_val, model_type_str[MODEL_PATH_LOSS]) == 0)
		return configure_model_path_loss(model, info, (bool *)&set);
	else {
		char *p = config_setting_path(type);
		fprintf(stderr, "Invalid value for setting %s: %s\n", 
			p, type_val);
		free(p);
		return false;
	}
	
	return false; // never reached
}

static char *model_ignore_message(const char *parent_path,
				      const char *model_type)
{
	char *msg_form = "Ignoring setting %s.%s not used by model \"%s\"\n";
	unsigned long msg_len = strlen(msg_form) + strlen(parent_path) +
				strlen(model_type) - 3 /* 2*"%s" - 1 ('\0') */;
	char *msg = malloc(msg_len);
	sprintf(msg, "Ignoring setting %s.%s not used by model \"%s\"\n",
		parent_path, "%s", model_type);
	return msg;
}

static bool configure_links(config_setting_t *links, void *value_matrix,
			    unsigned n_interfaces, int model_type)
{
	for (unsigned int i = 0; i < config_setting_length2(links); i++) {
		config_setting_t *el = config_setting_get_elem(links, i);
		
		if (config_setting_type(el) != CONFIG_TYPE_LIST ||
		    config_setting_length2(el) != 3) {
			fprintf(stderr, "Invalid attribute format (%s:%u)\n",
				config_setting_source_file(el),
				config_setting_source_line(el));
			return false;
		}

		config_setting_t *src_intf = config_setting_get_elem(el, 0);
		config_setting_t *dst_intf = config_setting_get_elem(el, 1);
		config_setting_t *value = config_setting_get_elem(el, 2);

		if (model_type == MODEL_SNR) {
			if (config_setting_type(src_intf) != CONFIG_TYPE_INT ||
			    config_setting_type(dst_intf) != CONFIG_TYPE_INT ||
			    config_setting_type(value) != CONFIG_TYPE_INT) {
				fprintf(stderr,
					"Invalid \"links\" member type (%s:%u)."
					" Should be (<int>,<int>,<int>)\n",
					config_setting_source_file(src_intf),
					config_setting_source_line(src_intf));
				return false;
			}
		} else if (model_type == MODEL_PROB) {
			if (config_setting_type(src_intf) != CONFIG_TYPE_INT ||
			    config_setting_type(dst_intf) != CONFIG_TYPE_INT ||
			    config_setting_type(value) != CONFIG_TYPE_FLOAT) {
				fprintf(stderr,
					"Invalid \"links\" member type (%s:%u)."
					" Should be (<int>,<int>,<float>)\n",
					config_setting_source_file(src_intf),
					config_setting_source_line(src_intf));
				return false;
			}
		} else { // WRONG model_type
			fprintf(stderr, "DEBUG: wrong parameter model_type in "
				"%s: %d\n", __func__, model_type);
			return false;
		}

		int source_interface = config_setting_get_int(src_intf);
		int destination_interface = config_setting_get_int(dst_intf);
		if (source_interface < 0 || source_interface >= n_interfaces ||
		    destination_interface < 0 ||
		    destination_interface >= n_interfaces) {
			    fprintf(stderr, "Invalid interface index (%s:%u)."
			    "Index should be >= 0 and < number of interfaces.\n",
			            config_setting_source_file(src_intf),
			            config_setting_source_line(src_intf));
			return false;
		}
		
		if (model_type == MODEL_SNR) {
			int *matrix = (int *) value_matrix;
			matrix[source_interface * n_interfaces + 
			       destination_interface] = 
					config_setting_get_int(value);
		} else if (model_type == MODEL_PROB) {
			double prob = config_setting_get_float(value);
			if (prob < 0.0 || prob > 1.0) {
				fprintf(stderr,
				    "Invalid probability value (%s:%u). "
				    "Probability should be >= 0.0 and <= 1.0.\n",
				    config_setting_source_file(value),
				    config_setting_source_line(value));
				return false;
			}
			double *matrix = (double *) value_matrix;
			matrix[source_interface * n_interfaces + 
			       destination_interface] = prob;
		}
	}
	return true;
}


static bool configure_model_snr(config_setting_t *model, 
			       struct medium *info,
			       bool *setting_present)
{
	/* check presence of required settings */
	// no required settings
	/* report unrelated settings */
	char *mp = config_setting_path(model);
	char *msg = model_ignore_message(mp, model_type_str[MODEL_SNR]);
	free(mp);

	const char * const *ms = model_sett_str;
	if (setting_present[MODEL_DEFAULT_PROBABILITY])
		fprintf(stdout, msg, ms[MODEL_DEFAULT_PROBABILITY]);
	if (setting_present[MODEL_SIMULATE_INTERFERENCE])
		fprintf(stdout, msg, ms[MODEL_SIMULATE_INTERFERENCE]);
	if (setting_present[MODEL_NOISE_LEVEL])
		fprintf(stdout, msg, ms[MODEL_NOISE_LEVEL]);
	if (setting_present[MODEL_FADING_COEFFICIENT])
		fprintf(stdout, msg, ms[MODEL_FADING_COEFFICIENT]);
	if (setting_present[MODEL_POSITIONS])
		fprintf(stdout, msg, ms[MODEL_POSITIONS]);
	if (setting_present[MODEL_MOVE_INTERVAL])
		fprintf(stdout, msg, ms[MODEL_MOVE_INTERVAL]);
	if (setting_present[MODEL_DIRECTIONS])
		fprintf(stdout, msg, ms[MODEL_DIRECTIONS]);
	if (setting_present[MODEL_TX_POWERS])
		fprintf(stdout, msg, ms[MODEL_TX_POWERS]);
	if (setting_present[MODEL_ANTENNA_GAIN])
		fprintf(stdout, msg, ms[MODEL_ANTENNA_GAIN]);
	if (setting_present[MODEL_ISNODEAPS])
		fprintf(stdout, msg, ms[MODEL_ISNODEAPS]);
	if (setting_present[MODEL_MODEL_NAME])
		fprintf(stdout, msg, ms[MODEL_MODEL_NAME]);
	if (setting_present[MODEL_MODEL_PARAMETERS])
		fprintf(stdout, msg, ms[MODEL_MODEL_PARAMETERS]);
	free((void *)msg);

	info->model_index = MN_SNR;

	/* check presence of optional settings, check value validity and
	report default values for settings not configured */
	// optional settings: default_snr, links
	int snr_default = CFG_DEFAULT_SNR;
	if (setting_present[MODEL_DEFAULT_SNR]) {
		config_setting_t *snr_d =
			config_setting_lookup(model, ms[MODEL_DEFAULT_SNR]);
		if (config_setting_type(snr_d) != CONFIG_TYPE_INT) {
			fprintf(stderr, setting_must_be_int,
				ms[MODEL_DEFAULT_SNR],
				config_setting_source_file(snr_d),
				config_setting_source_line(snr_d));
			return false;
		}
		snr_default = config_setting_get_int(snr_d);
	}

	// if a failure happens it is freed when delete_medium_info() is called.
	info->snr_matrix = malloc(sizeof(int) * 
			   info->n_interfaces * info->n_interfaces);
	for (unsigned int i = 0; i < info->n_interfaces; i++)
		for (unsigned j = 0; j < info->n_interfaces; j++)
			info->snr_matrix[i * info->n_interfaces + j] =
				snr_default;
	
	if (setting_present[MODEL_LINKS]) {
		config_setting_t *links =
			config_setting_lookup(model, ms[MODEL_LINKS]);
		if (config_setting_type(links) != CONFIG_TYPE_LIST) {
			fprintf(stderr, setting_must_be_list, links->name);
			return false;
		}
		if (!configure_links(links, (void *) info->snr_matrix,
				     info->n_interfaces, MODEL_SNR)) {
			return false;
		}
	}
	fprintf(stdout, "%s = %d used for all unconfigured pairs in %s.\n",
		ms[MODEL_DEFAULT_SNR], snr_default, ms[MODEL_LINKS]);
	
	info->get_link_snr = get_link_snr_from_snr_matrix;
	info->get_error_prob = _get_error_prob_from_snr;
	info->move_interfaces = NULL;

	return true;
}

static bool configure_model_prob(config_setting_t *model,
			        struct medium *info,
			        bool *setting_present)
{
	/* check presence of required settings */
	// no required settings
	/* report unrelated settings */
	char *mp = config_setting_path(model);
	char *msg = model_ignore_message(mp, model_type_str[MODEL_PROB]);
	free(mp);

	const char * const *ms = model_sett_str;
	if (setting_present[MODEL_DEFAULT_SNR])
		fprintf(stdout, msg, ms[MODEL_DEFAULT_SNR]);
	if (setting_present[MODEL_SIMULATE_INTERFERENCE])
		fprintf(stdout, msg, ms[MODEL_SIMULATE_INTERFERENCE]);
	if (setting_present[MODEL_NOISE_LEVEL])
		fprintf(stdout, msg, ms[MODEL_NOISE_LEVEL]);
	if (setting_present[MODEL_FADING_COEFFICIENT])
		fprintf(stdout, msg, ms[MODEL_FADING_COEFFICIENT]);
	if (setting_present[MODEL_POSITIONS])
		fprintf(stdout, msg, ms[MODEL_POSITIONS]);
	if (setting_present[MODEL_MOVE_INTERVAL])
		fprintf(stdout, msg, ms[MODEL_MOVE_INTERVAL]);
	if (setting_present[MODEL_DIRECTIONS])
		fprintf(stdout, msg, ms[MODEL_DIRECTIONS]);
	if (setting_present[MODEL_TX_POWERS])
		fprintf(stdout, msg, ms[MODEL_TX_POWERS]);
	if (setting_present[MODEL_ANTENNA_GAIN])
		fprintf(stdout, msg, ms[MODEL_ANTENNA_GAIN]);
	if (setting_present[MODEL_ISNODEAPS])
		fprintf(stdout, msg, ms[MODEL_ISNODEAPS]);
	if (setting_present[MODEL_MODEL_NAME])
		fprintf(stdout, msg, ms[MODEL_MODEL_NAME]);
	if (setting_present[MODEL_MODEL_PARAMETERS])
		fprintf(stdout, msg, ms[MODEL_MODEL_PARAMETERS]);
	free((void *)msg);

	info->model_index = MN_PROB;

	/* check presence of optional settings, check value validity and
	report default values for settings not configured */
	// optional settings: default_prob, links
	double prob_default = CFG_DEFAULT_PROB;
	if (setting_present[MODEL_DEFAULT_PROBABILITY]) {
		config_setting_t *prob_dflt =
			config_setting_lookup(model,
					      ms[MODEL_DEFAULT_PROBABILITY]);
		if (config_setting_type(prob_dflt) != CONFIG_TYPE_FLOAT) {
			fprintf(stderr, setting_must_be_float,
				ms[MODEL_DEFAULT_PROBABILITY],
				config_setting_source_file(prob_dflt),
				config_setting_source_line(prob_dflt));
			return false;
		}
		prob_default = config_setting_get_float(prob_dflt);
	}

	// if a failure happens it is freed when delete_medium_info() is called.
	info->prob_matrix = malloc(sizeof(double) * 
			   	   info->n_interfaces * info->n_interfaces);
	for (unsigned int i = 0; i < info->n_interfaces; i++)
		for (unsigned j = 0; j < info->n_interfaces; j++)
			info->prob_matrix[i * info->n_interfaces + j] =
				prob_default;
	
	if (setting_present[MODEL_LINKS]) {
		config_setting_t *links = config_setting_lookup(model, "links");
		if (config_setting_type(links) != CONFIG_TYPE_LIST) {
			fprintf(stderr, setting_must_be_list, ms[MODEL_LINKS]);
			return false;
		}
		if (!configure_links(links, (void *) info->prob_matrix,
				     info->n_interfaces, MODEL_PROB)) {
			return false;
		}
	}
	fprintf(stdout, "%s = %lf used for all unconfigured pairs in %s.\n",
		ms[MODEL_DEFAULT_PROBABILITY], prob_default, ms[MODEL_LINKS]);
	
	info->get_link_snr = get_link_snr_default;
	info->get_error_prob = get_error_prob_from_matrix;
	info->move_interfaces = NULL;

	return true;
}

static bool configure_model_path_loss(config_setting_t *model,
			             struct medium *info,
			       	     bool *setting_present)
{
	const char * const *ms = model_sett_str;
	
	/* check presence of required settings */
	// required: positions, tx_powers, model_name, model_params
	bool fail = false;
	const char *error_required = "Error: setting \"%s\" is required.\n";
	if (!setting_present[MODEL_POSITIONS]) {
		fprintf(stderr, error_required, ms[MODEL_POSITIONS]);
		fail = true;
	}
	if (!setting_present[MODEL_TX_POWERS]) {
		fprintf(stderr, error_required, ms[MODEL_TX_POWERS]);
		fail = true;
	}
	if (!setting_present[MODEL_MODEL_NAME]) {
		fprintf(stderr, error_required, ms[MODEL_MODEL_NAME]);
		fail = true;
	}
	if (!setting_present[MODEL_MODEL_PARAMETERS]) {
		fprintf(stderr, error_required, ms[MODEL_MODEL_PARAMETERS]);
		fail = true;
	}
	if (fail)
		return false;
	
	/* report unrelated settings */
	char *mp = config_setting_path(model);
	char *msg = model_ignore_message(mp, model_type_str[MODEL_PATH_LOSS]);
	free(mp);
	if (setting_present[MODEL_DEFAULT_SNR])
		fprintf(stdout, msg, ms[MODEL_DEFAULT_SNR]);
	if (setting_present[MODEL_DEFAULT_PROBABILITY])
		fprintf(stdout, msg, ms[MODEL_DEFAULT_PROBABILITY]);
	if (setting_present[MODEL_LINKS])
		fprintf(stdout, msg, ms[MODEL_LINKS]);
	free((void *)msg);
	
	info->move_interfaces = NULL;

	if (setting_present[MODEL_SIMULATE_INTERFERENCE]) {
		int val;
		if (config_setting_lookup_bool(model,
		    ms[MODEL_SIMULATE_INTERFERENCE], &val) == CONFIG_FALSE) {
			config_setting_t *s = config_setting_lookup(model,
				ms[MODEL_SIMULATE_INTERFERENCE]);
			fprintf(stderr,
				setting_must_be_bool,
				ms[MODEL_SIMULATE_INTERFERENCE],
				config_setting_source_file(s),
				config_setting_source_line(s));
			return false;
		} else {
			info->sim_interference = val == CONFIG_TRUE;
		}
	} else {
		info->sim_interference = CFG_DEFAULT_SIMULATE_INTERFERENCE;
		if (CFG_DEFAULT_SIMULATE_INTERFERENCE) {
			fprintf(stdout, "Interference enabled (default).");
		} else {
			fprintf(stdout, "Interference disabled (default).");
		}
	}

	if (setting_present[MODEL_NOISE_LEVEL]) {
		if (config_setting_lookup_int(model, ms[MODEL_NOISE_LEVEL], 
		    &info->noise_level) == CONFIG_FALSE) {
			config_setting_t *s = config_setting_lookup(model,
				ms[MODEL_NOISE_LEVEL]);
			fprintf(stderr,
				setting_must_be_int,
				ms[MODEL_NOISE_LEVEL],
				config_setting_source_file(s),
				config_setting_source_line(s));
			return false;
		}
	} else {
		info->noise_level = CFG_DEFAULT_NOISE_LEVEL;
		fprintf(stdout, "Using %s = %d (default).\n",
			ms[MODEL_NOISE_LEVEL], info->noise_level);
	}

	if (setting_present[MODEL_FADING_COEFFICIENT]) {
		config_setting_t *s = config_setting_lookup(model,
				ms[MODEL_FADING_COEFFICIENT]);
		if (config_setting_lookup_int(model,
		    ms[MODEL_FADING_COEFFICIENT], &info->fading_coefficient) ==
		    CONFIG_FALSE) {
			fprintf(stderr,
				setting_must_be_int,
				ms[MODEL_FADING_COEFFICIENT],
				config_setting_source_file(s),
				config_setting_source_line(s));
			return false;
		} else if (info->fading_coefficient < 1) {
			fprintf(stderr,
				"Setting \"%s\" (%s:%d) must be >= 1.\n",
				ms[MODEL_FADING_COEFFICIENT],
				config_setting_source_file(s),
				config_setting_source_line(s));
			return false;
		}
	} else {
		info->fading_coefficient = CFG_DEFAULT_FADING_COEFFICIENT;
		fprintf(stdout, "Using %s = %d (default).\n",
			ms[MODEL_FADING_COEFFICIENT], info->fading_coefficient);
	}

	if (setting_present[MODEL_MOVE_INTERVAL]) {
		config_setting_t *s = config_setting_lookup(model,
				ms[MODEL_MOVE_INTERVAL]);
		if (config_setting_lookup_float(model,
		    ms[MODEL_MOVE_INTERVAL], &info->move_interval) ==
		    CONFIG_FALSE) {
			fprintf(stderr,
				setting_must_be_float,
				ms[MODEL_MOVE_INTERVAL],
				config_setting_source_file(s),
				config_setting_source_line(s));
			return false;
		} else if (info->move_interval <= 0.0) {
			fprintf(stderr,
				"Setting \"%s\" (%s:%d) must be > 0.0.\n",
				ms[MODEL_MOVE_INTERVAL],
				config_setting_source_file(s),
				config_setting_source_line(s));
			return false;
		}
	} else {
		info->move_interval = CFG_DEFAULT_MOVE_INTERVAL;
		fprintf(stdout, "Using %s = %lf (default).\n",
			ms[MODEL_MOVE_INTERVAL], info->move_interval);
	}

	config_setting_t *positions =
		config_setting_lookup(model, ms[MODEL_POSITIONS]);
	if (config_setting_type(positions) != CONFIG_TYPE_LIST) {
		fprintf(stderr, setting_must_be_list,
			positions->name, config_setting_source_file(positions),
			config_setting_source_line(positions));
		return false;
	}
	if (!configure_positions_directions(positions, info,
	    CONFIGURE_POSITIONS))
		return false;

	if (setting_present[MODEL_DIRECTIONS]) {
		config_setting_t *directions =
			config_setting_lookup(model, ms[MODEL_DIRECTIONS]);
		if (config_setting_type(directions) != CONFIG_TYPE_LIST) {
			fprintf(stderr, setting_must_be_list,
				directions->name,
				config_setting_source_file(directions),
				config_setting_source_line(directions));
			return false;
		}
		if (!configure_positions_directions(directions, info,
		    CONFIGURE_DIRECTIONS))
			return false;
		info->move_interfaces = move_interfaces;
	} else {
		for (unsigned int i = 0; i < info->n_interfaces; i++) {
			info->interfaces[i].direction_x = 0;
			info->interfaces[i].direction_y = 0;
			info->interfaces[i].direction_z = 0;
		}
	}

	config_setting_t *tx_powers =
		config_setting_lookup(model, ms[MODEL_TX_POWERS]);
	if (config_setting_type(tx_powers) != CONFIG_TYPE_ARRAY) {
		fprintf(stderr,
			setting_must_be_array,
			tx_powers->name, config_setting_source_file(tx_powers),
			config_setting_source_line(tx_powers));
		return false;
	}
	if (config_setting_length2(tx_powers) != info->n_interfaces) {
		fprintf(stderr,
		"Setting %s (%s:%u) must have an entry for each interface.\n",
			tx_powers->name, config_setting_source_file(tx_powers),
			config_setting_source_line(tx_powers));
		return false;
	}
	for (unsigned int i = 0; i < info->n_interfaces; i++) {
		config_setting_t *e = config_setting_get_elem(tx_powers, i);
		if (config_setting_type(e) != CONFIG_TYPE_INT) {
			fprintf(stderr,
			"Setting %s (%s:%u) array members must be integers.\n",
				tx_powers->name,
				config_setting_source_file(tx_powers),
				config_setting_source_line(tx_powers));
			return false;
		}
		info->interfaces[i].tx_power = config_setting_get_int(e);
	}

	if (setting_present[MODEL_ANTENNA_GAIN]) {
		config_setting_t *antenna_g =
			config_setting_lookup(model, ms[MODEL_ANTENNA_GAIN]);
		if (config_setting_type(antenna_g) != CONFIG_TYPE_ARRAY) {
			fprintf(stderr,
				setting_must_be_array,
				antenna_g->name,
				config_setting_source_file(antenna_g),
				config_setting_source_line(antenna_g));
			return false;
		}
		if (config_setting_length2(antenna_g) != info->n_interfaces) {
			fprintf(stderr,
			"Setting %s (%s:%u) must have an entry for each interface.\n",
				antenna_g->name,
				config_setting_source_file(antenna_g),
				config_setting_source_line(antenna_g));
			return false;
		}
		for (unsigned int i = 0; i < info->n_interfaces; i++) {
			config_setting_t *e =
				config_setting_get_elem(antenna_g, i);
			if (config_setting_type(e) != CONFIG_TYPE_INT) {
				fprintf(stderr,
				"Setting %s (%s:%u) array members must be integers.\n",
					antenna_g->name,
					config_setting_source_file(antenna_g),
					config_setting_source_line(antenna_g));
				return false;
			}
			info->interfaces[i].antenna_gain = 
				config_setting_get_int(e);
		}
	} else {
		for (unsigned int i = 0; i < info->n_interfaces; i++)
			info->interfaces[i].antenna_gain =
				CFG_DEFAULT_ANTENNA_GAIN;

	}

	if (setting_present[MODEL_ISNODEAPS]) {
		config_setting_t *nodeaps =
			config_setting_lookup(model, ms[MODEL_ISNODEAPS]);
		if (config_setting_type(nodeaps) != CONFIG_TYPE_ARRAY) {
			fprintf(stderr,
				setting_must_be_array,
				nodeaps->name,
				config_setting_source_file(nodeaps),
				config_setting_source_line(nodeaps));
			return false;
		}
		if (config_setting_length2(nodeaps) != info->n_interfaces) {
			fprintf(stderr,
			"Setting %s (%s:%u) must have an entry for each interface.\n",
				nodeaps->name,
				config_setting_source_file(nodeaps),
				config_setting_source_line(nodeaps));
			return false;
		}
		for (unsigned int i = 0; i < info->n_interfaces; i++) {
			config_setting_t *e =
				config_setting_get_elem(nodeaps, i);
			if (config_setting_type(e) != CONFIG_TYPE_BOOL) {
				fprintf(stderr,
				"Setting %s (%s:%u) array members must be booleans.\n",
					nodeaps->name,
					config_setting_source_file(nodeaps),
					config_setting_source_line(nodeaps));
				return false;
			}
			info->interfaces[i].isap = 
				config_setting_get_bool(e) == CONFIG_TRUE;
		}
	} else {
		for (unsigned int i = 0; i < info->n_interfaces; i++)
			info->interfaces[i].isap = CFG_DEFAULT_ISNODEAPS;
	}
	
	config_setting_t *model_name =
		config_setting_lookup(model, ms[MODEL_MODEL_NAME]);
	if (config_setting_type(model_name) != CONFIG_TYPE_STRING) {
		fprintf(stderr, setting_must_be_string,
			model_name->name,
			config_setting_source_file(model_name),
			config_setting_source_line(model_name));
		return false;
	}
	config_setting_t *model_params =
		config_setting_lookup(model, ms[MODEL_MODEL_PARAMETERS]);
	if (config_setting_type(model_params) != CONFIG_TYPE_GROUP) {
		fprintf(stderr, setting_must_be_group,
			model_params->name,
			config_setting_source_file(model_params),
			config_setting_source_line(model_params));
		return false;
	}

	if (!configure_model_type(model_name, model_params, info)) {
		return false;
	}

	info->get_link_snr = get_link_snr_from_snr_matrix;
	info->get_error_prob = _get_error_prob_from_snr;

	info->snr_matrix =
		calloc(info->n_interfaces * info->n_interfaces, sizeof(int));
	recalc_path_loss(info);

	return true;
}

static bool configure_model_type(config_setting_t *name,
				 config_setting_t *params,
				 struct medium *info)
{
	const char* setting_is_required_str =
		"Setting \"%s\" is required by model \"%s\".\n";
	
	const char *model_name = config_setting_get_string(name);

	enum { MP_SYSLOSS, MP_PLEXPON, MP_XG, MP_NFLOOR, MP_FLOORPEN,
	       MP_POWLOSS, __MP_MAX };
	bool setting_present[__MP_MAX];
	for (unsigned int i = 0; i < __MP_MAX; i++) {
		setting_present[i] = false;
	}

	const char * const mp_str[] = { "system_loss", "path_loss_exponent",
		"xg", "n_floors", "floor_pen_factor", "power_loss_coefficient" };

	for (unsigned int i = 0; i < config_setting_length2(params); i++) {
		config_setting_t *e = config_setting_get_elem(params, i);
		const char *n = config_setting_name(e);
		if (strcmp(n, mp_str[MP_SYSLOSS]) == 0)
			setting_present[MP_SYSLOSS] = true;
		else if (strcmp(n, mp_str[MP_PLEXPON]) == 0)
			setting_present[MP_PLEXPON] = true;
		else if (strcmp(n, mp_str[MP_XG]) == 0)
			setting_present[MP_XG] = true;
		else if (strcmp(n, mp_str[MP_NFLOOR]) == 0)
			setting_present[MP_NFLOOR] = true;
		else if (strcmp(n, mp_str[MP_FLOORPEN]) == 0)
			setting_present[MP_FLOORPEN] = true;
		else if (strcmp(n, mp_str[MP_POWLOSS]) == 0)
			setting_present[MP_POWLOSS] = true;
		else {
			fprintf(stdout, setting_ignore_unknown, e->name,
				config_setting_source_file(e),
				config_setting_source_line(e));
		}
	}
	// system_loss (int) - free_space, log_norm_shadow, two_ray_ground
	// path_loss_exponent (float) - log_distance, log_normal_shadowing
	// xg (float) - log_distance
	// n_floors (int) - itu
	// floor_pen_factor (int) - itu
	// power_loss_coefficient (int) - itu (? = path_loss_exponent)?

	char *parent_path = config_setting_path(params);

	if (strcmp(model_name_str[MN_FREE_SPACE], model_name) == 0) {
		// required settings: system_loss
		if (!setting_present[MP_SYSLOSS]) {
			fprintf(stderr,
				setting_is_required_str, mp_str[MP_SYSLOSS],
				model_name_str[MN_FREE_SPACE]);
			return false;
		}
		// warn about settings not relevant
		char *ignore = model_ignore_message(parent_path,
			model_name_str[MN_FREE_SPACE]);
		if (setting_present[MP_PLEXPON])
			fprintf(stdout, ignore, mp_str[MP_PLEXPON]);
		if (setting_present[MP_XG])
			fprintf(stdout, ignore, mp_str[MP_XG]);
		if (setting_present[MP_NFLOOR])
			fprintf(stdout, ignore, mp_str[MP_NFLOOR]);
		if (setting_present[MP_FLOORPEN])
			fprintf(stdout, ignore, mp_str[MP_FLOORPEN]);
		if (setting_present[MP_POWLOSS])
			fprintf(stdout, ignore, mp_str[MP_POWLOSS]);
		free(ignore);

		info->model_index = MN_FREE_SPACE;
		config_setting_t *sys_loss =
			config_setting_lookup(params, mp_str[MP_SYSLOSS]);
		if (config_setting_type(sys_loss) != CONFIG_TYPE_INT) {
			fprintf(stderr,
				setting_must_be_int,
				sys_loss->name,
				config_setting_source_file(sys_loss),
				config_setting_source_line(sys_loss));
			return false;
		}
		// TODO add range check to info->system_loss
		info->system_loss = config_setting_get_int(sys_loss);

		info->path_loss_func = calc_path_loss_free_space;
		
	} else if (strcmp(model_name_str[MN_ITU], model_name) == 0) {
		// required settings: n_floors, floor_pen_factor, power_loss_coef
		if (!setting_present[MP_NFLOOR]) {
			fprintf(stderr, setting_is_required_str,
				mp_str[MP_NFLOOR],
				model_name_str[MN_ITU]);
			return false;
		}
		if (!setting_present[MP_FLOORPEN]) {
			fprintf(stderr, setting_is_required_str,
				mp_str[MP_FLOORPEN],
				model_name_str[MN_ITU]);
			return false;
		}
		if (!setting_present[MP_POWLOSS]) {
			fprintf(stderr, setting_is_required_str,
				mp_str[MP_POWLOSS],
				model_name_str[MN_ITU]);
			return false;
		}
		// warn about settings not relevant
		char *ignore = model_ignore_message(parent_path,
			model_name_str[MN_ITU]);
		if (setting_present[MP_PLEXPON])
			fprintf(stdout, ignore, mp_str[MP_PLEXPON]);
		if (setting_present[MP_XG])
			fprintf(stdout, ignore, mp_str[MP_XG]);
		if (setting_present[MP_SYSLOSS])
			fprintf(stdout, ignore, mp_str[MP_SYSLOSS]);
		free(ignore);

		info->model_index = MN_ITU;
		config_setting_t *nfloor =
			config_setting_lookup(params, mp_str[MP_NFLOOR]);
		if (config_setting_type(nfloor) != CONFIG_TYPE_INT) {
			fprintf(stderr, setting_must_be_int, nfloor->name,
				config_setting_source_file(nfloor),
				config_setting_source_line(nfloor));
			return false;
		}
		config_setting_t *floorpen =
			config_setting_lookup(params, mp_str[MP_FLOORPEN]);
		if (config_setting_type(floorpen) != CONFIG_TYPE_INT) {
			fprintf(stderr, setting_must_be_int, floorpen->name,
				config_setting_source_file(floorpen),
				config_setting_source_line(floorpen));
			return false;
		}
		config_setting_t *powerlosscoef =
			config_setting_lookup(params, mp_str[MP_POWLOSS]);
		if (config_setting_type(powerlosscoef) != CONFIG_TYPE_INT) {
			fprintf(stderr, setting_must_be_int,
				powerlosscoef->name,
				config_setting_source_file(powerlosscoef),
				config_setting_source_line(powerlosscoef));
			return false;
		}
		// TODO add range checks
		info->n_floors = (unsigned int) config_setting_get_int(nfloor);
		info->floor_pen_factor = config_setting_get_int(floorpen);
		info->power_loss_coeff = config_setting_get_int(powerlosscoef);

		info->path_loss_func = calc_path_loss_itu;
		
	} else if (strcmp(model_name_str[MN_TWO_RAY_GROUND], model_name) == 0) {
		// required settings: system_loss
		if (!setting_present[MP_SYSLOSS]) {
			fprintf(stderr, setting_is_required_str,
				mp_str[MP_SYSLOSS],
				model_name_str[MN_TWO_RAY_GROUND]);
			return false;
		}
		// warn about settings not relevant
		char *ignore = model_ignore_message(parent_path,
			model_name_str[MN_TWO_RAY_GROUND]);
		if (setting_present[MP_PLEXPON])
			fprintf(stdout, ignore, mp_str[MP_PLEXPON]);
		if (setting_present[MP_XG])
			fprintf(stdout, ignore, mp_str[MP_XG]);
		if (setting_present[MP_NFLOOR])
			fprintf(stdout, ignore, mp_str[MP_NFLOOR]);
		if (setting_present[MP_FLOORPEN])
			fprintf(stdout, ignore, mp_str[MP_FLOORPEN]);
		if (setting_present[MP_POWLOSS])
			fprintf(stdout, ignore, mp_str[MP_POWLOSS]);
		free(ignore);

		info->model_index = MN_TWO_RAY_GROUND;
		config_setting_t *sys_loss =
			config_setting_lookup(params, mp_str[MP_SYSLOSS]);
		if (config_setting_type(sys_loss) != CONFIG_TYPE_INT) {
			fprintf(stderr, setting_must_be_int, sys_loss->name,
				config_setting_source_file(sys_loss),
				config_setting_source_line(sys_loss));
			return false;
		}
		// TODO add range check to info->system_loss
		info->system_loss = config_setting_get_int(sys_loss);

		info->path_loss_func = calc_path_loss_two_ray_ground;

	} else if (strcmp(model_name_str[MN_LOG_DISTANCE], model_name) == 0) {
		// required settings: path_loss_exponent, xg
		if (!setting_present[MP_PLEXPON]) {
			fprintf(stderr, setting_is_required_str,
				mp_str[MP_PLEXPON],
				model_name_str[MN_LOG_DISTANCE]);
			return false;
		}
		if (!setting_present[MP_XG]) {
			fprintf(stderr, setting_is_required_str,
				mp_str[MP_XG],
				model_name_str[MN_LOG_DISTANCE]);
			return false;
		}
		// warn about settings not relevant
		char *ignore = model_ignore_message(parent_path,
			model_name_str[MN_LOG_DISTANCE]);
		if (setting_present[MP_SYSLOSS])
			fprintf(stdout, ignore, mp_str[MP_SYSLOSS]);
		if (setting_present[MP_NFLOOR])
			fprintf(stdout, ignore, mp_str[MP_NFLOOR]);
		if (setting_present[MP_FLOORPEN])
			fprintf(stdout, ignore, mp_str[MP_FLOORPEN]);
		if (setting_present[MP_POWLOSS])
			fprintf(stdout, ignore, mp_str[MP_POWLOSS]);
		free(ignore);

		info->model_index = MN_LOG_DISTANCE;
		config_setting_t *plexpon =
			config_setting_lookup(params, mp_str[MP_PLEXPON]);
		if (config_setting_type(plexpon) != CONFIG_TYPE_FLOAT) {
			fprintf(stderr, setting_must_be_float, plexpon->name,
				config_setting_source_file(plexpon),
				config_setting_source_line(plexpon));
			return false;
		}
		config_setting_t *xg =
			config_setting_lookup(params, mp_str[MP_XG]);
		if (config_setting_type(xg) != CONFIG_TYPE_FLOAT) {
			fprintf(stderr, setting_must_be_float, xg->name,
				config_setting_source_file(xg),
				config_setting_source_line(xg));
			return false;
		}
		// TODO add range check
		info->path_loss_exponent = config_setting_get_float(plexpon);
		info->xg = config_setting_get_float(xg);

		info->path_loss_func = calc_path_loss_log_distance;

	} else if (strcmp(model_name_str[MN_LOG_NORMAL_SHADOWING],
		          model_name) == 0) {
		// required settings: path_loss_exponent, system_loss
		if (!setting_present[MP_PLEXPON]) {
			fprintf(stderr, setting_is_required_str,
				mp_str[MP_PLEXPON],
				model_name_str[MN_LOG_NORMAL_SHADOWING]);
			return false;
		}
		if (!setting_present[MP_SYSLOSS]) {
			fprintf(stderr, setting_is_required_str,
				mp_str[MP_SYSLOSS],
				model_name_str[MN_LOG_NORMAL_SHADOWING]);
			return false;
		}
		// warn about settings not relevant
		char *ignore = model_ignore_message(parent_path,
			model_name_str[MN_LOG_NORMAL_SHADOWING]);
		if (setting_present[MP_SYSLOSS])
			fprintf(stdout, ignore, mp_str[MP_SYSLOSS]);
		if (setting_present[MP_NFLOOR])
			fprintf(stdout, ignore, mp_str[MP_NFLOOR]);
		if (setting_present[MP_FLOORPEN])
			fprintf(stdout, ignore, mp_str[MP_FLOORPEN]);
		if (setting_present[MP_POWLOSS])
			fprintf(stdout, ignore, mp_str[MP_POWLOSS]);
		free(ignore);

		info->model_index = MN_LOG_NORMAL_SHADOWING;
		config_setting_t *plexpon =
			config_setting_lookup(params, mp_str[MP_PLEXPON]);
		if (config_setting_type(plexpon) != CONFIG_TYPE_FLOAT) {
			fprintf(stderr, setting_must_be_float, plexpon->name,
				config_setting_source_file(plexpon),
				config_setting_source_line(plexpon));
			return false;
		}
		config_setting_t *sys_loss =
			config_setting_lookup(params, mp_str[MP_SYSLOSS]);
		if (config_setting_type(sys_loss) != CONFIG_TYPE_INT) {
			fprintf(stderr, setting_must_be_int, sys_loss->name,
				config_setting_source_file(sys_loss),
				config_setting_source_line(sys_loss));
			return false;
		}
		// TODO add range check
		info->path_loss_exponent = config_setting_get_float(plexpon);
		info->system_loss = config_setting_get_int(sys_loss);

		info->path_loss_func = calc_path_loss_log_normal_shadowing;

	} else {
		fprintf(stderr, "Unknown value of %s = %s (%s:%u).\n",
			name->name, model_name,
			config_setting_source_file(name),
			config_setting_source_line(name));
		return false;
	}
	free(parent_path);
	return true;
}

static bool configure_positions_directions(config_setting_t *list,
					   struct medium *info,
					   int pos_dir)
{
	if (config_setting_length2(list) != info->n_interfaces) {
		char *path = config_setting_path(list);
		fprintf(stderr, "%s must have an entry for each interface.\n",
			path);
		free(path);
		return false;
	}
	if ((pos_dir != CONFIGURE_POSITIONS) &&
	    (pos_dir != CONFIGURE_DIRECTIONS)) {
		fprintf(stderr, "DEBUG: Invalid argument pos_dir = %d\n",
			pos_dir);
		return false;
	}

	for (unsigned int i = 0; i < config_setting_length2(list); i++) {
		config_setting_t *el = config_setting_get_elem(list, i);
		
		if (config_setting_type(el) != CONFIG_TYPE_LIST ||
		    config_setting_length2(el) != 3) {
			fprintf(stderr, "Invalid attribute format (%s:%u).\n",
				config_setting_source_file(el),
				config_setting_source_line(el));
			return false;
		}

		config_setting_t *conf_x = config_setting_get_elem(el, 0);
		config_setting_t *conf_y = config_setting_get_elem(el, 1);
		config_setting_t *conf_z = config_setting_get_elem(el, 2);

		if (config_setting_type(conf_x) != CONFIG_TYPE_FLOAT ||
		    config_setting_type(conf_y) != CONFIG_TYPE_FLOAT ||
		    config_setting_type(conf_z) != CONFIG_TYPE_FLOAT) {
			fprintf(stderr, "Invalid member format at (%s:%u). "
				"Should be (<float>, <float>, <float>).\n",
				config_setting_source_file(conf_x),
				config_setting_source_line(conf_x));
			return false;
		}
		double x = config_setting_get_float(conf_x);
		double y = config_setting_get_float(conf_y);
		double z = config_setting_get_float(conf_z);

		if (pos_dir == CONFIGURE_POSITIONS) {
			info->interfaces[i].position_x = x;
			info->interfaces[i].position_y = y;
			info->interfaces[i].position_z = z;
		} else if (pos_dir == CONFIGURE_DIRECTIONS) {
			info->interfaces[i].direction_x = x;
			info->interfaces[i].direction_y = y;
			info->interfaces[i].direction_z = z;
		}
	}
	return true;
}


/******************************************************************************/
/* Helpers */

/**
 * @brief Provides full path from configuration root to parameter setting.
 * Includes the setting name in the path.
 * set1.set2.set3.<...>.<param setting name>
 * set1.set2..set3 - .. means there is a unnamed setting between the dots
 * set1.set2.<unnamed setting>.set3
 * 
 * @param setting to return path from configuration root
 * @return char* with the path - must be freed
 */
static char *config_setting_path(config_setting_t *setting)
{
	if (setting == NULL)
		return NULL;
	config_setting_t *temp = setting;
	unsigned long str_len = 0;
	while (config_setting_is_root(temp) == CONFIG_FALSE) {
		// +1 for separator '.' or '\0' at the end of the string
		str_len++;
		if (config_setting_name(temp) != NULL) {
			str_len += strlen(config_setting_name(temp));
		} else {
			str_len += 1 + 1 - 1 + /* +[ +] -. */
				number_char_length(config_setting_index(temp));
		}
		temp = config_setting_parent(temp);
	}

	temp = setting;
	char *str = malloc(sizeof(char) * str_len);
	str[str_len - 1] = '\0';
	unsigned long start = str_len - 1;
	do {
		if (config_setting_name(temp) != NULL) {
			start = start - strlen(config_setting_name(temp));
			strncpy(str + start, config_setting_name(temp),
				strlen(config_setting_name(temp)));
			if (start > 0) {
				start--;
				str[start] = '.';
			}
		} else {
			unsigned int l = 1 + 1 + /* +[ +]*/
				number_char_length(config_setting_index(temp));
			char *p = malloc(sizeof(char) * (l + 1));
			sprintf(p,"[%u]", config_setting_index(temp));
			start = start - l;
			strncpy(str + start, p, l);
			free(p);
		}
		
		temp = config_setting_parent(temp); // NULL if it was the root
	} while (config_setting_is_root(temp) == CONFIG_FALSE);

	return str;
}

/**
 * @brief Calculates the number of characters a representation in base 10 of a
 * number takes.
 * 
 * @param n 
 * @return unsigned int 
 */
static unsigned int number_char_length(unsigned int n)
{
	unsigned l = 1;
	while (n > 10) {
		l++;
		n = n / 10;
	}
	return l;
}

/**
 * @brief Same as config_setting_length but with unsigned int return instead of
 * int
 * 
 * @param setting 
 * @return unsigned int 
 */
static unsigned int config_setting_length2(config_setting_t *setting)
{
	return (unsigned int) config_setting_length(setting);
}


/**
 * @brief Prints a matrix of rows x cols row by row to stdout
 * 
 * @param matrix 
 * @param rows 
 * @param cols 
 */
static void print_matrix_int(int *matrix, unsigned int rows, unsigned int cols)
{
	for (unsigned int i = 0; i < rows; i++) {
		for (unsigned int j = 0; j < cols; j++)
			printf("%d ", matrix[i * cols + j]);
		printf("\n");
	}
}

/**
 * @brief Prints a matrix of rows x cols row by row to stdout
 * 
 * @param matrix 
 * @param rows 
 * @param cols 
 */
static void print_matrix_double(double *matrix, unsigned int rows,
				unsigned int cols)
{
	for (unsigned int i = 0; i < rows; i++) {
		for (unsigned int j = 0; j < cols; j++)
			printf("%f ", matrix[i * cols + j]);
		printf("\n");
	}
}

/**
 * @brief Prints a dump of the configuration of the parameter to the stdout. 
 * 
 * @param info 
 */
void dump_medium_info(struct medium* info)
{
	printf("id: %d\n", info->id);
	printf("n_interfaces: %u\n", info->n_interfaces);
	for (unsigned i = 0; i < info->n_interfaces; i++) {
		struct interface *c = &(info->interfaces[i]);
		printf("Interface %u: %02x:%02x:%02x:%02x:%02x:%02x\n"
			"position: (%lf,%lf,%lf), direction: (%lf,%lf,%lf)\n"
			"tx power: %d, antenna gain: %d\n",
			i, c->addr[0], c->addr[1], c->addr[2], 
			c->addr[3], c->addr[4], c->addr[5],
			c->position_x, c->position_y, c->position_z,
			c->direction_x, c->direction_y, c->direction_z,
			c->tx_power, c->antenna_gain);
	}
	if (info->snr_matrix != NULL) {
		printf("snr_matrix:\n");
		print_matrix_int(info->snr_matrix, info->n_interfaces,
				 info->n_interfaces);
	}
	if (info->prob_matrix != NULL) {
		printf("prob_matrix:\n");
		print_matrix_double(info->prob_matrix, info->n_interfaces,
				    info->n_interfaces);
	}
	printf("move_interval = %f\n", info->move_interval);
	printf("fading_coefficient = %d\n", info->fading_coefficient);
	printf("noise_level = %d\n", info->noise_level);
	printf("model_name = %s\n", model_name_str[info->model_index]);
	// printf("calc_path_loss = %lu\n", info->path_loss_func);
	// printf("get_prob_func = %lu\n", info->get_error_prob);
	// printf("get_snr_func = %lu\n", info->get_link_snr);
}

/**
 * @brief Allocates a struct medium_info, initializes the list_head and all 
 * the remaining values to 0
 * 
 * @return struct medium_info* 
 */
static struct medium* new_medium_info() 
{
	struct medium *info = calloc(1, sizeof(struct medium));
	INIT_LIST_HEAD(&(info->list));
	medium_init_qos_queues(info);
	return info;
}

/**
 * @brief Frees the argument along with all struct medium_info members
 * dynamically allocated.
 * 
 * @param mi 
 */
static void delete_medium_info(struct medium *mi)
{
	if (mi != NULL) {
		if (mi->interfaces != NULL)
			free(mi->interfaces);
		if (mi->snr_matrix != NULL)
			free(mi->snr_matrix);
		if (mi->prob_matrix != NULL)
			free(mi->prob_matrix);
		free(mi);
	}
	return;
}

/**
 * @brief Removes and frees all struct medium_info in the list. Does not free
 * the struct mediums.
 * 
 * @param mediums container of the list head of the struct medium_info 
 */
void delete_mediums(struct yawmd *mediums)
{
	struct medium *pos, *temp;
	list_for_each_entry_safe(pos, temp, &(mediums->medium_list), list) {
		list_del(&(pos->list));
		delete_medium_info(pos);
	}
}

static void wqueue_init(struct wqueue *wqueue, int cw_min, int cw_max)
{
	INIT_LIST_HEAD(&wqueue->frames);
	wqueue->cw_min = cw_min;
	wqueue->cw_max = cw_max;
}

/**
 * Initializes the 4 QoS queues
 */
static void medium_init_qos_queues(struct medium *medium)
{
	wqueue_init(&medium->qos_queues[IEEE80211_AC_BK], 15, 1023);
	wqueue_init(&medium->qos_queues[IEEE80211_AC_BE], 15, 1023);
	wqueue_init(&medium->qos_queues[IEEE80211_AC_VI], 7, 15);
	wqueue_init(&medium->qos_queues[IEEE80211_AC_VO], 3, 7);
}


/******************************************************************************/

static double pseudo_normal_distribution(void)
{
	int i;
	double normal = -6.0;

	for (i = 0; i < 12; i++)
		normal += drand48();

	return normal;
}

/**
 * @brief Calculates random fading component.
 * 
 * @param medium 
 * @return Signal fading power [dBm]
 * 
 * Takes into account ctx->fading_coefficient
 */
int get_fading_signal(struct medium *medium)
{
	if (medium->fading_coefficient != 0)
		return (int) ((double) medium->fading_coefficient * 
				       pseudo_normal_distribution());
	return 0;
}


/**
 * @brief Get the link SNR.
 * 
 * @param medium 
 * @param sender 
 * @param receiver 
 * @return DEFAULT_SNR
 */
static int get_link_snr_default(struct medium *medium, struct interface *sender,
				struct interface *receiver)
{
	return DEFAULT_SNR;
}

/**
 * @brief Get the link SNR from ctx->snr_matrix
 * 
 * @param medium 
 * @param sender 
 * @param receiver 
 * @return SNR for the pair (sender, receiver)
 */
static int get_link_snr_from_snr_matrix(struct medium *medium,
					struct interface *sender,
					struct interface *receiver)
{
	int val = medium->snr_matrix[
		        sender->index * medium->n_interfaces + receiver->index];
	return val;
}

/**
 * @brief Calculates error probability depending on the SNR value and the
 * signal modulation used for the transmission.
 * 
 * @param medium 
 * @param snr 
 * @param rate_idx 
 * @param freq 
 * @param frame_len 
 * @param src 
 * @param dst 
 * @return double 
 */
static double _get_error_prob_from_snr(struct medium *medium, double snr,
				       unsigned int rate_idx, u32 freq,
				       int frame_len, struct interface *src,
				       struct interface *dst)
{
	return get_error_prob_from_snr(snr, rate_idx, freq, frame_len);
}

/**
 * @brief Get the error probability from the medium->prob_matrix.
 * 
 * @param medium 
 * @param snr 
 * @param rate_idx 
 * @param freq 
 * @param frame_len 
 * @param src 
 * @param dst 
 * @return double
 */
static double get_error_prob_from_matrix(struct medium *medium, double snr,
					 unsigned int rate_idx, u32 freq,
					 int frame_len, struct interface *src,
					 struct interface *dst)
{
	if (dst == NULL) // dst is multicast. returned value will not be used.
		return 0.0;

	double val = medium->prob_matrix[
			        medium->n_interfaces * src->index + dst->index];
	return val;
}


// ----------------------------------------------------------------------------


/**
 * @brief Calculate path loss based on a free-space path loss model.
 * 
 * @param medium 
 * @param dst 
 * @param src 
 * @return path loss in dBm
 */
static int calc_path_loss_free_space(struct medium *medium,
				     struct interface *src,
				     struct interface *dst)
{
	double PL, d, denominator, numerator, lambda;
	double f = src->frequency * pow(10, 6);

	if (f < 0.1)
		f = FREQ_CH1;


	d = sqrt((src->position_x - dst->position_x) * 
		      	(src->position_x - dst->position_x) +
		 (src->position_y - dst->position_y) *
		      	(src->position_y - dst->position_y) +
		 (src->position_z - dst->position_z) *
		      	(src->position_z - dst->position_z));

	/*
	 * Calculate PL0 with Free-space path loss in decibels
	 *
	 * 20 * log10 * (4 * M_PI * d * f / c)
	 *   d: distance [meter]
	 *   f: frequency [Hz]
	 *   c: speed of light in a vacuum [meter/second]
	 *
	 * https://en.wikipedia.org/wiki/Free-space_path_loss
	 */
	lambda = SPEED_LIGHT / f;
	denominator = pow(lambda, 2);
	numerator = pow((4.0 * M_PI * d), 2) * medium->system_loss;
	PL = 10.0 * log10(numerator / denominator);
	return (int) PL;
}

/**
 * @brief Calculate path loss based on a log distance model
 * 
 * @param medium 
 * @param src 
 * @param dst
 * @return path loss in dBm
 */
static int calc_path_loss_log_distance(struct medium *medium,
				       struct interface *src,
				       struct interface *dst)
{
	double PL, PL0, d;
	double f = src->frequency * pow(10, 6);

	if (f < 0.1)
		f = FREQ_CH1;

	d = sqrt((src->position_x - dst->position_x) * 
		      	(src->position_x - dst->position_x) +
		 (src->position_y - dst->position_y) *
		      	(src->position_y - dst->position_y) +
		 (src->position_z - dst->position_z) *
		      	(src->position_z - dst->position_z));

	/*
	 * Calculate PL0 with Free-space path loss in decibels
	 *
	 * 20 * log10 * (4 * M_PI * d * f / c)
	 *   d: distance [meter]
	 *   f: frequency [Hz]
	 *   c: speed of light in a vacuum [meter/second]
	 *
	 * https://en.wikipedia.org/wiki/Free-space_path_loss
	 */
	PL0 = 20.0 * log10(4.0 * M_PI * 1.0 * f / SPEED_LIGHT);

	/*
	 * Calculate signal strength with Log-distance path loss model
	 * https://en.wikipedia.org/wiki/Log-distance_path_loss_model
	 */
	PL = PL0 + 10.0 * medium->path_loss_exponent * log10(d) + medium->xg;
	return (int) PL;
}

/**
 * @brief Calculate path loss based on a itu model
 * 
 * @param medium 
 * @param src 
 * @param dst 
 * @return path loss in dBm 
 */
static int calc_path_loss_itu(struct medium *medium, struct interface *src,
			      struct interface *dst)
{
	double PL, d;
	double f = src->frequency;
	int N = 28;
	int pL = medium->power_loss_coeff;

	if (f < 0.1)
		f = FREQ_CH1;

	d = sqrt((src->position_x - dst->position_x) * 
		      	(src->position_x - dst->position_x) +
		 (src->position_y - dst->position_y) *
		      	(src->position_y - dst->position_y) +
		 (src->position_z - dst->position_z) *
		      	(src->position_z - dst->position_z));

	if (d > 16)
		N = 38;
	if (pL != 0)
		N = pL;
	/*
	 * Calculate signal strength with ITU path loss model
	 * Power Loss Coefficient Based on the Paper
	 * Site-Specific Validation of ITU Indoor Path Loss Model at 2.4 GHz
	 * from Theofilos Chrysikos, Giannis Georgopoulos and Stavros Kotsopoulos
	 * LF: floor penetration loss factor
	 * nFLOORS: number of floors
	 */

	PL = 20.0 * log10(f) + N * log10(d) +
		medium->floor_pen_factor * medium->n_floors - 28;
	return (int) PL;
}

/*
 * Calculate path loss based on a log-normal shadowing model
 *
 * This function returns path loss [dBm].
 */
static int calc_path_loss_log_normal_shadowing(struct medium *medium,
					       struct interface *src,
					       struct interface *dst)
{
	double PL, PL0, d;
	double f = src->frequency * pow(10, 6);
	double gRandom = 1; // FIXME WHAT IS THIS??

	if (f < 0.1)
		f = FREQ_CH1;

	d = sqrt((src->position_x - dst->position_x) * 
		      	(src->position_x - dst->position_x) +
		 (src->position_y - dst->position_y) *
		      	(src->position_y - dst->position_y) +
		 (src->position_z - dst->position_z) *
		      	(src->position_z - dst->position_z));

	/*
	 * Calculate PL0 with Free-space path loss in decibels
	 *
	 * 20 * log10 * (4 * M_PI * d * f / c)
	 *   d: distance [meter]
	 *   f: frequency [Hz]
	 *   c: speed of light in a vacuum [meter/second]
	 *
	 * https://en.wikipedia.org/wiki/Free-space_path_loss
	 */
	PL0 = 20.0 * log10(4.0 * M_PI * 1.0 * f / SPEED_LIGHT);

	/*
	 * Calculate signal strength with
	 * Log-distance path loss model + gRandom (Gaussian random variable)
	 * https://en.wikipedia.org/wiki/Log-distance_path_loss_model
	 */
	PL = PL0 + 10.0 * medium->path_loss_exponent * log10(d) - gRandom;
	return (int) PL;
}

static int calc_path_loss_two_ray_ground(struct medium *medium,
					 struct interface *src,
					 struct interface *dst)
{
	double PL, d;
	double f = (double) src->frequency;

	if (f < 0.1)
		f = FREQ_CH1;

	d = sqrt((src->position_x - dst->position_x) * 
		      	(src->position_x - dst->position_x) +
		 (src->position_y - dst->position_y) *
		      	(src->position_y - dst->position_y));

	// struct interface .position_z is the antenna height
	PL = 10 * log10(pow(src->position_z * dst->position_z, 2)) -
	     10 * log10(pow(d, 4)) - 10 * log10(medium->system_loss);
	return (int) PL;
}

static void recalc_path_loss(struct medium *medium)
{
	int path_loss, gains;
	for (unsigned itf1 = 0; itf1 < medium->n_interfaces; itf1++) {
		for (unsigned itf2 = 0; itf2 < medium->n_interfaces; itf2++) {
			if (itf1 == itf2)
				continue;
			path_loss = medium->path_loss_func(medium,
					&(medium->interfaces[itf1]),
					&(medium->interfaces[itf2]));
			gains = medium->interfaces[itf1].tx_power +
				medium->interfaces[itf1].antenna_gain +
				medium->interfaces[itf2].antenna_gain;
			medium->snr_matrix[medium->n_interfaces * itf1 + itf2] =
				gains - path_loss - medium->noise_level;
		}
	}
}

/**
 * @brief Moves station and recalculates path loss.
 * 
 * @param medium 
 * 
 */
static void move_interfaces(struct medium *medium)
{
	for (unsigned int i = 0; i < medium->n_interfaces; i++) {
		medium->interfaces[i].position_x +=
			medium->interfaces[i].direction_x;
		medium->interfaces[i].position_y +=
			medium->interfaces[i].direction_y;
		medium->interfaces[i].position_z +=
			medium->interfaces[i].direction_z;
	}
	recalc_path_loss(medium);
}
