/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Smart-dimming gamma generation for the Samsung S6E3HA8 / AMB577PX01 panel.
 *
 * The panel does not implement DCS brightness (0x51); brightness is set by
 * writing a per-level gamma table (0xca) together with AID (0xb1), ELVSS
 * (0xb5) and ACL (0x55), then latching with 0xf7. The gamma table is not a
 * fixed list -- it is computed from calibration data (MTP) read out of the
 * panel itself, so it differs per unit.
 *
 * All the computation happens once, in ->init(). ->generate_gamma() is then a
 * lookup into the table built there.
 */
#ifndef __S6E3HA8_DIMMING_H__
#define __S6E3HA8_DIMMING_H__

struct SMART_DIM;

struct smartdim_conf {
	void (*generate_gamma)(struct smartdim_conf *conf, int cd, char *str);
	void (*generate_hbm_gamma)(struct smartdim_conf *conf, int cd, char *str);
	void (*init)(struct smartdim_conf *conf);
	void (*print_aid_log)(struct smartdim_conf *conf);
	struct SMART_DIM *psmart;

	void (*get_min_lux_table)(char *str, int size);
	/* MTP read from the panel, register 0xC8, 34 bytes */
	char *mtp_buffer;
	int *lux_tab;
	int lux_tabsize;
	unsigned int man_id;
	char panel_revision;

	char *hbm_payload;
};

#define GAMMA_INDEX_MAX 256

/* Length of the 0xca gamma payload the algorithm produces. */
#define S6E3HA8_GAMMA_LEN 34

struct smartdim_conf *s6e3ha8_smartdim_get_conf(void);

#endif /* __S6E3HA8_DIMMING_H__ */
