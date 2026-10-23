/*
 *	s4_status.c
 *	Pack/unpack of subsystem 4 application data (see s4_status.h).
 */

#include "s4_status.h"

#define F(v, mask, sh)	(((UW)(v) & (mask)) << (sh))

LOCAL W sext(UW v, UINT bits)
{
	UW	sign = 1UL << (bits - 1U);

	v &= (sign << 1) - 1U;
	return (W)((v ^ sign) - sign);		/* two's complement sign-extend */
}

EXPORT B s4_deg8(W cdeg)
{
	W	d = (cdeg >= 0) ? (cdeg + 50) / 100 : (cdeg - 50) / 100;

	if (d > 90) {
		d = 90;
	} else if (d < -90) {
		d = -90;
	}
	return (B)d;
}

EXPORT H s4_rate9(W mdps)
{
	W	d = (mdps >= 0) ? (mdps + 500) / 1000 : (mdps - 500) / 1000;

	if (d > 255) {
		d = 255;
	} else if (d < -255) {
		d = -255;
	}
	return (H)d;
}

EXPORT UB s4_shock5(W mg)
{
	W	h = (mg + 250) / 500;

	if (h < 0) {
		h = 0;
	} else if (h > 31) {
		h = 31;
	}
	return (UB)h;
}

EXPORT void s4_imu_pack(const S4_IMU_STATUS *s, UW w[2])
{
	w[0] = F(s->pitch_deg, 0xFFUL, 0) | F(s->roll_deg, 0xFFUL, 8)
	     | F(s->yaw_rate_dps, 0x1FFUL, 16) | F(s->motion_class, 0x3UL, 25)
	     | F(s->status, 0x3UL, 27) | F(s->tilt, 0x1UL, 29)
	     | F(s->collision, 0x1UL, 30) | F(s->valid, 0x1UL, 31);
	w[1] = F(s->heading_deg, 0x1FFUL, 0) | F(s->shock_half_g, 0x1FUL, 9)
	     | F(s->seq, 0xFFUL, 14) | F(s->chip, 0x3UL, 22)
	     | F(s->calibrated, 0x1UL, 24);
}

EXPORT void s4_imu_unpack(const UW w[2], S4_IMU_STATUS *s)
{
	s->pitch_deg = (B)sext(w[0], 8);
	s->roll_deg = (B)sext(w[0] >> 8, 8);
	s->yaw_rate_dps = (H)sext(w[0] >> 16, 9);
	s->motion_class = (UB)((w[0] >> 25) & 0x3UL);
	s->status = (UB)((w[0] >> 27) & 0x3UL);
	s->tilt = (UB)((w[0] >> 29) & 0x1UL);
	s->collision = (UB)((w[0] >> 30) & 0x1UL);
	s->valid = (UB)((w[0] >> 31) & 0x1UL);
	s->heading_deg = (UH)(w[1] & 0x1FFUL);
	s->shock_half_g = (UB)((w[1] >> 9) & 0x1FUL);
	s->seq = (UB)((w[1] >> 14) & 0xFFUL);
	s->chip = (UB)((w[1] >> 22) & 0x3UL);
	s->calibrated = (UB)((w[1] >> 24) & 0x1UL);
}

EXPORT UW s4_hump_pack(const S4_HUMP_STATUS *s)
{
	return F(s->last_code, 0x3FUL, 0) | F(s->max_code, 0x3FUL, 6)
	     | F((s->count > 15U) ? 15U : s->count, 0xFUL, 12)
	     | F(s->active, 0x1UL, 16) | F(s->last_sat, 0x1UL, 17)
	     | F(s->max_sat, 0x1UL, 18) | F(s->last_len_cm, 0xFFUL, 19);
}

EXPORT void s4_hump_unpack(UW w, S4_HUMP_STATUS *s)
{
	s->last_code = (UB)(w & 0x3FUL);
	s->max_code = (UB)((w >> 6) & 0x3FUL);
	s->count = (UB)((w >> 12) & 0xFUL);
	s->active = (UB)((w >> 16) & 0x1UL);
	s->last_sat = (UB)((w >> 17) & 0x1UL);
	s->max_sat = (UB)((w >> 18) & 0x1UL);
	s->last_len_cm = (UB)((w >> 19) & 0xFFUL);
}

EXPORT UW s4_sample_pack(const S4_HUMP_SAMPLE *s)
{
	W	p = s->pitch_ddeg;

	if (p > 1023) {
		p = 1023;
	} else if (p < -1023) {
		p = -1023;
	}
	return F(p, 0x7FFUL, 0) | F(s->ds_100um, 0xFFUL, 11)
	     | F(s->valid, 0x1UL, 19) | F(s->seq, 0xFUL, 20);
}

EXPORT void s4_sample_unpack(UW w, S4_HUMP_SAMPLE *s)
{
	s->pitch_ddeg = (H)sext(w, 11);
	s->ds_100um = (UB)((w >> 11) & 0xFFUL);
	s->valid = (UB)((w >> 19) & 0x1UL);
	s->seq = (UB)((w >> 20) & 0xFUL);
}
