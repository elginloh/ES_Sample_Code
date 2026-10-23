/*
 *	s5_msg.c
 *	Pack/unpack of subsystem 5 messages (see s5_msg.h).
 */

#include "s5_msg.h"

#define F(v, mask, sh)	(((UW)(v) & (mask)) << (sh))

LOCAL W sext(UW v, UINT bits)
{
	UW	sign = 1UL << (bits - 1U);

	v &= (sign << 1) - 1U;
	return (W)((v ^ sign) - sign);
}

LOCAL W clamp(W v, W lo, W hi)
{
	return (v < lo) ? lo : ((v > hi) ? hi : v);
}

EXPORT UH s5_dist_code(UW mm)
{
	UW	c = (mm + S5_DIST_STEP_MM / 2U) / S5_DIST_STEP_MM;

	return (UH)((c > S5_DIST_MAX_CODE) ? S5_DIST_MAX_CODE : c);
}

EXPORT W s5_dist_mm(UH code)
{
	if (code > S5_DIST_MAX_CODE) {
		return -1;
	}
	return (W)code * S5_DIST_STEP_MM;
}

EXPORT UW s5_req_pack(const S5_SCAN_REQ *r)
{
	return F(r->type, 0x3UL, 0) | F(r->scan_id, 0x7UL, 2)
	     | F(r->start_deg, 0xFFUL, 5) | F(r->end_deg, 0xFFUL, 13)
	     | F(r->step_deg, 0xFUL, 21);
}

EXPORT void s5_req_unpack(UW w, S5_SCAN_REQ *r)
{
	r->type = (UB)(w & 0x3UL);
	r->scan_id = (UB)((w >> 2) & 0x7UL);
	r->start_deg = (B)sext(w >> 5, 8);
	r->end_deg = (B)sext(w >> 13, 8);
	r->step_deg = (UB)((w >> 21) & 0xFUL);
}

EXPORT UW s5_sample_pack(const S5_SAMPLE *s)
{
	return F(s->angle_deg, 0xFFUL, 0) | F(s->dist_code, 0x3FFUL, 8)
	     | F(s->scan_id, 0x7UL, 18) | F(s->phase, 0x1UL, 21)
	     | F(s->last, 0x1UL, 22) | F(s->status, 0x3UL, 23);
}

EXPORT void s5_sample_unpack(UW w, S5_SAMPLE *s)
{
	s->angle_deg = (B)sext(w, 8);
	s->dist_code = (UH)((w >> 8) & 0x3FFUL);
	s->scan_id = (UB)((w >> 18) & 0x7UL);
	s->phase = (UB)((w >> 21) & 0x1UL);
	s->last = (UB)((w >> 22) & 0x1UL);
	s->status = (UB)((w >> 23) & 0x3UL);
}

EXPORT void s5_profile_pack(const S5_PROFILE *p, UW w[2])
{
	UH	dist = (p->dist_mm < 0) ? (UH)S5_DIST_INVALID : s5_dist_code((UW)p->dist_mm);
	UW	width = (p->width_mm < 0) ? S5_WIDTH_UNKNOWN
		      : (UW)clamp((p->width_mm + 2) / 5, 0, 254);
	UW	cl = (p->clear_left_mm < 0) ? S5_CLEAR_UNKNOWN
		   : (UW)clamp((p->clear_left_mm + 5) / 10, 0, 254);
	UW	cr = (p->clear_right_mm < 0) ? S5_CLEAR_UNKNOWN
		   : (UW)clamp((p->clear_right_mm + 5) / 10, 0, 254);
	W	le = clamp(p->left_mm / 10, -127, 127);
	W	re = clamp(p->right_mm / 10, -127, 127);

	w[0] = F(p->scan_id, 0x7UL, 0) | F(p->status, 0x3UL, 3)
	     | F(dist, 0x3FFUL, 5) | F(clamp(p->bearing_deg, -127, 127), 0xFFUL, 15)
	     | F(width, 0xFFUL, 23) | F(p->phase, 0x1UL, 31);
	w[1] = F(le, 0xFFUL, 0) | F(re, 0xFFUL, 8) | F(cl, 0xFFUL, 16) | F(cr, 0xFFUL, 24);
}

EXPORT void s5_profile_unpack(const UW w[2], S5_PROFILE *p)
{
	UW	width = (w[0] >> 23) & 0xFFUL;
	UW	cl = (w[1] >> 16) & 0xFFUL;
	UW	cr = (w[1] >> 24) & 0xFFUL;

	p->scan_id = (UB)(w[0] & 0x7UL);
	p->status = (UB)((w[0] >> 3) & 0x3UL);
	p->dist_mm = s5_dist_mm((UH)((w[0] >> 5) & 0x3FFUL));
	p->bearing_deg = sext(w[0] >> 15, 8);
	p->width_mm = (width == S5_WIDTH_UNKNOWN) ? -1 : (W)width * 5;
	p->phase = (UB)((w[0] >> 31) & 0x1UL);
	p->left_mm = sext(w[1], 8) * 10;
	p->right_mm = sext(w[1] >> 8, 8) * 10;
	p->clear_left_mm = (cl == S5_CLEAR_UNKNOWN) ? -1 : (W)cl * 10;
	p->clear_right_mm = (cr == S5_CLEAR_UNKNOWN) ? -1 : (W)cr * 10;
}

EXPORT UW s5_scan_status_pack(const S5_SCAN_STATUS *s)
{
	return F(s->dist_code, 0x3FFUL, 0) | F(s->servo_deg, 0xFFUL, 10)
	     | F(s->us_result, 0x3UL, 18) | F(s->mode, 0x3UL, 20)
	     | F(s->seq, 0xFUL, 22) | F(s->cap, 0x1UL, 26)
	     | F(s->detected, 0x1UL, 27);
}

EXPORT void s5_scan_status_unpack(UW w, S5_SCAN_STATUS *s)
{
	s->dist_code = (UH)(w & 0x3FFUL);
	s->servo_deg = (B)sext(w >> 10, 8);
	s->us_result = (UB)((w >> 18) & 0x3UL);
	s->mode = (UB)((w >> 20) & 0x3UL);
	s->seq = (UB)((w >> 22) & 0xFUL);
	s->cap = (UB)((w >> 26) & 0x1UL);
	s->detected = (UB)((w >> 27) & 0x1UL);
}

EXPORT UW s5_avoid_status_pack(const S5_AVOID_STATUS *s)
{
	return F(s->state, 0xFUL, 0) | F(s->side, 0x3UL, 4) | F(s->step, 0x7UL, 6)
	     | F(s->retries, 0x3UL, 9) | F(s->outcome, 0x3UL, 11)
	     | F(s->count, 0xFUL, 13) | F(s->fail, 0x7UL, 17);
}

EXPORT void s5_avoid_status_unpack(UW w, S5_AVOID_STATUS *s)
{
	s->state = (UB)(w & 0xFUL);
	s->side = (UB)((w >> 4) & 0x3UL);
	s->step = (UB)((w >> 6) & 0x7UL);
	s->retries = (UB)((w >> 9) & 0x3UL);
	s->outcome = (UB)((w >> 11) & 0x3UL);
	s->count = (UB)((w >> 13) & 0xFUL);
	s->fail = (UB)((w >> 17) & 0x7UL);
}
