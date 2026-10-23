/*
 *	subsystem1.c
 *	Subsystem 1 -- communication and telemetry: kernel objects and tasks.
 *
 *	Kernel objects (TEAM.md section 6 budget):
 *	  flg_comm        event flag   tick / inbound frame
 *	  mbf_telemetry   message buf  Collection -> Communication (4 frames)
 *	  mbf_mqtt_tx     message buf  Communication -> MQTT client (8 frames)
 *	  mbf_mqtt_rx     message buf  MQTT client -> Communication (4 frames)
 *	  cyc_comm 100 ms, cyc_tlm 500 ms
 *	  comm_task 20 (core 0), telemetry_task 21 (core 0)
 *	No pins: the radio belongs to the kernel's cyw43_utk_task.
 */

#include "subsystem1.h"

EXPORT ID	mbf_telemetry;
EXPORT ID	mbf_mqtt_tx;
EXPORT ID	mbf_mqtt_rx;
EXPORT ID	flg_comm;
EXPORT ID	cyc_comm;
EXPORT ID	cyc_tlm;
EXPORT ID	tid_comm;
EXPORT ID	tid_tlm;

LOCAL ID make_mbf(INT n, INT maxmsz)
{
	T_CMBF	cmbf = {0};

	cmbf.mbfatr = TA_TFIFO;
	cmbf.bufsz  = APP_MBF_BUFSZ(n, maxmsz);
	cmbf.maxmsz = maxmsz;
	return tk_cre_mbf(&cmbf);
}

LOCAL ID make_cyc(FP hdr, RELTIM period)
{
	T_CCYC	ccyc = {0};

	ccyc.cycatr = TA_HLNG;
	ccyc.cychdr = hdr;
	ccyc.cyctim = period;
	ccyc.cycphs = period;
#if TK_SUPPORT_SMP
	ccyc.cycatr |= TA_ASSPRC;
	ccyc.assprc  = APP_CORE_COMMS;
#endif
	return tk_cre_cyc(&ccyc);
}

EXPORT ER subsystem1_init(void)
{
	T_CFLG	cflg = {0};

	s1_comm_word = 0;
	s1_tlm_dropped = 0;
	s1_rx_rejected = 0;
	s1_status_now = 0;

	cflg.flgatr = TA_TFIFO | TA_WSGL;
	flg_comm = tk_cre_flg(&cflg);
	if (flg_comm < E_OK) {
		return flg_comm;
	}
	mbf_telemetry = make_mbf(S1_TLM_QUEUE_LEN, S1_MSG_MAX);
	if (mbf_telemetry < E_OK) {
		return mbf_telemetry;
	}
	mbf_mqtt_tx = make_mbf(S1_TX_QUEUE_LEN, S1_MSG_MAX);
	if (mbf_mqtt_tx < E_OK) {
		return mbf_mqtt_tx;
	}
	mbf_mqtt_rx = make_mbf(S1_RX_QUEUE_LEN, S1_MQTT_FRAME_MAX);
	if (mbf_mqtt_rx < E_OK) {
		return mbf_mqtt_rx;
	}
	cyc_comm = make_cyc((FP)comm_cyclic_handler, S1_COMM_PERIOD_MS);
	if (cyc_comm < E_OK) {
		return cyc_comm;
	}
	cyc_tlm = make_cyc((FP)tlm_cyclic_handler, S1_TLM_PERIOD_MS);
	return (cyc_tlm < E_OK) ? cyc_tlm : E_OK;
}

EXPORT ER subsystem1_start(void)
{
	ER	er;

	tid_comm = app_start_task(comm_task, S1_PRI_COMM, APP_CORE_COMMS, "comm_task");
	if (tid_comm < E_OK) {
		return tid_comm;
	}
	tid_tlm = app_start_task(telemetry_task, S1_PRI_TLM, APP_CORE_COMMS,
				 "telemetry_task");
	if (tid_tlm < E_OK) {
		return tid_tlm;
	}
	er = tk_sta_cyc(cyc_comm);
	return (er < E_OK) ? er : tk_sta_cyc(cyc_tlm);
}
