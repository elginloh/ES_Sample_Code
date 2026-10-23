/*
 *	s2_private.h
 *	Data shared only between subsystem 2's own files.  Every word has
 *	exactly one writer (noted per line); readers use app_load() or the
 *	sequence count, so no read-modify-write races (Change Requirement 1).
 */

#ifndef S2_PRIVATE_H
#define S2_PRIVATE_H

#include "subsystem2.h"

IMPORT volatile UW	s2_result_word[MSRC_COUNT];	/* writer: motion task */
IMPORT volatile UW	s2_steer_word;			/* writer: line task   */
IMPORT volatile UW	s2_cap_word;			/* writer: scan task   */
IMPORT volatile UW	s2_stop_req;			/* atomic OR / xchg    */
IMPORT volatile UW	s2_armed;			/* writer: tests/init  */
IMPORT volatile UW	s2_speed_avg_mms;		/* writer: motion task */
IMPORT volatile UW	s2_creep_mms;			/* writer: motion task */
IMPORT APP_SEQ		s2_mstat_seq;			/* writer: motion task */
IMPORT UW		s2_mstat_words[2];

IMPORT void sys_state_init(void);
IMPORT UW   sys_take_events(void);			/* motion task only */
IMPORT void sys_vsw_publish(UW vsw);			/* motion task only */
IMPORT void motion_api_init(void);

#endif /* S2_PRIVATE_H */
