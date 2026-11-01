-- 003_run_summary_avoid_failures.sql
-- Avoidance failures in a run are counted once, from the backend's
-- AVOID_FAILED event (avoidance state entering FAILED in telemetry).  With
-- the patched firmware the robot also sends an OBSTACLE event with outcome
-- FAILED (it carries the reason), which 002 counted a second time.  Same
-- signature and columns; no data is touched.

CREATE OR REPLACE FUNCTION csc14.run_summary(p_run uuid)
RETURNS TABLE (
	run_id uuid, name text, source text, started_at timestamptz, ended_at timestamptz,
	duration_s numeric, tlm_frames bigint, tlm_expected bigint, tlm_missing_known bigint,
	tlm_gaps_unknown bigint, max_rx_gap_s numeric, boot_sessions bigint, robot_restarts bigint,
	distance_m numeric, humps bigint, humps_without_height bigint, hump_max_mm smallint,
	barcode_reads_valid bigint, barcode_reads_invalid bigint, avoid_success bigint,
	avoid_failed bigint, collisions bigint, faults_raised text[], link_outages bigint,
	notes text[])
LANGUAGE sql STABLE AS $$
WITH r AS (
	SELECT * FROM csc14.run WHERE run_id = p_run
), t AS (
	SELECT tt.*, lag(tt.received_at) OVER w AS prev_rx
	FROM csc14.telemetry tt WHERE tt.run_id = p_run
	WINDOW w AS (PARTITION BY tt.boot_id ORDER BY tt.received_at, tt.id)
), tg AS (
	SELECT count(*) AS n, coalesce(sum(seq_gap), 0) AS missing,
	       count(*) FILTER (WHERE seq_gap IS NULL) AS unknown_gaps,
	       coalesce(max(extract(epoch FROM received_at - prev_rx)), 0) AS max_gap,
	       count(DISTINCT boot_id) AS boots,
	       bool_or(hump_count = 15) AS hump_saturated,
	       bool_or(odometer_cm = 65535) AS odo_saturated
	FROM t
), dist AS (
	SELECT coalesce(sum(d), 0) AS cm FROM (
		SELECT max(odometer_cm) - min(odometer_cm) AS d FROM t GROUP BY boot_id) x
), last_t AS (
	SELECT hump_active FROM t ORDER BY received_at DESC, id DESC LIMIT 1
), h AS (
	SELECT count(*) AS n, count(*) FILTER (WHERE height_mm IS NULL) AS no_height,
	       max(height_mm) AS max_mm
	FROM csc14.hump WHERE run_id = p_run
), ev AS (
	SELECT count(*) FILTER (WHERE kind = 'OBSTACLE' AND data->>'outcome' IN ('1', '3')) AS av_ok,
	       count(*) FILTER (WHERE kind = 'AVOID_FAILED') AS av_fail,
	       count(*) FILTER (WHERE kind = 'COLLISION') AS coll,
	       count(*) FILTER (WHERE kind IN ('LINK_STALE', 'LINK_OFFLINE')) AS outages,
	       count(*) FILTER (WHERE kind = 'EVENTS_MISSED') AS ev_missed,
	       count(*) FILTER (WHERE kind = 'BARCODE' AND data->>'result' = '1') AS bc_v,
	       count(*) FILTER (WHERE kind = 'BARCODE' AND data->>'result' = '2') AS bc_i,
	       array_agg(DISTINCT data->>'fault') FILTER (WHERE kind = 'FAULT_SET') AS faults
	FROM csc14.event WHERE run_id = p_run
), b AS (
	SELECT count(*) FILTER (WHERE bs.start_reason = 'robot_restart') AS restarts
	FROM csc14.boot_session bs, r
	WHERE bs.robot_id = r.robot_id AND bs.source = r.source
	  AND bs.first_rx_at >= r.started_at AND (r.ended_at IS NULL OR bs.first_rx_at < r.ended_at)
)
SELECT r.run_id, r.name, r.source, r.started_at, r.ended_at,
       round(extract(epoch FROM coalesce(r.ended_at, now()) - r.started_at)::numeric, 1),
       tg.n, floor(extract(epoch FROM coalesce(r.ended_at, now()) - r.started_at) * 2)::bigint,
       tg.missing, tg.unknown_gaps, round(tg.max_gap::numeric, 1), tg.boots, b.restarts,
       round(dist.cm / 100.0, 2), h.n, h.no_height, h.max_mm, ev.bc_v, ev.bc_i,
       ev.av_ok, ev.av_fail, ev.coll, coalesce(ev.faults, '{}'), ev.outages,
       array_remove(ARRAY[
	CASE WHEN tg.n = 0 THEN 'no telemetry received during the run' END,
	CASE WHEN tg.hump_saturated THEN 'robot hump counter saturated at 15: a hump as high as the previous one is not detectable' END,
	CASE WHEN h.no_height > 0 THEN h.no_height || ' hump(s) completed without a received height' END,
	CASE WHEN (SELECT hump_active FROM last_t) THEN 'a hump was being measured when the run ended' END,
	CASE WHEN tg.missing > 0 OR tg.unknown_gaps > tg.boots THEN 'telemetry frames missing: brief changes and events may be absent' END,
	CASE WHEN ev.ev_missed > 0 THEN 'robot event frames missing: barcode, hump or obstacle counts may be low' END,
	CASE WHEN b.restarts > 0 THEN 'robot restarted during the run' END,
	CASE WHEN tg.odo_saturated THEN 'odometer saturated (655 m): distance is a lower bound' END,
	CASE WHEN r.ended_at IS NULL THEN 'run still active' END
       ], NULL)
FROM r, tg, dist, h, ev, b
$$;
