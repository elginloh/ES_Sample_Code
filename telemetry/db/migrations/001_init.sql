-- 001_init.sql
-- CSC14 telemetry schema.  Applied once by tools/migrate.js inside a
-- transaction; never edit an applied migration (the tool checks its
-- checksum), add 002_... instead.  Nothing here drops or rewrites data.
--
-- Times: received_at is when the backend (Node-RED) received the frame,
-- UTC timestamptz.  The robot sends no clock time; uptime_ds is the robot's
-- 16-bit uptime (0.1 s, wraps every 6553.6 s) and uptime_s the backend's
-- unwrapped value within one inferred boot session.

CREATE SCHEMA IF NOT EXISTS csc14;

-- Lookup tables, filled by tools/migrate.js from the protocol codec
CREATE TABLE csc14.enum_label (
	enum_name	text NOT NULL,
	code		smallint NOT NULL,
	label		text NOT NULL,
	PRIMARY KEY (enum_name, code)
);

CREATE TABLE csc14.fault_bit (
	bit		smallint PRIMARY KEY,
	name		text NOT NULL,
	owner		text NOT NULL,
	severity	text NOT NULL	-- under the policy flags in .env
);

CREATE FUNCTION csc14.lbl(p_enum text, p_code integer) RETURNS text
LANGUAGE sql STABLE PARALLEL SAFE AS $$
	SELECT coalesce((SELECT label FROM csc14.enum_label WHERE enum_name = p_enum AND code = p_code),
			CASE WHEN p_code IS NULL THEN NULL ELSE 'CODE_' || p_code END)
$$;

CREATE FUNCTION csc14.fault_names(p_mask integer) RETURNS text[]
LANGUAGE sql STABLE PARALLEL SAFE AS $$
	SELECT coalesce(array_agg(name ORDER BY bit), '{}')
	FROM csc14.fault_bit WHERE (p_mask >> bit) & 1 = 1
$$;

-- ---------------------------------------------------------------------
-- Runs (experiments).  Started and ended explicitly (bin/csc14 run ...);
-- never by a robot reboot or reconnect.  Rows received while no run is
-- active are kept with run_id NULL.
CREATE TABLE csc14.run (
	run_id		uuid PRIMARY KEY DEFAULT gen_random_uuid(),
	robot_id	text NOT NULL,
	source		text NOT NULL CHECK (source IN ('live', 'simulated')),
	name		text NOT NULL,
	description	text,
	notes		text,
	started_at	timestamptz NOT NULL DEFAULT now(),
	ended_at	timestamptz,
	firmware	jsonb NOT NULL DEFAULT '{}',	-- commit, build profile, flags if known
	created_by	text,
	CHECK (ended_at IS NULL OR ended_at >= started_at)
);
CREATE UNIQUE INDEX run_one_active ON csc14.run (robot_id, source) WHERE ended_at IS NULL;
CREATE INDEX run_by_start ON csc14.run (robot_id, source, started_at DESC);

CREATE FUNCTION csc14.start_run(p_robot text, p_source text, p_name text, p_description text,
				p_notes text, p_firmware jsonb, p_created_by text)
RETURNS uuid LANGUAGE plpgsql AS $$
DECLARE
	v_id uuid;
BEGIN
	IF EXISTS (SELECT 1 FROM csc14.run WHERE robot_id = p_robot AND source = p_source AND ended_at IS NULL) THEN
		RAISE EXCEPTION 'a % run is already active for %; end it first', p_source, p_robot
			USING ERRCODE = 'unique_violation';
	END IF;
	INSERT INTO csc14.run (robot_id, source, name, description, notes, firmware, created_by)
	VALUES (p_robot, p_source, p_name, p_description, p_notes, coalesce(p_firmware, '{}'), p_created_by)
	RETURNING run_id INTO v_id;
	RETURN v_id;
END $$;

CREATE FUNCTION csc14.end_run(p_robot text, p_source text, p_notes text)
RETURNS uuid LANGUAGE plpgsql AS $$
DECLARE
	v_id uuid;
BEGIN
	UPDATE csc14.run SET ended_at = now(),
		notes = CASE WHEN p_notes IS NULL OR p_notes = '' THEN notes
			     ELSE coalesce(notes || E'\n', '') || p_notes END
	WHERE robot_id = p_robot AND source = p_source AND ended_at IS NULL
	RETURNING run_id INTO v_id;
	RETURN v_id;		-- NULL when no run was active
END $$;

-- Every data row joins the run that was active for its robot and source
-- at received_at, also when it is written late (buffered during a
-- database outage).
CREATE FUNCTION csc14.assign_run() RETURNS trigger LANGUAGE plpgsql AS $$
BEGIN
	IF NEW.run_id IS NULL THEN
		SELECT r.run_id INTO NEW.run_id FROM csc14.run r
		WHERE r.robot_id = NEW.robot_id AND r.source = NEW.source
		  AND r.started_at <= NEW.received_at
		  AND (r.ended_at IS NULL OR NEW.received_at < r.ended_at)
		ORDER BY r.started_at DESC LIMIT 1;
	END IF;
	RETURN NEW;
END $$;

-- ---------------------------------------------------------------------
-- Inferred boot sessions (the protocol has no boot id): one per stretch of
-- continuous robot uptime that the backend observed.
CREATE TABLE csc14.boot_session (
	boot_id		uuid PRIMARY KEY,
	robot_id	text NOT NULL,
	source		text NOT NULL,
	first_rx_at	timestamptz NOT NULL,
	last_rx_at	timestamptz NOT NULL,
	first_uptime_s	numeric(10,1),
	last_uptime_s	numeric(10,1),
	start_reason	text NOT NULL,	-- first_seen | robot_restart | uptime_discontinuity | continuity_unknown
	ir_sides_mirrored boolean,	-- interpretation settings in force
	policy		jsonb
);
CREATE INDEX boot_session_by_time ON csc14.boot_session (robot_id, source, first_rx_at DESC);

-- ---------------------------------------------------------------------
-- Telemetry snapshots (type 1, every 500 ms).  Typed columns carry the
-- measurements in engineering units (NULL = the robot reported the value
-- as unavailable); enumerations are firmware codes (csc14.lbl() names
-- them); frame keeps the original 40 bytes; decoded the complete decoded
-- record when CSC14_STORE_DECODED_JSON=1.
CREATE TABLE csc14.telemetry (
	id		bigint GENERATED ALWAYS AS IDENTITY PRIMARY KEY,
	run_id		uuid REFERENCES csc14.run (run_id),
	robot_id	text NOT NULL,
	source		text NOT NULL,
	boot_id		uuid,
	received_at	timestamptz NOT NULL,
	stored_at	timestamptz NOT NULL DEFAULT clock_timestamp(),
	topic		text NOT NULL,
	proto_version	smallint NOT NULL,
	seq		smallint NOT NULL,		-- 8-bit, per message type
	seq_gap		integer,			-- frames missing before this one; NULL unknown
	rx_delay_ms	integer,			-- estimated lateness vs. the session's fastest frame
	late		boolean,			-- generated well before it was received
	uptime_ds	integer NOT NULL,		-- robot uptime, raw 16-bit, 0.1 s
	uptime_s	numeric(10,1),			-- unwrapped within the boot session
	-- system
	mode		smallint NOT NULL,		-- lbl('mode')
	epoch		smallint NOT NULL,
	fault_mask	integer NOT NULL,		-- ACTIVE faults (27-bit)
	critical	boolean NOT NULL,		-- any active fault critical under the policy flags
	-- motion
	motion_state	smallint NOT NULL,		-- lbl('motion_state')
	motion_source	smallint NOT NULL,		-- lbl('motion_source')
	cmd_active	boolean NOT NULL,
	speed_l_mms	smallint,			-- measured wheel speed, mm/s
	speed_r_mms	smallint,
	odometer_cm	integer NOT NULL,		-- total travel since boot, saturates at 65535
	heading_enc_cw_deg smallint NOT NULL,		-- encoder heading, + clockwise, 2 deg steps
	cruise_set_mms	smallint NOT NULL,		-- cruise SETTING, 10 mm/s steps (not a wheel target)
	-- line (car's real sides, see ir_sides_mirrored in boot_session)
	ir_raw		smallint NOT NULL,		-- firmware bits: 0 IR_L/GP2, 1 IR_C/GP6, 2 IR_R/GP27
	ir_left		boolean NOT NULL,		-- true = black
	ir_centre	boolean NOT NULL,
	ir_right	boolean NOT NULL,
	line_pos	smallint NOT NULL,		-- lbl('line_pos'), line follower's own labels
	line_pos_phys	smallint NOT NULL,		-- lbl('line_pos_phys'), car's real sides
	line_state	smallint NOT NULL,		-- lbl('line_state')
	steer		smallint NOT NULL,		-- -63..63, + = turn right (clockwise)
	-- barcode
	bc_state	smallint NOT NULL,		-- lbl('bc_state')
	bc_symbol	text,				-- last Code 39 character, NULL none
	bc_result	smallint NOT NULL,		-- lbl('bc_result')
	nav		smallint,			-- lbl('nav') mapping of the last symbol; NULL unmapped
	nav_valid	boolean NOT NULL,
	bc_valid_mod64	smallint NOT NULL,		-- counters modulo 64
	bc_invalid_mod64 smallint NOT NULL,
	-- IMU (LSM303DLHC: accelerometer + magnetometer, no gyroscope)
	imu_status	smallint NOT NULL,		-- lbl('imu_status')
	pitch_deg	smallint,			-- nose up +
	roll_deg	smallint,			-- right side down +
	yaw_rate_ccw_dps smallint,			-- + = left / counter-clockwise
	heading_imu_ccw_deg smallint,			-- magnetometer heading, + CCW from calibration
	motion_class	smallint NOT NULL,		-- lbl('motion_class')
	tilt		boolean NOT NULL,
	collision	boolean NOT NULL,		-- latched until CLEAR/RESUME
	shock_g		real NOT NULL,			-- peak horizontal impact, 0.5 g steps
	-- humps (2 mm steps)
	hump_last_mm	smallint,
	hump_max_boot_mm smallint,			-- highest since ROBOT BOOT, not per run
	hump_count	smallint NOT NULL,		-- saturates at 15
	hump_active	boolean NOT NULL,
	-- obstacle (5 mm range steps)
	us_dist_mm	smallint,			-- latest ranging, whatever the servo angle
	us_no_echo	boolean NOT NULL,		-- nothing within 4 m
	us_result	smallint NOT NULL,		-- lbl('us_result')
	us_context	text NOT NULL,			-- forward | scan_sweep | held
	us_forward_mm	smallint,			-- us_dist_mm only when it is a forward range
	avoid_state	smallint NOT NULL,		-- lbl('avoid_state')
	avoid_side	smallint NOT NULL,		-- lbl('avoid_side')
	avoid_outcome	smallint NOT NULL,		-- lbl('avoid_outcome') of the last avoidance
	profile_reported boolean NOT NULL,		-- false = no profile since boot (zero words)
	obst_width_mm	smallint,
	obst_bearing_deg smallint,			-- + = right
	clear_left_mm	smallint,
	clear_left_at_least boolean NOT NULL,		-- true: clearance >= 2540 mm
	clear_right_mm	smallint,
	clear_right_at_least boolean NOT NULL,
	-- communication (robot view)
	comm_state	smallint NOT NULL,		-- lbl('comm_state')
	tlm_dropped	smallint NOT NULL,		-- saturates at 255
	rx_rejected	smallint NOT NULL,		-- saturates at 255
	frame		bytea NOT NULL,
	decoded		jsonb
);
CREATE INDEX telemetry_by_time ON csc14.telemetry (robot_id, source, received_at DESC);
CREATE INDEX telemetry_by_run ON csc14.telemetry (run_id, received_at) WHERE run_id IS NOT NULL;
CREATE TRIGGER telemetry_run BEFORE INSERT ON csc14.telemetry
	FOR EACH ROW EXECUTE FUNCTION csc14.assign_run();

-- Heartbeats (type 2, every second while the robot is online)
CREATE TABLE csc14.heartbeat (
	id		bigint GENERATED ALWAYS AS IDENTITY PRIMARY KEY,
	run_id		uuid REFERENCES csc14.run (run_id),
	robot_id	text NOT NULL,
	source		text NOT NULL,
	boot_id		uuid,
	received_at	timestamptz NOT NULL,
	stored_at	timestamptz NOT NULL DEFAULT clock_timestamp(),
	topic		text NOT NULL,
	seq		smallint NOT NULL,
	seq_gap		integer,
	uptime_ds	integer NOT NULL,
	uptime_s	numeric(10,1),
	mode		smallint NOT NULL,
	comm_state	smallint NOT NULL,
	critical	boolean NOT NULL,
	fault_count	smallint NOT NULL,
	reconnects	smallint NOT NULL,		-- saturates at 15
	frame		bytea NOT NULL
);
CREATE INDEX heartbeat_by_time ON csc14.heartbeat (robot_id, source, received_at DESC);
CREATE INDEX heartbeat_by_run ON csc14.heartbeat (run_id, received_at) WHERE run_id IS NOT NULL;
CREATE TRIGGER heartbeat_run BEFORE INSERT ON csc14.heartbeat
	FOR EACH ROW EXECUTE FUNCTION csc14.assign_run();

-- Events: robot frames (origin 'robot'), changes the backend derived from
-- telemetry or time (origin 'server'), commands seen on the bus ('bus')
CREATE TABLE csc14.event (
	id		bigint GENERATED ALWAYS AS IDENTITY PRIMARY KEY,
	run_id		uuid REFERENCES csc14.run (run_id),
	robot_id	text NOT NULL,
	source		text NOT NULL,
	boot_id		uuid,
	received_at	timestamptz NOT NULL,
	stored_at	timestamptz NOT NULL DEFAULT clock_timestamp(),
	topic		text,
	origin		text NOT NULL CHECK (origin IN ('robot', 'server', 'bus')),
	kind		text NOT NULL,
	severity	text NOT NULL CHECK (severity IN ('info', 'warning', 'critical')),
	seq		smallint,
	message		text NOT NULL,
	data		jsonb NOT NULL DEFAULT '{}',
	frame		bytea
);
CREATE INDEX event_by_time ON csc14.event (robot_id, source, received_at DESC);
CREATE INDEX event_by_run ON csc14.event (run_id, received_at) WHERE run_id IS NOT NULL;
CREATE INDEX event_by_kind ON csc14.event (kind, received_at DESC);
CREATE TRIGGER event_run BEFORE INSERT ON csc14.event
	FOR EACH ROW EXECUTE FUNCTION csc14.assign_run();

-- Individual hump measurements (FR6).  One row per robot hump count in a
-- boot session; robot HUMP events and telemetry count changes describe the
-- same hump and merge into one row.  count_after NULL: found after the
-- robot's 4-bit counter saturated.
CREATE TABLE csc14.hump (
	id		bigint GENERATED ALWAYS AS IDENTITY PRIMARY KEY,
	run_id		uuid REFERENCES csc14.run (run_id),
	robot_id	text NOT NULL,
	source		text NOT NULL,
	boot_id		uuid NOT NULL,
	received_at	timestamptz NOT NULL,
	stored_at	timestamptz NOT NULL DEFAULT clock_timestamp(),
	topic		text,
	height_mm	smallint,			-- NULL: completed but height never received
	height_code	smallint,
	count_after	smallint,
	max_boot_mm	smallint,			-- robot's highest since boot at that time
	detected_by	text NOT NULL,			-- robot_event, telemetry, max_increase, last_change (+ joined)
	UNIQUE (boot_id, count_after)
);
CREATE INDEX hump_by_time ON csc14.hump (robot_id, source, received_at DESC);
CREATE INDEX hump_by_run ON csc14.hump (run_id) WHERE run_id IS NOT NULL;
CREATE TRIGGER hump_run BEFORE INSERT ON csc14.hump
	FOR EACH ROW EXECUTE FUNCTION csc14.assign_run();

-- Command acknowledgements (type 5)
CREATE TABLE csc14.ack (
	id		bigint GENERATED ALWAYS AS IDENTITY PRIMARY KEY,
	run_id		uuid REFERENCES csc14.run (run_id),
	robot_id	text NOT NULL,
	source		text NOT NULL,
	boot_id		uuid,
	received_at	timestamptz NOT NULL,
	stored_at	timestamptz NOT NULL DEFAULT clock_timestamp(),
	topic		text NOT NULL,
	seq		smallint NOT NULL,
	cmd_id		smallint NOT NULL,
	cmd		smallint NOT NULL,		-- lbl('command')
	result		smallint NOT NULL,		-- lbl('ack_result')
	detail		smallint NOT NULL,		-- lbl('reject_reason') when result = REJ_STATE
	frame		bytea NOT NULL
);
CREATE INDEX ack_by_time ON csc14.ack (robot_id, source, received_at DESC);
CREATE TRIGGER ack_run BEFORE INSERT ON csc14.ack
	FOR EACH ROW EXECUTE FUNCTION csc14.assign_run();

-- Commands sent by tools/command.js (optional; movement-related commands
-- are refused unless CSC14_ALLOW_MOTION_COMMANDS=1)
CREATE TABLE csc14.command (
	id		bigint GENERATED ALWAYS AS IDENTITY PRIMARY KEY,
	run_id		uuid REFERENCES csc14.run (run_id),
	robot_id	text NOT NULL,
	source		text NOT NULL,
	received_at	timestamptz NOT NULL DEFAULT now(),	-- when it was published
	requested_by	text,
	topic		text NOT NULL,
	cmd		text NOT NULL,
	arg		integer NOT NULL DEFAULT 0,
	cmd_id		smallint NOT NULL,
	seq		smallint NOT NULL,
	frame		bytea NOT NULL,
	status		text NOT NULL DEFAULT 'pending'
			CHECK (status IN ('pending', 'accepted', 'rejected', 'send_failed')),
	ack_result	text,
	ack_detail	text,
	acked_at	timestamptz,
	ack_id		bigint
);
CREATE INDEX command_pending ON csc14.command (robot_id, source, cmd_id, received_at DESC);
CREATE TRIGGER command_run BEFORE INSERT ON csc14.command
	FOR EACH ROW EXECUTE FUNCTION csc14.assign_run();

CREATE FUNCTION csc14.apply_ack() RETURNS trigger LANGUAGE plpgsql AS $$
BEGIN
	UPDATE csc14.command c SET
		status = CASE WHEN NEW.result = 0 THEN 'accepted' ELSE 'rejected' END,
		ack_result = csc14.lbl('ack_result', NEW.result),
		ack_detail = CASE WHEN NEW.result = 6 THEN csc14.lbl('reject_reason', NEW.detail)
				  ELSE NEW.detail::text END,
		acked_at = NEW.received_at, ack_id = NEW.id
	WHERE c.id = (SELECT id FROM csc14.command
		      WHERE robot_id = NEW.robot_id AND source = NEW.source AND cmd_id = NEW.cmd_id
			AND status = 'pending' AND received_at > NEW.received_at - interval '30 seconds'
		      ORDER BY received_at DESC LIMIT 1);
	RETURN NULL;
END $$;
CREATE TRIGGER ack_to_command AFTER INSERT ON csc14.ack
	FOR EACH ROW EXECUTE FUNCTION csc14.apply_ack();

-- Frames that failed validation; never in the measurement tables
CREATE TABLE csc14.rejected_frame (
	id		bigint GENERATED ALWAYS AS IDENTITY PRIMARY KEY,
	run_id		uuid REFERENCES csc14.run (run_id),
	robot_id	text NOT NULL,
	source		text NOT NULL,
	received_at	timestamptz NOT NULL,
	stored_at	timestamptz NOT NULL DEFAULT clock_timestamp(),
	topic		text NOT NULL,
	reason		text NOT NULL,			-- SHORT, CRC, VERSION, TYPE, LENGTH, FIELD_RANGE
	detail		text,
	length		integer NOT NULL,
	frame		bytea				-- first 128 bytes
);
CREATE INDEX rejected_by_time ON csc14.rejected_frame (received_at DESC);
CREATE TRIGGER rejected_run BEFORE INSERT ON csc14.rejected_frame
	FOR EACH ROW EXECUTE FUNCTION csc14.assign_run();

-- Latest link state per robot and source (the backend's view)
CREATE TABLE csc14.robot_link (
	robot_id	text NOT NULL,
	source		text NOT NULL,
	state		text NOT NULL,			-- ONLINE, STALE, OFFLINE, UNKNOWN, NEVER_SEEN
	state_since	timestamptz,
	last_rx_at	timestamptz,
	last_hb_at	timestamptz,
	last_tlm_at	timestamptz,
	last_reconnects	smallint,
	last_comm_state	smallint,
	boot_id		uuid,
	updated_at	timestamptz NOT NULL,
	PRIMARY KEY (robot_id, source)
);

-- Pipeline health snapshots written by Node-RED every 10 s
CREATE TABLE csc14.ingest_health (
	id		bigint GENERATED ALWAYS AS IDENTITY PRIMARY KEY,
	at		timestamptz NOT NULL,
	stored_at	timestamptz NOT NULL DEFAULT clock_timestamp(),
	instance	text NOT NULL,
	broker_connected boolean,
	db_connected	boolean,
	queue_bulk	integer,
	queue_priority	integer,
	dropped_bulk_total bigint,
	dropped_priority_total bigint,
	written_total	bigint,
	write_errors_total bigint,
	latency_p50_ms	real,				-- frame received -> row committed
	latency_p95_ms	real,
	latency_max_ms	real,
	frames_total	bigint,
	rejected_total	bigint,
	rss_mb		real,
	heap_mb		real,
	cpu_pct		real,
	uptime_s	integer,
	details		jsonb
);
CREATE INDEX ingest_health_by_time ON csc14.ingest_health (at DESC);

-- ---------------------------------------------------------------------
-- Views for the dashboard (no frame bytes, labels resolved)
CREATE VIEW csc14.v_telemetry AS
SELECT t.id, t.run_id, t.robot_id, t.source, t.boot_id, t.received_at, t.seq, t.seq_gap,
       t.rx_delay_ms, t.late, t.uptime_s,
       t.mode, csc14.lbl('mode', t.mode) AS mode_label, t.epoch, t.fault_mask,
       csc14.fault_names(t.fault_mask) AS faults, t.critical,
       csc14.lbl('motion_state', t.motion_state) AS motion_state_label,
       csc14.lbl('motion_source', t.motion_source) AS motion_source_label, t.cmd_active,
       t.speed_l_mms, t.speed_r_mms, t.odometer_cm, t.heading_enc_cw_deg, t.cruise_set_mms,
       t.ir_left, t.ir_centre, t.ir_right, csc14.lbl('line_pos_phys', t.line_pos_phys) AS line_pos_label,
       csc14.lbl('line_state', t.line_state) AS line_state_label, t.line_state, t.steer,
       csc14.lbl('bc_state', t.bc_state) AS bc_state_label, t.bc_symbol,
       csc14.lbl('bc_result', t.bc_result) AS bc_result_label, csc14.lbl('nav', t.nav) AS nav_label,
       t.nav_valid, t.bc_valid_mod64, t.bc_invalid_mod64,
       csc14.lbl('imu_status', t.imu_status) AS imu_status_label, t.imu_status, t.pitch_deg, t.roll_deg,
       t.yaw_rate_ccw_dps, t.heading_imu_ccw_deg, csc14.lbl('motion_class', t.motion_class) AS motion_class_label,
       t.tilt, t.collision, t.shock_g, t.hump_last_mm, t.hump_max_boot_mm, t.hump_count, t.hump_active,
       t.us_dist_mm, t.us_no_echo, csc14.lbl('us_result', t.us_result) AS us_result_label, t.us_context,
       t.us_forward_mm, t.avoid_state, csc14.lbl('avoid_state', t.avoid_state) AS avoid_state_label,
       csc14.lbl('avoid_side', t.avoid_side) AS avoid_side_label,
       csc14.lbl('avoid_outcome', t.avoid_outcome) AS avoid_outcome_label, t.profile_reported,
       t.obst_width_mm, t.obst_bearing_deg, t.clear_left_mm, t.clear_left_at_least, t.clear_right_mm,
       t.clear_right_at_least, csc14.lbl('comm_state', t.comm_state) AS comm_state_label,
       t.tlm_dropped, t.rx_rejected
FROM csc14.telemetry t;

-- Run summary.  Everything is computed from what the backend RECEIVED
-- during the run; 'notes' lists why a figure may be incomplete.
CREATE FUNCTION csc14.run_summary(p_run uuid)
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
	SELECT tt.*, lag(tt.received_at) OVER w AS prev_rx, lag(tt.bc_valid_mod64) OVER w AS prev_v,
	       lag(tt.bc_invalid_mod64) OVER w AS prev_i
	FROM csc14.telemetry tt WHERE tt.run_id = p_run
	WINDOW w AS (PARTITION BY tt.boot_id ORDER BY tt.received_at, tt.id)
), tg AS (
	SELECT count(*) AS n, coalesce(sum(seq_gap), 0) AS missing,
	       count(*) FILTER (WHERE seq_gap IS NULL) AS unknown_gaps,
	       coalesce(max(extract(epoch FROM received_at - prev_rx)), 0) AS max_gap,
	       count(DISTINCT boot_id) AS boots,
	       coalesce(sum((bc_valid_mod64 - prev_v + 64) % 64), 0) AS bc_v,
	       coalesce(sum((bc_invalid_mod64 - prev_i + 64) % 64), 0) AS bc_i,
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
	       count(*) FILTER (WHERE kind = 'AVOID_FAILED'
				   OR (kind = 'OBSTACLE' AND data->>'outcome' = '2')) AS av_fail,
	       count(*) FILTER (WHERE kind = 'COLLISION') AS coll,
	       count(*) FILTER (WHERE kind IN ('LINK_STALE', 'LINK_OFFLINE')) AS outages,
	       count(*) FILTER (WHERE kind = 'EVENTS_MISSED') AS ev_missed,
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
       round(dist.cm / 100.0, 2), h.n, h.no_height, h.max_mm, tg.bc_v, tg.bc_i,
       ev.av_ok, ev.av_fail, ev.coll, coalesce(ev.faults, '{}'), ev.outages,
       array_remove(ARRAY[
	CASE WHEN tg.n = 0 THEN 'no telemetry received during the run' END,
	CASE WHEN tg.hump_saturated THEN 'robot hump counter saturated at 15: a hump as high as the previous one is not detectable' END,
	CASE WHEN h.no_height > 0 THEN h.no_height || ' hump(s) completed without a received height' END,
	CASE WHEN (SELECT hump_active FROM last_t) THEN 'a hump was being measured when the run ended' END,
	CASE WHEN tg.missing > 0 OR tg.unknown_gaps > tg.boots THEN 'telemetry frames missing: brief changes and events may be absent' END,
	CASE WHEN ev.ev_missed > 0 THEN 'robot event frames missing' END,
	CASE WHEN b.restarts > 0 THEN 'robot restarted during the run' END,
	CASE WHEN tg.odo_saturated THEN 'odometer saturated (655 m): distance is a lower bound' END,
	CASE WHEN r.ended_at IS NULL THEN 'run still active' END
       ], NULL)
FROM r, tg, dist, h, ev, b
$$;

CREATE VIEW csc14.v_run AS
SELECT s.* FROM csc14.run r CROSS JOIN LATERAL csc14.run_summary(r.run_id) s;
