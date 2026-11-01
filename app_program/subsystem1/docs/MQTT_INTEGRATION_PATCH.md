# MQTT transport hook-up: change to a protected file

**Status: applied** in commit `05a675c` (2026-10-28):
`lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c` calls the four bridge
functions below. This document is kept as the record of the change; the
current file differs from the diff in comments only. Steps 3-6 of
"How to verify" still need the robot and are listed again in
`telemetry/docs/FIRMWARE.md` (live verification).

Buddy 1's firmware encodes, queues, validates and acknowledges everything
inside `app_program/subsystem1/`. Getting frames onto the network, however,
needs a few lines in **`lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c`**.
That file is professor-provided and outside the subsystem folders, so it was
not modified. This document gives the exact change for the team (and the
professor) to agree and apply.

## Why the change has to be there

* `TEAM.md` §3: *"No student task may call `cyw43_...`, lwIP or MQTT
  functions … A 'cloud / reporting' subsystem would add a kernel object
  (e.g. a message buffer) that application tasks fill. The MQTT code then
  reads it with `TMO_POL` (never blocking) and publishes it."*
* lwIP runs with `NO_SYS=1` on the single radio task (`cyw43_utk_task`).
  Every `mqtt_publish()` must therefore be called from that task, which
  means from `mqtt_utk.c`.
* The current `mqtt_utk.c` is a demo. It publishes `"hb N"` text on
  `MQTT_TOPIC` every 5 s, subscribes to the same topic, and **replaces every
  non-printable byte of a received payload with `'.'`**, which would corrupt
  binary command frames.

## What Buddy 1 already provides (in `subsystem1/`, tested)

`mqtt_bridge.h` declares four non-blocking functions with plain
`<stdint.h>` types, meant to be called only from `cyw43_utk_task`:

| Function | Purpose |
|---|---|
| `int32_t s1_mqtt_take_tx(uint8_t *buf, int32_t cap, const char **topic)` | next frame to publish and its topic; 0 = nothing queued (`TMO_POL`) |
| `int32_t s1_mqtt_put_rx(const char *topic, const uint8_t *data, int32_t len)` | a received payload; only `csc14/cmd` is accepted; never blocks |
| `void s1_mqtt_session(int32_t state)` | session state for Buddy 1's connection state machine |
| `int32_t s1_mqtt_reconnect_requested(void)` | 1 once when Buddy 1's RECOVERY state asks for a fresh session |

The `TEST=1` image exercises them on the Pico without a broker:

* **S1-U24/U25:** command frames in, ACK frames out, rejection of a wrong
  topic, an empty frame and a corrupted frame.
* **S1-U26:** the drop-oldest queue.

## The change (unified diff against the current file)

```diff
--- a/lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c
+++ b/lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c
@@
 #include "lwip/apps/mqtt.h"
 #include "lwip/ip_addr.h"
 #include "lwip/timeouts.h"
 #include "wifi_secrets.h"
 #include "mqtt_utk.h"
+#include "../../../../app_program/subsystem1/mqtt_bridge.h"	/* stdint only */
 #include <tm/tmonitor.h>
 
-#define MQTT_UTK_HEARTBEAT_MS   5000
 #define MQTT_UTK_KEEPALIVE_S    60
 #define MQTT_UTK_RETRY_MS       3000
+#define MQTT_UTK_BRIDGE_MS      50	/* poll Buddy 1's transmit queue */
+#define MQTT_UTK_BRIDGE_BURST   4	/* frames published per poll */
@@
 static char		rx_topic[MQTT_UTK_RX_TOPIC_MAX];
 static UB		rx_data[MQTT_UTK_RX_DATA_MAX];
+static uint8_t		rx_raw[MQTT_UTK_RX_DATA_MAX];	/* binary, for Buddy 1 */
 static u16_t		rx_len;
 static u8_t		rx_truncated;
 
-static void heartbeat_timeout(void *arg);
+static void bridge_timeout(void *arg);
@@ static void incoming_data_cb(void *arg, const u8_t *data, u16_t len, u8_t flags)
 		if(rx_len < (MQTT_UTK_RX_DATA_MAX - 1)) {
 			UB c = data[i];
+			rx_raw[rx_len] = c;			/* keep the real bytes */
 			rx_data[rx_len++] = (c >= 0x20 && c < 0x7f)? c: '.';
 		} else {
 			rx_truncated = 1;
 		}
 	}
 
 	if((flags & MQTT_DATA_FLAG_LAST) != 0) {
 		rx_data[rx_len] = '\0';
-		tm_printf((UB*)"mqtt: RX [%s] \"%s\"%s\n", rx_topic, rx_data,
-			  rx_truncated? " (truncated)": "");
+		if(!rx_truncated) {
+			(void)s1_mqtt_put_rx(rx_topic, rx_raw, (int32_t)rx_len);
+		}
 	}
 }
 
 static void subscribe_done_cb(void *arg, err_t result)
 {
 	(void)arg;
 	if(result == ERR_OK) {
-		tm_printf((UB*)"mqtt: subscribed to " MQTT_TOPIC "\n");
+		s1_mqtt_session(S1_SESSION_SUBSCRIBED);
+		tm_printf((UB*)"mqtt: subscribed to " S1_TOPIC_CMD "\n");
 	} else {
 		tm_printf((UB*)"mqtt: subscribe failed, err=%d\n", (int)result);
 	}
 }
@@
-static void heartbeat_timeout(void *arg)
-{
-	... (demo "hb N" publisher removed: Buddy 1 sends csc14/hb) ...
-}
+/* Publish what Buddy 1 queued; runs inside sys_check_timeouts() on the
+   radio task, the only task allowed to call lwIP. */
+static void bridge_timeout(void *arg)
+{
+	uint8_t		f[S1_MQTT_FRAME_MAX];
+	const char	*topic;
+	int32_t		n;
+	int		k;
+
+	(void)arg;
+	if(s1_mqtt_reconnect_requested() && client != NULL) {
+		mqtt_disconnect(client);		/* no callback (lwIP) */
+		s1_mqtt_session(S1_SESSION_LOST);
+		return;					/* poll loop reconnects */
+	}
+	for(k = 0; k < MQTT_UTK_BRIDGE_BURST && mqtt_client_is_connected(client); k++) {
+		n = s1_mqtt_take_tx(f, (int32_t)sizeof(f), &topic);
+		if(n <= 0) {
+			break;
+		}
+		if(mqtt_publish(client, topic, f, (u16_t)n, 0 /* qos */, 0 /* retain */,
+				publish_done_cb, NULL) != ERR_OK) {
+			break;		/* output ring full: frame lost, next poll goes on */
+		}
+	}
+	sys_timeout(MQTT_UTK_BRIDGE_MS, bridge_timeout, NULL);
+}
@@ static void connection_cb(mqtt_client_t *c, void *arg, mqtt_connection_status_t status)
 	if(status == MQTT_CONNECT_ACCEPTED) {
 		err_t	err;
 
 		connecting = 0;
+		s1_mqtt_session(S1_SESSION_CONNECTED);
 		tm_printf((UB*)"mqtt: connected to " MQTT_BROKER_IP "\n");
 		mqtt_set_inpub_callback(client, incoming_publish_cb,
 					incoming_data_cb, NULL);
-		err = mqtt_subscribe(client, MQTT_TOPIC, 0 /* qos */,
+		err = mqtt_subscribe(client, S1_TOPIC_CMD, 0 /* qos */,
 				     subscribe_done_cb, NULL);
 		...
-		sys_untimeout(heartbeat_timeout, NULL);
-		sys_timeout(MQTT_UTK_HEARTBEAT_MS, heartbeat_timeout, NULL);
+		sys_untimeout(bridge_timeout, NULL);
+		sys_timeout(MQTT_UTK_BRIDGE_MS, bridge_timeout, NULL);
 	} else {
+		s1_mqtt_session(S1_SESSION_LOST);
 		tm_printf((UB*)"mqtt: connection status %d, retrying in %ds\n",
 			  (int)status, MQTT_UTK_RETRY_MS / 1000);
 		sys_timeout(MQTT_UTK_RETRY_MS, retry_timeout, NULL);
 	}
```

Notes on the change:

* **No makefile change.** The quoted relative include resolves from the
  directory of `mqtt_utk.c`, and `mqtt_bridge.c` is already compiled with
  `app_program`. The bridge functions are already in every image (in the
  normal image the linker drops them as unused until this patch calls them).
* **Timer budget unchanged.** `bridge_timeout` replaces the demo heartbeat
  chain one for one (`MEMP_NUM_SYS_TIMEOUT` already has +4).
* **Never blocks.** Every bridge call uses `TMO_POL` or a single atomic word.
  A full Buddy 1 queue or a full lwIP output ring loses a frame; it never
  stalls the radio task. Telemetry is non-critical (NFR9). Commands are
  idempotent supervisory requests.
* **Topics** (`S1_TOPIC_*`) are fixed in `mqtt_bridge.h`. `MQTT_TOPIC` from
  `wifi_secrets.h` is then no longer used. `MQTT_BROKER_IP`, `MQTT_BROKER_PORT`
  and `MQTT_CLIENT_ID` still are.
* **TEAM.md impact.** Section 3's rule stays intact: only `cyw43_utk_task`
  calls lwIP. Buddy 1's tasks only fill and drain kernel message buffers.

## How to verify once applied (hardware + broker needed)

1. `cp config/wifi_secrets.example.h config/wifi_secrets.h` and fill in the
   SSID, password and broker IP. **Do not commit it** (it is gitignored).
2. `cd build_make && make NET=lwip MQTT=1 -j8`, then flash.
3. On the broker host: `mosquitto_sub -h <broker> -t 'csc14/#' -v` should
   show `csc14/tlm` twice a second and `csc14/hb` once a second, as binary.
4. Node-RED: `mqtt in` (topic `csc14/#`, output "a Buffer") → Function node
   containing `nodered_decoder.js` → debug. The decoded objects should match
   the console (`S1_CONSOLE_TELEMETRY` in `subsystem1.h` prints hex frames
   when set to N > 0).
5. Send a PING: `node -e "process.stdout.write(require('./nodered_decoder.js')
   .encodeCommand('PING',0,1,1))" | mosquitto_pub -h <broker> -t csc14/cmd -s`.
   Expect an ACK OK on `csc14/ack`.
6. Disconnect the access point for 30 s while the car runs on the bench.
   Expected:
   * Telemetry shows `WIFI_LOSS`, then `RETRY_WAIT` with growing delays.
   * The heartbeat reports `RECOVERING` after reconnection.
   * Motion Control's period (S2-U42 numbers in the test image) is unchanged.
