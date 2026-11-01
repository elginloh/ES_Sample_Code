# Telemetry backend measurements (2026-10-30T05:51:04.432Z)

Host: Darwin 27.0.0 arm64, 10 CPUs, 16 GB, Node.js v26.3.1. Isolated services, simulated frames (lib/sim.js).
MEASURED on this computer; not evidence about the Pico, its Wi-Fi or its control loops.

## idle

| service | CPU (avg) | RSS (end) | RSS (peak) |
|---|---|---|---|
| Node-RED | 0.2 % | 19.2 MB | 119.0 MB |
| PostgreSQL | 0.1 % | 19.4 MB | 29.8 MB |
| Mosquitto | 0.0 % | 1.1 MB | 2.2 MB |
| Grafana | 0.7 % | 26.0 MB | 293.4 MB |

## normal

Rows: 236. Publish -> received by Node-RED: p50 1.0 ms, p95 4.0 ms, max 41.0 ms. Received -> row committed: p50 2.4 ms, p95 7.0 ms, max 905.0 ms.

Frames published 373 (10692 payload bytes, 89.1 B/s). Rows: telemetry 236, heartbeat 118, events 32.
Storage: 576 KiB added in 120 s (telemetry+heartbeat+event incl. indexes) = 16.9 MiB per hour of robot operation. Average telemetry row 1478.0 B (frame 41.0 B, decoded JSONB 1166.5 B).

| service | CPU (avg) | RSS (end) | RSS (peak) |
|---|---|---|---|
| Node-RED | 0.7 % | 37.8 MB | 71.8 MB |
| PostgreSQL | 0.3 % | 30.0 MB | 32.0 MB |
| Mosquitto | 0.0 % | 1.5 MB | 1.5 MB |
| Grafana | 0.9 % | 55.0 MB | 173.4 MB |

## dashboard

420 panel queries (0 failed), every panel every 5 s: p50 7.0 ms, p95 14.5 ms, max 80.4 ms. 6-hour range: p50 6.0 ms, max 11.5 ms (0 failed).

| service | CPU (avg) | RSS (end) | RSS (peak) |
|---|---|---|---|
| Node-RED | 0.4 % | 37.7 MB | 45.5 MB |
| PostgreSQL | 0.9 % | 44.9 MB | 55.1 MB |
| Mosquitto | 0.0 % | 1.5 MB | 1.5 MB |
| Grafana | 5.0 % | 324.0 MB | 323.9 MB |

## stress

Rows: 600. Publish -> received by Node-RED: p50 2.0 ms, p95 4.0 ms, max 9.0 ms. Received -> row committed: p50 3.7 ms, p95 6.4 ms, max 20.8 ms.

| service | CPU (avg) | RSS (end) | RSS (peak) |
|---|---|---|---|
| Node-RED | 1.4 % | 70.5 MB | 80.1 MB |
| PostgreSQL | 0.5 % | 38.8 MB | 46.5 MB |
| Mosquitto | 0.0 % | 1.5 MB | 1.5 MB |
| Grafana | 0.6 % | 46.9 MB | 324.0 MB |

## db_outage

Outage 30 s: peak buffer 93 bulk + 13 priority rows; drained 2633 ms after the database returned; telemetry rows kept 60; dropped 0 bulk / 0 priority.

## broker_outage

Broker down 15 s: Node-RED noticed after 8 ms, reconnected 4934 ms after the broker returned. Frames published in the phase 64, telemetry rows added 10 (frames sent while the broker was down are lost: QoS 0, no queue, as on the Pico). LINK_UNKNOWN events: 1.

