#!/usr/bin/env bash
#
# install-services.sh -- Raspberry Pi OS (64-bit) / Debian: run Node-RED and
# Grafana of the CSC14 telemetry backend as systemd services that start at
# boot and restart after a crash.  Run AFTER 'bin/csc14 setup' succeeded:
#
#   sudo telemetry/deploy/pi/install-services.sh            install + start
#   sudo telemetry/deploy/pi/install-services.sh --remove   stop + remove
#
# Mosquitto and PostgreSQL are the distribution's own services (apt); see
# deploy/pi/README.md.  Nothing else on the system is changed.
set -euo pipefail

[ "$(id -u)" -eq 0 ] || { echo "run with sudo" >&2; exit 1; }
TELEMETRY="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
RUN_AS="${SUDO_USER:-pi}"
UNITS=/etc/systemd/system

if [ "${1:-}" = "--remove" ]; then
	systemctl disable --now csc14-grafana.service csc14-nodered.service 2>/dev/null || true
	rm -f "$UNITS/csc14-grafana.service" "$UNITS/csc14-nodered.service"
	systemctl daemon-reload
	echo "removed csc14-nodered and csc14-grafana"
	exit 0
fi

[ -f "$TELEMETRY/.env" ] || { echo "telemetry/.env missing: run bin/csc14 setup first" >&2; exit 1; }

for name in nodered grafana; do
	sed -e "s|@TELEMETRY@|$TELEMETRY|g" -e "s|@USER@|$RUN_AS|g" \
		"$TELEMETRY/deploy/pi/csc14-$name.service" > "$UNITS/csc14-$name.service"
done
systemctl daemon-reload
systemctl enable --now csc14-nodered.service csc14-grafana.service
echo "installed: systemctl status csc14-nodered csc14-grafana; logs: journalctl -u csc14-nodered -f"
