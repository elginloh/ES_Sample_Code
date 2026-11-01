# Deploying the telemetry backend on a Raspberry Pi

> **Not yet tried on a Raspberry Pi.** These steps use the same commands
> that were tested on the Mac (`bin/csc14 setup`, `serve`, the integration
> tests), plus Debian packages and systemd. Run `bin/csc14 test all` on the
> Pi after installing.

Target: Raspberry Pi 4 or 5, **Raspberry Pi OS 64-bit (Bookworm or later)**.
Measured backend footprint on the Mac (`docs/MEASUREMENTS.md`): Node-RED
up to about 120 MB, PostgreSQL about 50 MB, Grafana up to about 330 MB
while the dashboard is open; ingestion uses under 1 % of a core at 2 Hz,
Grafana about 5 % while the dashboard refreshes. A Pi is slower per core,
so expect higher percentages there (not measured). A Pi 4 with 2 GB or more
is comfortable; allow 2 GB of disk (Grafana alone is about 450 MB unpacked)
and about 17 MiB of database per hour of robot operation.

## 1. System packages

```sh
sudo apt update
sudo apt install -y mosquitto mosquitto-clients postgresql git curl
# Node.js >= 22.9 (Node-RED 5). Bookworm's own nodejs is too old:
curl -fsSL https://deb.nodesource.com/setup_22.x -o /tmp/nodesource.sh   # read it first
sudo -E bash /tmp/nodesource.sh && sudo apt install -y nodejs
node --version
```

## 2. Broker

```sh
sudo cp telemetry/deploy/pi/mosquitto-csc14.conf /etc/mosquitto/conf.d/csc14.conf
sudo systemctl restart mosquitto
```

It allows anonymous clients on port 1883, because the Pico has no MQTT
user name or password. Keep the Pi on a private network.

## 3. PostgreSQL account for the setup

Debian's PostgreSQL does not trust local TCP logins, so give the setup an
administrator account (only `setup`/`migrate` use it):

```sh
sudo -u postgres psql -c "CREATE ROLE csc14_admin LOGIN CREATEDB CREATEROLE PASSWORD 'choose-one';"
```

## 4. Backend

```sh
git clone <your repository> ~/ES_Sample_Code
cd ~/ES_Sample_Code/telemetry
bin/csc14 setup            # creates .env, downloads Grafana (linux arm64), creates the database
nano .env                  # set:
                           #   CSC14_PG_ADMIN_USER=csc14_admin
                           #   CSC14_PG_ADMIN_PASSWORD=choose-one
                           #   CSC14_BROKER_CONF=            (empty: Mosquitto is a system service)
                           #   CSC14_PG_DATA=                (empty: PostgreSQL is a system service)
                           #   CSC14_GRAFANA_HOST=0.0.0.0    (to open the dashboard from a laptop)
bin/csc14 setup            # again, now with the admin account: creates roles and tables
sudo deploy/pi/install-services.sh   # Node-RED + Grafana as systemd services, restart on failure
bin/csc14 status
bin/csc14 test all         # unit + isolated integration tests on the Pi
```

Dashboard: `http://<pi-address>:3000/d/csc14-robot`.

## 5. Point the robot at the Pi

The Pico connects to `MQTT_BROKER_IP` in `config/wifi_secrets.h`. If the
backend moves from the Mac to the Pi, set it to the Pi's address and rebuild
the firmware (`make NET=lwip MQTT=1 -j8`). Nothing in `telemetry/` changes
that file.

## Undo

```sh
sudo telemetry/deploy/pi/install-services.sh --remove
sudo rm /etc/mosquitto/conf.d/csc14.conf && sudo systemctl restart mosquitto
# the database stays; to delete it on purpose:
#   sudo -u postgres dropdb csc14_telemetry
```
