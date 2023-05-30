# araf-raspberrypi

## Setting up dependencies

Run:
```bash
./script/setup/setup.sh
```

## Deploying code to araf

| Start wifi hotspot |-|
|----------|-----------|
| SSID     | araf      |
| Password | arafaraf  |

Wait for RaspberryPI to connect

Run:
```bash
./script/deploy/deploy.sh
```

## Connecting to the araf

| Start wifi hotspot |-|
|----------|-----------|
| SSID     | araf      |
| Password | arafaraf  |

Wait for RaspberryPI to connect

Run:
```bash
ssh araf@araf.local
```
If araf.local can't be found, use ip address of araf found in wifi hotspot settings

## Services

The software has two services to operate. These are the `araf` and `mavproxy` service.

### Installing services

copy services from ./scripts/services/ to /etc/systemd/system/
```bash
rsync ./scripts/services/ araf@araf.local:/etc/systemd/system/ --chown=root:root --progress
```

Then reload the daemon and enable/start the services.
```bash
sudo systemctl daemon-reload
sudo systemctl enable araf 
sudo systemctl enable mavproxy 
sudo systemctl start araf 
sudo systemctl start mavproxy 
```

### ARAF service
The `araf` service is the main service, this service is used to run the ros master and nodes.

This service runs the `startROS.sh` script on startup.

#### *Possible issues*
The `mavproxy` service being offline could cause the `araf` service to keep restart due to no connecting to PX4.

### MavProxy service
The `mavproxy` service is the service that is used to start and restart mavproxy.

This service runs the `startMavProxy.sh` script on startup.

#### *Possible issues*
When Mission planner is connected to mavproxy and connection is lost, to reconnect mavproxy might need to be restarted.

## Debugging
Debugging can be done by viewing to logs with:
```bash
journalctl -u araf --no-pager
```
And
```bash
journalctl -u mavproxy --no-pager
```

Further debugging can be done by launching `ROS` manually.

Debugging of individual ros topics can be done by logging them with
```bash
rostopic echo name_of_topic
```
