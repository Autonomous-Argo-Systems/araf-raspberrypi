# araf-raspberrypi

# Setting up dependencies

Run:
```bash
./script/setup/setup.sh
```

# Installing services

copy services from ./scripts/services/ to /etc/systemd/system/
```bash
sudo systemctl daemon-reload
sudo systemctl enable araf 
sudo systemctl enable mavproxy 
sudo systemctl start araf 
sudo systemctl start mavproxy 
```

# Deploying code to araf

Start wifi hotspot:
SSID: araf
Password: arafaraf

Wait for RaspberryPI to connect

Run:
```bash
./script/deploy/deploy.sh
```

# Connecting to the araf

Start wifi hotspot:
SSID: araf
Password: arafaraf

Wait for RaspberryPI to connect

Run:
```bash
ssh araf@araf.local
```
If araf.local can't be found, use ip address of araf found in wifi hotspot settings
