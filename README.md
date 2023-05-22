# araf-raspberrypi




# Installing services

copy services from ./scripts/services/ to /etc/systemd/system/
```bash
sudo systemctl daemon-reload
sudo systemctl enable araf 
sudo systemctl enable mavproxy 
sudo systemctl start araf 
sudo systemctl start mavproxy 
```

