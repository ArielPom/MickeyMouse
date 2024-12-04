#!/bin/bash

service_file="mediamtx.service"
chmod +x "run_stream.sh"

sudo cp "$service_file" /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable "$service_file"
sudo systemctl start "$service_file"