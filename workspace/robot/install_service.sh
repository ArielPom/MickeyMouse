#!/bin/bash

service_file="tof_reader.service"

sudo cp "$service_file" /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable "$service_file"
sudo systemctl restart "$service_file"