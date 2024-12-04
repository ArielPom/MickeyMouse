#!/bin/bash

ssh_pass="ENTER_PASS"
pi_ip="ENTER_IP"
destination_folder="pi@$pi_ip:~/"

# Use rsync to copy the folder to the Pi
sshpass -p "$ssh_pass" rsync -avz ./workspace "$destination_folder"

exit 