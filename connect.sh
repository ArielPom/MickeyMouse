#!/bin/bash

RPI_IP="ENTER_IP"
sshpass -p 'ENTER_PASS' ssh -o StrictHostKeyChecking=no  pi@${RPI_IP}
