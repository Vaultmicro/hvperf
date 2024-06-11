#!/bin/bash
sudo modprobe gadgetfs default_uid=1000 default_gid=1000
sudo mkdir -p /dev/gadget
sudo mountpoint -q /dev/gadget || sudo mount -t gadgetfs gadgetfs /dev/gadget

