#!/bin/sh
rmmod pitdm
rmmod pidma
insmod pidma.ko
insmod pitdm.ko debug=0
dahdi_cfg
