#!/bin/sh
ouster_ip=$(avahi-browse -lrt _roger._tcp | grep address | grep "[0-9]")
echo $ouster_ip
