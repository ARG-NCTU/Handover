#!/bin/bash



if [ ! "$1" ]; then
    echo "Please specify your network interface. (Check through ifconfig)"
    return
fi
echo "Multicast Interface: $1"

MULTICAST_INTERFACE=$1

ifconfig $MULTICAST_INTERFACE multicast
route add -net 224.0.0.0 netmask 240.0.0.0 dev $MULTICAST_INTERFACE
export LCM_DEFAULT_URL=udpm://238.255.76.67:7667?ttl=1
