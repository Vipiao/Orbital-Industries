#!/usr/bin/env bash
# Local two-peer test: a server, then a client on localhost two seconds later.
cd "$(dirname "$0")/bin"
printf 's\n\n' | ./OrbitalIndustries &
sleep 2
printf 'c\n\n\n' | ./OrbitalIndustries
wait
