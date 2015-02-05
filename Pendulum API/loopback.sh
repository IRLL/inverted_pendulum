#!/bin/bash

#creates two ports that are connected to each other for testing
socat -d -d pty,raw,echo=0 pty,raw,echo=0
