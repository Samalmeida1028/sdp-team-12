#!/bin/bash

bluetoothctl disconnect 64:72:D8:49:95:D9
bluetoothctl connect 64:72:D8:49:95:D9
bt-device -l
