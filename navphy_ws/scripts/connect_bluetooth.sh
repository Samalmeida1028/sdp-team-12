#!/bin/bash

bluetoothctl disconnect 64:72:D8:49:95:D9
bluetoothctl disconnect 48:A5:E7:7F:84:40

bluetoothctl connect 64:72:D8:49:95:D9
bluetoothctl connect 48:A5:E7:7F:84:40
bt-device -l
