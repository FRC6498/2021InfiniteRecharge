#!/bin/bash

scp admin@roborio-6498-frc.local:/home/lvuser/HOOD-LOGS.csv . && py -3 plot_data.py
read
