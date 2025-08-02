# ConfuSense: Sensor Reconfiguration Attacks for Stealthy UAV Manipulation
Artifact related to the paper "ConfuSense: Sensor Reconfiguration Attacks for Stealthy UAV Manipulation" in the Proceedings of the 2025 USENIX Symposium on Vehicle Security and Privacy (VehicleSec '25)
## ardupilot folder 
contains the ardupilot modified firmware sources for hardware evaluation

## IEMI testbed folder 
contains the sources for IEMI injection on the I2C testbed
including  Figure 7 and Figure 12

## local attack testbed folder 
contains the sources for local injection on the I2C testbed 
including Figure 5

## crazyflie folder
contains the real-time visualization library of crazyflie sensor readings 

## crazyflie_firmware folder
contains the modified crazyflie firmware sources for hardware evaluation 
as seen in the video https://www.youtube.com/watch?v=7vgxqLVDyUA

## data_analysis folder 
contains the scripts and the data logs for the evaluation
including Figure 8, Table 2, Table 3, Table 4, Table 5, Figure 13

## detection_eval folder
contains the data and scripts for the comparison with prior work detectors
including the reulsts of Section 7.1

## attack synthesis folder
contains the code for the attack synthesis framework 
including Figure 10 and Figure 11


When using the code or the data please cite our work
```
@inproceedings{Erba2025ConfuSense,
title = {ConfuSense: Sensor Reconfiguration Attacks for Stealthy UAV Manipulation},
author = {Erba, Alessandro and Castellanos, John H. and Sihag, Shail and Zonouz, Saman and Tippenhauer, Nils Ole},
booktitle = {Proceedings of the 2025 USENIX Symposium on Vehicle Security and Privacy (VehicleSec '25)},
year = {2025},
publisher = {USENIX}
}
```
