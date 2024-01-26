# nRF70_WiFi_Camera_Demo
WiFi Camear demo based on nRF7002DK and Arducam Mega Camera.

# Hardware Prepare

## Arducam Mega Cameara
Link:

## nRF7002DK 
Link:
nRF7002DK IO voltage uses 1.8v by default.Arducam Mega Camear IO voltage only support 5v/3.3v.


# Firmware Prepare
## Install nRF Connect SDK(NCS) version 2.5.1

## Cherry pick ArduCAM Mega Zephyr driver
git remote add arducam https://github.com/ArduCAM/zephyr.git 
git fetch arducam
From https://github.com/ArduCAM/zephyr
 * [new branch]              add_arducam_mega_beta    -> arducam/add_arducam_mega_beta
 * [new branch]              add_arducam_mega_driver  -> arducam/add_arducam_mega_driver
 * [new branch]              add_full_featured_sample -> arducam/add_full_featured_sample
 * [new branch]              main                     -> arducam/main
 * [new branch]              update_video_controls    -> arducam/update_video_controls
git log arducam/add_arducam_mega_driver
git cherry-pick ce6334
git cherry-pick 8cb926
git cherry-pick 2db7d6
git cherry-pick 37d1b7
git remote remove arducam

## Build Application

The WiFi camear demo firmare refer to the following two samples.

# Test Setup

ToDo:
1. UDP Transciever
2. Enable CONFIG_WIFI_CREDENTIALS_SHELL to be able to select WiFi networks at runtime.
3. Input UDP server address at runtime.

