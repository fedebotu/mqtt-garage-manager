# MQTT Garage Manager

## Functions
This is a ESP8266 custom module project I made for managing my garage with Home Assistant. A summary of the main functions are the following:
* Controlling the garage door via 433MHz and feedback loop with a Reed sensor
* Detecting the car presence with ultrasonic sensors
* Movement sensor for person presence
* MQTT connection to local broker
* Telnet debugger
* Web Updater (OTA updates)

## Other info (to be updated! Coming soon ;))
I will also include more information in the future about how it works and how to integrate it with Home Assistant!
Besides, the garage door was automated without installing new stuff on the motor: it is controlled via a 433 MHz module. The status of the door (open, close) is read via a reed sensor. I have built a simple sniffer for catching the door code; if I'm not lazy, I'll upload it too ;)

## Wiring diagram in Fritzing

<p align="center">
  <img src="https://github.com/Juju-botu/mqtt-garage-manager/blob/main/Images/garage_module_HW_v0.1_bb.png" height = 600>
</p>
