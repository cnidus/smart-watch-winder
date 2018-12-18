# smart-watch-winder
Gyroscopic smart watch winder w/ AWS IoT integration, computer vision skew detection and OLED display.

## Intro
This project is based on the [3d printed Gyro watch winder by NedalLive](https://cults3d.com/en/3d-model/fashion/gyro-winder-watch-winder-remontoir-montre). The intent is to wind an automatic mechanical watch, track each watch's accuracy over time and display the accuracy and winding statics on the onboard OLED display. Eventually, I'd like to display via a webapp as well.

To achieve this, a camera is mounted to the rear support frame to periodically take photos of the watch-face. These watch-face images are uploaded to AWS and processed via an computer-vision algorithm to determine the displayed time, the inferred time is then compared with NTP and stored as a statistic for each unique watch.

## Current State
Winder stepper control: Functional, complete.
IoT Winder stats publish: Functional, needs work..
IoT Winder stats subscribe: Broken, subscription works, but messages no longer displayed to console. Needs some attention.

## Next steps (TODO:)
1. Port/adapt [ESP32 Camera driver](https://github.com/espressif/esp32-camera) to arduino.
2. Design optimal path to upload the images. Probably a direct s3 object put & likely an API for metadata.
3. Build training data set from my collection of automatic watches.

## Hardware
Board: [Heltec ESP32 WiFiKit (ESP32 + OLED)](https://github.com/Heltec-Aaron-Lee/WiFi_Kit_series#instructions)
Camera: [ArduCam OV2640](http://www.arducam.com/camera-modules/2mp-ov2640/)
Stepper motor/controller: [28BYJ-48 + ULN2003 ](https://www.aliexpress.com/item/Frees-hipping-5V-4-Phase-Stepper-Step-Motor-Driver-Board-ULN2003-with-drive-Test-Module-Machinery/32706559510.html)
Ribbon extension: [24pin FPC extension](https://www.aliexpress.com/item/0-5-Type-A-250mm-24P-Flexible-Flat-Cable-Extension-Adapter-24Pin-FPC-TTL-Extend-cord/32811483374.html)
Glass Dome: [Ikea BEGÅVNING Glass dome with base](https://www.ikea.com/us/en/catalog/products/50343939/?query=BEG%C3%85VNING&icid=iba|us|unbxdsuggestion|201811132211556208_1)
Gyro parts: [3d printed Gyro watch winder by NedalLive](https://cults3d.com/en/3d-model/fashion/gyro-winder-watch-winder-remontoir-montre)

I also have a couple of [AI-Thinker ESP32-Cam](https://www.aliexpress.com/item/ESP32-CAM-WiFi-Bluetooth-Module-Camera-Module-Development-Board-ESP32-with-Camera-Module-OV2640-2MP/32919183232.html) boards I plan to test with. This package dramatically simplifies wiring, as the camera is integrated. It will also allow for additional remote mounting locations of the OLED display, since the OLED is more compact when not mounted to the

Both camera modules have zif/fpc ribbon cables, so I'm hoping to be able to extend this remote, so the camera can remain in the rear support arm and the hide the rest of electronics in the base. Not sure yet if this will work as the cam modules normally require short cable runs

## How it Works
### NTP
I found an [NTP Clock instructables](https://www.instructables.com/id/WiFi-Kit-32-NTP-Clock/) project that conveniently used the same board. This proved
### Driving the OLED display
### Winder & stepper control

### Winder stats to AWS IoT core
IoT integration is loosely based on the tutorial [AWS IoT for ESP32](https://exploreembedded.com/wiki/AWS_IOT_with_Arduino_ESP32) by ExploreEmbedded. Currently the Subscription is successful, but the handler doesnt seem to run. This happened after I sped up my main loop, so I suspect it may be a timing / resource issue, will troubleshoot later.

### Computer Vision
A core component to this project is the ability to read time from an analogue watch. I've just started research into this area, since I'll need to get a set of images from the camera to build a training set first.

Ultimately the project needs an ML algorithm for 2 core uses:
1. Fingerprint each individual watchface
2. Read displayed time from the analogue watchface

One of the goals is to attribute stats to each watch, since its likely people will use the winder with multiple watches, there needs to be a mechanism for selecting each watch when it is added. I'd like to avoid mandatory user interaction, so the system needs to fingerprint the watchface, this fingerprint can then be used as an ID to attribute stats to.

There has been some work in the industry in this space I'll be building from.
[Analogue clock and watch reader](https://www.cs.bgu.ac.il/~ben-shahar/Teaching/Computational-Vision/StudentProjects/ICBV151/ICBV-2015-1-ChemiShumacher/Report.pdf)
[DeepTime](https://felixduvallet.github.io/blog/deep.time/)

## Notes


##Other References (no particular order)
[Arduino using ESP32 IDF modules](https://medium.com/home-wireless/how-to-program-an-esp32-in-arduino-while-using-esp-idf-functions-90033d860f75)
