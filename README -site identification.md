Site Identification Module (Site/Marker Detection)

This module serves as the core component of the "Parcel Sorting and Line-following Cart" project. It is responsible for providing a reliable stop/docking trigger signal when the cart arrives at designated sorting points or loading/unloading points.

1.  Functional Overview
To meet project requirements, site identification employs a "Vision + Physical" dual-verification mechanism to achieve low-risk docking:

Visual Identification: Utilizes a Raspberry Pi camera to detect specific Color Blocks or markers on the ground.

Physical Compensation: Utilizes a side-mounted Limit Switch to ensure precise docking even in the event of visual system failure.

2.  Hardware Environment
Controller: Raspberry Pi 

Sensors:

Raspberry Pi Official Camera (CSI/USB interface)
Side-mounted Tactile / Limit Switch (GPIO connection)
Ground Markers: Recommended to use green/red reflective tape or specific colored patches.

3.  Technical Implementation
3.1 Visual Detection Logic
Based on OpenCV (C++ version) for Color Space Conversion (BGR to HSV). The HSV model offers stronger robustness against lighting variations compared to RGB.

Core Steps:

Color Space Conversion: Convert the captured frame to HSV.

Masking: Filter for the target color using the inRange function.

Morphological Filtering: Use Erode and Dilate operations to remove image noise and prevent false triggers.

Area Judgment: Calculate the area of white pixels within the mask. A site is considered reached when the area > Threshold.

3.2 Physical Trigger Logic
As a "low-risk" fallback scheme, the code integrates GPIO interrupt or polling detection:

Pin Configuration: Use the wiringPi library to set the pin as a pull-up input (PUD_UP).

Logic Determination: Immediately trigger the signal when the switch contacts a stopper, generating a low-level signal.