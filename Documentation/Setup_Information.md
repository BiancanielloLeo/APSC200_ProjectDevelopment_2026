



APSC 200 Project Development
Multi-Agent Differential Drive Robotic System

## Setup Information

## Leo Biancaniello
## Will Beveridge

## Queen’s University
Mathematics and Statistics Department

## Summer 2026




## Contents
Overview ............................................................................................................................................................ 3
1 Accessing Files ........................................................................................................................................... 4
2 Computer Setup .......................................................................................................................................... 4
2.1 Software Setup ................................................................................................................................... 4
2.1.1 Camera Calibration ......................................................................................................................... 4
2.2 Firmware Setup ................................................................................................................................... 4
3 Hardware Setup .......................................................................................................................................... 5
3.1 Editing Chassis Model ......................................................................................................................... 5
3.2 Chassis Printing .................................................................................................................................. 5
3.3 Chassis’ Assembly .............................................................................................................................. 6
3.3.1 Soldering Components .................................................................................................................... 6
3.3.2 Fitting Components ......................................................................................................................... 7
Appendix A: Testing ............................................................................................................................................ 9




## Overview
This document contains information regarding how to handle the system. The software to download on
the computer, the instructions for how to interact with the file system, and the assembly instructions for
the agents are included.
To run this system: A computer with Wi-Fi access, camera, USB-to-USB extension cable, router, micro-
USB data cable, and robot agents will be needed.
To run the system, mount the camera overhead using tape and connect the extension cord so that the
camera’s cable reaches the computer. Take the router and plug its power cord into the wall so that it can
run a wireless local access network. Take each agent to be used, plug the micro-USB cable into the USB-
to-UART port on the microcontroller and flash the agent with the firmware to be run. Then place each
agent inside the field-of-view of the camera and turn them on. Connect the computer to the routers Wi-Fi
network, navigate to tracker.py and run the file. A pop-up of the camera’s feed should appear and a
prompt to start the system will appear and the system can be run.





## 1 Accessing Files
The project information and files can be accessed from the git repository. The "Software" folder contains
all code to be ran on the computer, the "Firmware" folder contains all the code to be ran on the robots,
and the "Hardware" folder contains all the CAD models for the robots.
The git repository allows for files to be stored on the cloud and to be managed by multiple users.
To make changes to the project for the next season, make a fork of the repository to your GitHub account,
then clone the fork to the computer and place it in a convenient folder location. As changes are made,
they should be pushed to the fork to save them to the repository.
## 2 Computer Setup
To work on the project, an IDE to manage the Python code and an IDE to manage the ESP code for the
microcontroller will be needed.
## 2.1 Software Setup
To edit the software, install Python 3.10 or newer. Then open the computers command line and type “pip
install _____” one by one for the necessary Python libraries “numpy” and “opencv-contrib-python”.
Then install a Python compatible IDE, ideally VS Code and start editing or running the files.
## 2.1.1 Camera Calibration
To prepare the camera, open findFocus.py in the “Computer Vision” folder and run the script. Watch the
screen as the camera tries different focus settings and note the setting which makes the markers on the
agents most clear. Then copy that focus value into the other scripts.
If the screen does not show the camera feed, switch the camera input from 0 to 1 as if the computer has
a built-in camera, the script may try accessing the wrong one.
## 2.2 Firmware Setup
To edit the firmware, install the Arduino IDE 2.3.9 or newer. Go to “File”, “Preferences”, then paste the link
https://espressif.github.io/arduino-esp32/package_esp32_index.json into “Additional Board Manager
URLs”, then go to “Boards Manager” on the side bar and install “esp32” by Espressif Systems.
Then, to flash the microcontrollers, install the CP210x driver. Connect the microcontroller to the
computer using a micro-USB data cable and the USB-to-UART port on the microcontroller. Open the
“Device Manager” on the computer and check that the microcontroller comes up under COM ___ with
some number.
In the Arduino IDE, click “Tools”, then “Board”, then “esp32”, then select “ESP32S3 Dev Module”. Then
click the drop-down menu and select the COM port for the microcontroller. Go to “Tools” and enable
“USB CDC On Boot”.




## Figure 1: Arduino Board Dropdown
Press the “Upload” button to flash the microcontroller with the code and the Serial Monitor should open.
Set the baud rate to 115200 then outputs can be read.
## 3 Hardware Setup
To work on the robot hardware, install Fusion 360 and a 3D printing slicer that is compatible with an
accessible 3D printer.
## 3.1 Editing Chassis Model
The chassis model is stored as a “.f3d” file which stores the feature tree and model information so that
every part of the model can be edited. When editing the chassis, make well defined planes and
dimension all sketches so that no geometries shift unintentionally. Be cautious of how changes can
affect other areas of the part and label all the feature changes in the feature tree at the bottom of the
screen so that changes can be tracked.
When labelling the chassis, create a sketch on the front of the chassis and the underside of the roof.
Navigate to “Text” under the “Create” tab, then draw out a text box. Set the text to Bolded Gills Sans with
40mm letter height, then manually center the number. Save the sketch and make an extrude-cut 0.5mm
deep into the chassis.
## 3.2 Chassis Printing
To 3D print the chassis, go to the Fusion 360 CAD file for the chassis, export the chassis as a STEP file,
then import that file into a 3D printing slicer. The chassis should be printed with 1.75mm PETG as it is
heat resistant and won’t melt from the motor’s warmth, but PLA can be used for the roof as it is cheaper.




Before printing, a 3D printing “Temperature Tower” should be printed to calibrate the temperature of the
extruder to get the highest quality print.
When slicing the chassis for 3D printing, go to “Paint on Supports” and make it so that all of the surfaces
in the motor sleeve block supports. No supports can be generated inside of the motor sleeves as that will
affect the finish of the inside of the sleeve and thus the fit of the motor. They also may be impossible to
remove. After slicing, go to the layer-by-layer view and get a section view of the motor sleeve to check
that no supports are generated inside.
The chassis were printed in the SparQ laboratory in Mitchell Hall with Bambu Labs P2S 3D printers. They
were oriented so that they were printed number side down to minimize the need for supports and ensure
clean edges. The chassis were printed with the “0.12mm High Quality @BBL P2S” default settings and the
following changes:
- 0.2mm first layer height
## • 25% Gyroid Infill
- Tree Supports with 0.2mm Top Z Distance
## • Adjusted Temperature
## 3.3 Chassis’ Assembly
The components needed to assemble the chassis are:
- Two 400-Pin Breadboards
## • One Microcontroller
## • One Motor Driver
## • Two Motors
## • Two Wheels
## • One Voltage Regulator
## • One Battery Case
## • One Rocker Switch
- One 100KΩ + One 47KΩ Resistor
- One 47휇F Ceramic Capacitor + One 100 휇F Electrolytic Capacitor
- Multiple Colours of 22 AWG wire
- Multiple Sizes of Heat Shrink Tubing
- 100lb Cardstock Matte ArUco Markers
## 3.3.1 Soldering Components
Some of the Integrated Circuits may not have pre-soldered header pins so using 0.6mm solder, solder on
the header pins. To ensure safety during soldering, prepare a clean table in an open room. Set up the
soldering stand so that there is slack in the cord and the stand cannot be accidentally knocked over. Set
up the pair of helping hands far in front of you so that your arms can be partially extended and so that the
fumes do not rise towards your face. Heat the soldering iron to 350°C and “tin” the iron so that the tip is



shiny. Take your time while soldering and only ever put the soldering iron down into the stand. Remember
to unplug the iron after use even if it has a switch.
The wires on the motors are stranded instead of solid and do not make proper electrical connections with
the breadboards, so the wires must be spliced with 22 AWG wire that fits in the breadboard. The motor
wires should be cut at 9cm and have 1.5cm stripped for splicing. The 22 AWG wire should be cut to 12cm
and have 0.5cm stripped for splicing. The stranded wire should be wrapped around the solid wire using a
V-Shape Twist Splice and the two wires should be soldered together. After soldering, heat shrink tubing
should be put around the exposed wire for safety.
The battery must be connected to the switch and have its wires spliced with 22 AWG wire so that it can be
inserted into the Voltage Regulators screw terminals. The Ground and Power wires should be extended to
20cm by splicing it with the 22 AWG wire. Heat shrink tubing should cover the exposed wire.
A larger piece of heat shrink tubing should be placed on the power wire, then the wire should be soldered
to the switch and then the tube should be shrunk to cover the connection. Then a 10cm 22 AWG wire
should be soldered to the output of the switch and heat shrink tubing should cover the connection.
## 3.3.2 Fitting Components
First, a dummy motor should be pressed into the motor sleeves to test the fit and prepare the holes for
the real motors to be inserted.
Tape the motor wires together so that they form one large wire. Take the right motor, feed the combined
wire up through the front right hole, and insert the motor. Take the left motor and feed the combined wire
up through the front left wire hole, but do not yet insert the motor. Then take the switch and ground wire
for the battery and weave them below and behind the motor wires so that they are touching the front left
corner. Route the wires up through the left side cable holder, then insert the left motor into the sleeve to
hold everything in place. Pull the ground and power wires up through the back left wire hole. Place the
switch so that the Off is towards the left side and the On is towards the middle. Then take a zip tie and
secure the switch. Make sure that the locking mechanism of the zip tie is inside the chassis and not
below as it will rub on the ground. Use pliers to pull the zip tie tight.
When inserting the motors, be very careful as pushing on the back of the encoder can bend the motor
shaft and break the motor so make sure that a reasonable amount of pressure is used and press in line
with the axle, not on an angle.
Then the hubs should be inserted into the wheels and pressed onto the motors axle. Hold the motor in-
place while inserting the wheel hub and check the fit by gently pushing and pulling the wheel. If the fit is
proper, the motor shouldn’t move in the sleeve.
Take the two 400-pin breadboards and remove the power rails from the sides. Connect the breadboards
so that you have a breadboard, a power rail, then another breadboard. Build the circuit following the
circuit schematic with all the components that are not already inserted into the chassis. Be careful as the



numbers on the microcontroller are below the pins they represent so it is easy to misinterpret the pin
labels.
Then insert the breadboard into the chassis and connect the rest of the wires. Test the circuit by turning
on the power and see if the LED on the voltage regulator and the LED on the microcontroller turn on.




## Appendix A: Testing
Individual test files have been created to check different aspects of the agents. If an agent begins to
malfunction, flash and test the agent with the test files stored in the Firmware folder under the Test Code
subfolder.