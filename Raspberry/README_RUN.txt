APEX ROVER MODULAR BRAIN V2
===========================

Why V2?
-------
The previous modular version was too slow and some commands felt like one-shot.

V2 fixes:
- Manual movement sends every time you press the key.
- Camera movement sends every time you press the key.
- UP/DOWN inversion is fixed in config.py:
      INVERT_CAMERA_VERTICAL = True
- Serial timeout is reduced.
- Old serial buffers are cleared when connected.
- Main loop drains several Mega lines to reduce lag.
- Jack test keys are added.
- jack_test.py is added.

Run main brain
--------------
cd ~/ApexRoverBrain
python3 main_brain.py

Main brain keys
---------------
Modes:
0 = IDLE
m = MANUAL
1 = OBJECT
2 = STAIRS
3 = CLIMB_ASSIST

Movement:
f = forward
b = backward
l = left
r = right
x = stop

Camera:
w = camera up
s = camera down
a = camera left
d = camera right
c = camera center

Jacks:
g = front jack extend
h = front jack retract
n = front jack stop

u = rear jack extend
j = rear jack retract
k = rear jack stop

z = stop all jacks

Other:
p = print latest sensors
q = quit

Test jacks only
---------------
cd ~/ApexRoverBrain
python3 jack_test.py

Then write:
fe     front extend
fr     front retract
fs     front stop

re     rear extend
rr     rear retract
rs     rear stop

jstop  stop both jacks
q      quit

Safe jack test steps
--------------------
1) Write fe
2) Wait about 1 second
3) Write fs

Then:
1) Write fr
2) Wait about 1 second
3) Write fs

Same idea for rear jack using re/rr/rs.

Mega sensor format
------------------
DATA:PITCH=5.2;ROLL=-1.4;UF=8.5;UR=9.1

Mega / ESP32 mode commands
--------------------------
MODE:OBJECT
MODE:STAIRS
MODE:CLIMB_ASSIST
MODE:MANUAL
MODE:IDLE

Target color commands
---------------------
TARGET:red
TARGET:blue
TARGET:green
TARGET:yellow
TARGET:orange
TARGET:pink
TARGET:purple
TARGET:black
TARGET:white

Mega should understand
----------------------
FORWARD
BACKWARD
LEFT
RIGHT
STOP
SPEED:40
MODE:CLIMB
MODE:NORMAL
JACK:FRONT:EXTEND
JACK:FRONT:RETRACT
JACK:FRONT:STOP
JACK:REAR:EXTEND
JACK:REAR:RETRACT
JACK:REAR:STOP

UNO should understand
---------------------
CAM:LEFT
CAM:RIGHT
CAM:UP
CAM:DOWN
CAM:CENTER
CAM:STOP
