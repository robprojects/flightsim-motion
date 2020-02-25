# flightsim-motion

Implements motion control for a 6DOF flight simulator, with graphical output showing the platform position. Takes game data as input with XPlane (flight simulator) and NoLimits2 (roller coaster simulator) currently supported. Can be configured to support multiple actuator types, simulator geometry and game sources, but you will need to modify the code!

I built this to run my own simulator. It is fully functional (and a lot of fun!) but should be considered experimental and a work in progress.

See http://robprojects.github.io for more details.

## Building

I use on Linux but the dependencies are fairly minimal so should be portable to almost anything. A simple Makefile is supplied. The only dependencies are OpenGL and GLUT for the graphical interface.

## Usage 

``` motion -x 192.168.1.2``` connect to XPlane UDP server at 192.168.1.2

``` motion -l 192.168.1.2``` connect to NoLimits2 server at 192.168.1.2

``` motion -m ``` manual mode (no game data), use keypresses to generate motion cues

Running without any arguments starts up in a mode where you can move the platform through the six degrees of freedom using the keyboard. A,D,W,S and Z move in the rotational axis. The arrow keys and page up/down move in the translational axis.

Other arguments:

``` -n     --nodevice ``` use without a device (useful for testing if you don't have any actuators)

## Washout algorithm
The classical motion cueing algorithm described by Reid and Nahon is implemented in washout.c.
There are various tunable parameters in the function sim_params_init()

## 6DOF platform geometry
The 6DOF (Stewart) platform inverse kinematics are implemented in geo6dof.c. The call to the function init_geometry() sets up the geometry of the platform and actuators. You will need to change these parameters to suit your platform.

| Parameter | Dimension |
|-----------|-----------|
| radius_base | The radius of the fixed base in meters |
| radius_platform | The radius of the motion platform in meters |
| mid_length | The length of the actuators in the centre of their travel in meters |
| min_length | The minimum length of the actuators in meters |
| range | The range of the actuators (min to max) in meters |
| sep_angle | The angle between two adjacent actuator mountings on the fixed base in radians |
| sep_angle_platform | The angle between two adjacent actuator mountings on the motion platform in radians |

The current code supports linear actuators only.

## Actuators
Code to drive the actuators lives in actuator.c. This is set up to run my actuators, to use with a different type you will need to make changes to this file.

The function setup_serial() contains the code to initialise the serial port and actuators.

The function send_serial_command() takes an array of actuator lengths and generates a serial command.

For simple actuators that take a serial command stream and don't provide position updates you will need to remove most of the code in setup_serial().

### Serial command format

#### To actuator
| Serial string | Command |
|---------------|---------|
| P | start of a new command (reset) |
| A\<id\>D\<value\> | Set the position of actuator \<id\> to decimal length \<value\> between 0 and 2^16 |
| K\<id\>D\<value\> | Set constant \<id\> to decimal value \<value\> (PID coefficients are fixed point) |
| I | Ping |
| H | Send actuator to home position | 

#### From actuator
| Serial string | Response |
|---------------|----------|
| P | start of a new response (reset) |
| A\<id\>D\<value\>F\<flags\> | Report the position of actuator \<id\> as decimal length <value> between 0 and 2^16 and return status in \<flags\> |

Flags:
 * Bits 0-3 are the state of the actuator
 * Bit 5 indicates a constant was set in the last command
 * Bit 6 indicates a response to a ping
 * Bit 7 indicates the brake resistor has hit the thermal limit

## Game data

The code currently supports two game data sources:

 * XPlane11 UDP interface is implemented in xplane.c
 * NoLimits2 TCP interface is implemented in nolimits2.c

## License

Copyright 2020 Rob Dimond

Licenced under the GNU General Public License, Version 3.0
