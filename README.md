# Robust-Positioning

This project implements a robust positioning scheme.
Base positioning algorithm is AZIPE-angulation to opencv ArUco markers, and robustness is achieved with a separate visual odometry algorithm.

--Angulation--
Implementation of AZIPE algorithm:

KIM, JiJoong; HMAM, Hatem. 3D Self-Localisation from Angle of Arrival Measurements. DEFENCE SCIENCE AND TECHNOLOGY ORGANISATION EDINBURGH (AUSTRALIA) WEAPONS SYSTEMS DIV, 2009.

Positioning algorithm is dependent on separate input of vehicle (camera) roll and pitch, 3d location and yaw are solved for.
Algorithm solves a quartic equation, for which the following repo is used:

https://github.com/sasamil/Quartic

--Visual Odometry--

--Installation instructions--
For raspberry pi camera module, the RaspiCam library is needed.
