# Open-Source-Delta-Robot-Project
Delta Robot pick &amp; place Machine GUI on visual Studio and 3D Model
TunMaker
WebSite ► https://www.tunmaker.tn/
Twitter ► https://twitter.com/Tun_Maker
Facebook ► https://www.facebook.com/TunisanMaker/
Instagram ► https://www.instagram.com/tunmaker/
YouTube ► https://www.youtube.com/c/tunmaker

Playlist of the Project Build from testing and fails to working prototype : https://youtu.be/g7xa9a76zoU?t=8m35s

this repository contains the GUI for the delta robot including the image processing part it is not yet complete and needs your help and expertise to become 100% functional.

the GUI is made with Visual Studio 2017 Community version , i tweaked the code from http://forums.trossenrobotics.com/tutorials/introduction-129/delta-robot-kinematics-3276/ for the kinematics calculations to determine its position. the EmguCV library for image processing (most be installed for the program to function).
the GUI interfaces with Machine using USB SERIAL of an arduino like a cnc machine.in order for it to work you need grbl(https://github.com/grbl/grbl) installed and configured so that each stepper X Y and Z are moved according to there angles ( steps/mm parameter).

The 3D Model is Made with Fusion360 which you can view on the browser with this public link : https://a360.co/2LHBC2j
