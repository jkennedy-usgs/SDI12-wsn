# SDI12-wsn
c interface for SDI-12 communication on Atmel AVR and XBee nodes

This project is intended to allow wireless sensor networks - in particular, XBee devices - to be deployed at dataloggers with a host SDI-12 port. The data logger issues normal SDI-12 commands to a "bridge" device, which coordinates communication among the devices in the wireless sensor network. The benefit is that a local wireless network, using low-power radios, can relay data through an existing satellite telemetry network that uses data loggers with (usually ubiquitous) SDI-12 ports.

Additional detail is provided in the conference abstract and poster here:
https://www.researchgate.net/publication/266385447_Expanding_the_Usefulness_of_Existing_Data-Collection_Infrastructure_with_Wireless_Sensor_Networks
and in the draft manuscript provided in the Word document. 

The project includes c code to run on an AVR microcontroller and schematics and board layouts for producing PCB's for the bridge and node unit. The nodes are intended for use with voltage-output soil moisture probes but could be modified for other sensors. NOTE: the project is not designed to implement SDI-12 sensors in a wireless sensor network. 

The project was originally developed in 2010, and bench tested successfully. The hardware has not been field tested. 

The software and related documentation on these web pages were developed by the U.S. Geological Survey (USGS) for use by the USGS in fulfilling its mission. The software can be used, copied, modified, and distributed without any fee or cost. Use of appropriate credit is requested. The USGS provides no warranty, expressed or implied, as to the correctness of the furnished software or the suitability for any purpose. The software has been tested, but as with any complex software, there could be undetected errors. 

![State chart](/Statechart_WSN.png "State chart")
