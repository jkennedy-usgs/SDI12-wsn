# SDI12-wsn
c interface for SDI-12 communication on Atmel AVR and XBee nodes

This project is intended to allow wireless sensor networks - in particular, XBee devices - to be deployed at dataloggers with a host SDI-12 port. The data logger issues normal SDI-12 commands to a "bridge" device, which coordinates communication among the devices in the wireless sensor network. The benefit is that a local wireless network, using low-power radios, can relay data through an existing satellite telemetry network that uses data loggers with (usually ubiquitous) SDI-12 ports.
