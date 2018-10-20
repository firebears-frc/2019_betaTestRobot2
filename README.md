# Test Robot Zero

This project was originally generated from a pre-season version of RobotBuilder, but then reworked for GradleRio.  

This robot uses the Command pattern. Major parts of the physical robot are represented by Subsystems in the software.  Commands are objects that operate on the Subsystems.  The robot's Operator Interface (OI) class connects joysticks and buttons to the commands.

This project uses a property file to define the important numbers and configurations in program.  For instance, the CAN identifiers for the motors are set in the property file.

Some documentation is at https://firstmncsa.wordpress.com
