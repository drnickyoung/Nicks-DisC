# Nick's DisC
Nick's Disk Controller, project to interface and read floppy disks for example from an Amiga.

## Disclaimer
This project connects various pieces of hardware and attempts to control disks at a low level. I take no responibility what-so-ever for any damage or loss of data to your equipment. You use this code and instructions at your own risk. 

## Aim
The project aims to create a floppy disk controller. This will connect, via a USB port, to a floppy disk drive. In between there will be a controller. The controller will allow the reading, and possibly writing, of different floppy disks such as MS-DOS and Amiga. A computer program will then create image files of the disks which can be used for storage or in emulators.

## Roadmap
This is a project that will be completed in stages, these may change over time but the initial plan is:

1. Arduino Interface
  * Connecting an external Amiga floppy disk drive (DD), controlling all elements (except reading & writing)
  * Connecting a standard internal PC floppy disk drive, controlling all elements (except reading & writing)
2. Disk reading
  * Get Arduino to read data directly from the drive and transmit, via USB, to connected computer.
  * Depending on above, look at alternative processors/systems.
3. Data decoding
  * Get computer software to decode MFM (or other?) data stream.
  * Get Arduino, or alternative, to decode data directly from the drive and transmit, via USB, to connected computer.
  * Depending on 3.2, look at alternative processors/systems.
4. Create img file
  * Create an img, ADF, or other file from the data.
  * Test the file in UAE, VirtualBox, or other.
5. Release Version 1
  * Create a PCB and test
  * Release Version 1 - Read only system for MS-DOS and Amiga disks.
6. Write Data
  * Allow Arduino, or alternative, to write data to a disk.
  * Update computer software.
  * Test the disk in a REAL system, a PC, Amiga, etc.
7. Release Version 2
  * Update the PCB, if required, and test
  * Release Version 2 - Read and write system for MS-DOS and Amiga disks.
8. Expand
  * Add support for other disk formats.
  * Add support for other file types.
  * Add support for 5 1/4 inch and possibly 8 inch drives.

There are no timescales, this is being done in my spare time with equipment available.
