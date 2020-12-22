# Orca2
MK20DX256 libraries for Orca2 control

This Library contains all the tools needed to control an Iris Dynamics Orca2 driver using and Eagle2 USB controller along with the IrisControls Windows USB Application 

# High Level


## Streaming  Data 
Toggling the "Stream" button commands the USB controller to begin streaming data at thefFrequency indicated by the Frequency input slider. 
Streaming begins the next time a Command button is pressed and continues until the Stream button is untoggled, or no run command is toggled. 

(good) bug: After toggling "Stream" button, two more verbose packages are sent and reported due to the toggling of the Command button
## Caputuring Multiple Packages 
The Text window of the app can be used to keep track of the past responses form the Orca. When History is toggled, non-streamed responses will be displayed on the Text window as well as on the Feedback Sliders. When streaming, a number of packages equal to the History Depth slider will be printed to the text window, after which Streaming is automatically disabled. This is to prevent clogging of the USB line. 
#### TODO: prepetual streaming below a certain frequency 



## Flashing Orcas 

## Orca Fuses
### Extended Fuse:  0xFD
Brown Out Detection is set to ~2.5 V 

### High Fuse: 0xD6
* Allow External Reset 
* Disable Debug Wire 
* Enable Serial Programming (Bootloaders + Fuses) 
* Watchdog disabled
* Preserve EEPROM when Flashed 
* Minimum Bootloader Space
* Use Bootloader Reset Vector 

### Low Fuse: 0xFF
* Disable Clock Divide-by-8
* Disable Clock output on B0
* Maximum Startup time 
* Select External Clock Source (20MHz crystal) 


# Low Level 
## Tx Frames 
1)  An Orca asks its UART for a frame pointer (to the next open frame) using get_next_frame()
2)  The Orca populates the frame object according to whatever its doing 
3)  The orca asks the UART to send that populated (hopefully legal) frame. 
4)  The UART complies by moving its circular buffer to include the new frame (UART contains a circular buffer of pointers to frames)
5)  The Orca then requests the UART to send_all_frames() which will enable the transmit interrupts and automatically empty the tx_buffer. 
## Rx Frames  
1)  A data comes in to UARTn with the 9th bit set, implying it is a new frame 
2)  If a previous frame was being built, it is checked for validity 
3)  A new frame is created, and the header is decoded to predict the frame length. 
4)  Subsequent incoming bytes are added to the frame until either  
* the size of the frame is reached and the frame is checked for validity
* The time between two consecutive bytes exceeds a threshold and the frame is marked as invalid 
5) Valid frames are sent to the Orca indicated by the header 
6) Invalid frames result in an increase to the error sum corresponding to the reason for invalidity 


