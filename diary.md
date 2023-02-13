## 6th Jan 2023
After matrix, went to Ardra Lab for working on hola bot. Felt new experience
with cutting, grinding and designing.

## 7th Jan 2023
slept till 6pm then again started building bot at ardra lab. Got a circular
container and fixed it on top of main body.

Searched and found a perfect sized (21 cm diameter and 10 cm height) plastic
box(dabba) which will act as 

## 11th Jan 2023
Ordered 5 cm M3 Studs from electronicscomp.com.

## 
Reading about PWM pin config of eYFI Mega 
Finding a way to control stepper motors in non-blocking manner.

##
Got a library `AccelStepper`, and reading its documentation from `Hackaday.io`
[link](https://hackaday.io/project/183279-accelstepper-the-missing-manual/details)

## 14th Jan 2023
Runebook Launched

## 
Not able to find M3 Screws

## 

## 22nd Jan 2023
Testing motordrivers on breadboard. Not able to give power efficiantly to the
motor drivers. Currently trying a hacky way by bending jumpers and connecting
it to breadboard. Burnet my fingers by accidentaly sorting terminals of the
battry while removing insulation(rubber coating).

## 23rd Jan 2023
Connected female sockets on perfboard for all 3 motor drivers and connected all
the vcc and ground pins. Also connected the power supply to perfboard. 

## 25th Jan 2023
Soldered the leftover pins of DIR and STEP pins of motor driver to mega board.
Also soldered vcc and ground to mega board from perfboard. Used 8 pins socket
to connect 6 pins of dir and step, 1 microstepping pin and 1 vcc.

But while testing, it was found that the potentiometer kept on spinning on one
motor driver.

## 26th Jan 2023
Testing the newly made circuit on perfboard and mega. Found that only 1 motor
is spinning. While debugging came to know that only the first two pins ie. DIR
and STEP of front wheel is giving pwm signals to the motordriver.

Searched for M3 screws at as many hardware stores we could found. 

Bought new Motor driver and burned a old working one. Plugged jumpers on the
sockets of DIR and STEP pins of 2 drivers while connecting the working
one(front wheel), tested all the ports with dmm and while debuging(testing) a
dangling jumper got touched with the heat sink of front wheel motor driver(RIP
moment).

So now back to square one with the motor drivers. But found a hacky way for M3
screws, removed 2 out of 4 screws from each motor clamps, and took 2 jumbo size
screws which came along motors, and connected both perfboard and Mega to their
places(Temporary solution).

## 31st Jan 2023
Continued debugging, and found left_wheel motor driver position is not working.
And i was suspecting the pin numbers used for left_wheel to be wrong. Because
the connections are made in a manner that if middle wheel(left) motor driver's
connection is not ok, then either side of the motors would also not be
working(series connection). 

## 3th Feb 2023
Not at all getting motivated to move further, reasons could be... things not
going as planned, first time working on hardware, working alone, missed
deadline, etc.

## 4th Feb 2023
Task 5 got released, yet we have not submetted task 4. Started testing camera
calibration script, just to get motivated again. At first the readymade script
was showing error. After googling for a bit, i found that i need to clone an
extra repo of `usb_cam` inside `src` directory of catkin workspace.

## 5th Feb 2023
Printed checker board on A4 paper and stuck it on the writing pad, calibrated
the camera with 97 samples with all possible angles distance and sizes which i
could.

Started testing the motor driver problem again, but eYFI Mega was not at all
responding. Tried to flash it again through wifi but upload failed again and
again, then tried wired flashing but it was taking forever but nothing was
happening. Tried to search on discussion forum and saw the same problem was
mentioned by someone, and came to know that the bootloader got RIP and found a
method to load the bootloader again through another arduino.

## 6th Feb 2023
Found an Arduino Mega board from our senior, started reading Readme page for
loading bootloader, followed steps but how could things go in right direction.
The arduino board was not being detected in windows, and linux was detecting it
normally. Tried on another laptop but same problem. And bootloader burning
program(AVR) which was provided would only run on windows.

## 8th Feb 2023
Out of the blue found that the board was detected for a fraction of second,
later found that it was issue with loose cable(dont know why it was detected on
linux then). But flashed the bootloader after making all the necessary
connections. 

Never thought it would feel so relieved after running LED blink program on eYFY
Mega before. Still i dont know why the bootloader got burned.

## 9th Feb 2023
Testing each and every pwm pins for both as a DIR and as a STEP pin by pluging
jumpers into 8 pin socket and testing for 1 motor driver at a time(took a lot
of time). Found these pin configuration as perfect combination for now.

![Pin Config](./docs/diary/pin_config.png)

Finally for the first time, all three wheels were rotating simultaneously(cant
describe that feeling here).

Wrote a code to drive bot clockwise and anticlockwise. Found that `delay()`
function is not working, even for loop was not adding any delay to the working.
But later after reading the documentation of `AccelStepper`, found that the
mechanism was a bit diffrent from what we were expecting, so delay was working
fine but it was useless here. 

Wrote code for moving bot back and forth with manually rotating 2 wheels, which
works fine. Now i have to move to the next level where giving `error` (set
point) in x, y or w would convert to wheel speed.

## 12th Feb 2023
Made the bot travel in any given direction i.e. any combination of x, y and w.
Gave 1, 1, 1 to x, y & w respectively to complete a circle. 

Fixed the camera on seeling fan which is 2.5 meters at height, 
recalibrated the camera with 110 samples. Tested detection of 5 arucos with ros which works perfectly fine.
