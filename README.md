# ElFarrama - Autonomous Robot

An autonomous robot system with computer vision capabilities for digit detection and precise navigation control.

## Features

For the computer vision , 
We chose to use open cv since it's the fastest and does not need that much of resources, like an ai model does, 
so we basically want to identify the 4 digits, (5,3,6,9) and what we did is let the openncv library detect in this logic

if there's any circle:
    if there's a line on top of the circle:
        print 6
    if there's a line under the cicle:
        print 9
else if there's horizontal lines:
        if there's aa vertical line on the left of the horizontal lines:
            print 5
        else 
            print 3

and it actually worked better than the model method, so yeah

