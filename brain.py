#!/usr/bin/env python

# TODO: import packages
# import rospy
# TODO: import ros message types
# TODO: import files with algos: path planning

def main():
    return
    # ------- STANDBY -------
    # robot responds to fun controller commands: driving, squatting, clucking, arm movements
    
    # driving
        # convert controller inputs into encoder values to send to interface
        # TODO: what values should be changed for rotating and translationg (how much to increment / decrement values)
        # rotate (counter/clock)
        # translate (forward/back)

    # clucking 

    # squatting
    # idk @ meche people TODO 

    # arm movements
    # TODO @meche

    # initialize fetch routine


    # ------- SEARCHING -------
    # robot turns to search for the ball

    # robot receives input from controller telling it to begin search 
    # tilt head at a starting angle
    # initialize variables: ball position, angle increment
    # while not found, perform 360 scan
        # to perform 360 scan, run CV algo as she slowly rotates in a circle
            # ie camera node returns ball position if that's found, otherwise returns null
            # TODO: (check with Warren about best strat for this) check how stably she rotates (bc not perfect circle), maybe have her incrementally pause as she moves into each configuration
                # staring 
                # ie pauses at first scan, rotates 15 degrees clockwise, pauses again and scans, moves 15 degrees clockwise, pauses and scans, ...
        # if found, store location and break while loop
        # if not found, tilt head upward at a set increment (and perform another 360)
            # TODO: consider/calculate decreasing increment as we perform more iterations of the while loop; as angle increases, field of vision increases
            # TODO: check with Warren how the angle would be measured (oh check overleaf first actually)
    # TODO: consider using for loop instead of while loop; at end of for loop, robot returns that it cannot find the ball
    # TODO: consider how to run this if we had a controller?? — robot stops search when we press any key? or robot ignores our keypresses as it's in this stage?

    # ------- PATH PLANNING #1 -------
    # path plan robot from current location to a fixed distance from the ball


    # TODO: maybe this whole thing might be 1 function that is called in the next while loop 
    # calculate a (concentric?) circle around the robot, which represents the locations that the robot can potentially stand in 
        # the concentric circle is a fixed distance away from the ball
        # r^2 = (x - h)^2 + (y - k)^2, with r being radius, h being x position, k being y pos
        # TODO: find h and k in which frame?
    # initialize: potential position (p) for robot to move to
        # point on the concentric circle that is closest to the robot
    # while True 
        # run path planning algo from robot to p
        # if path can be created, break
        # if path cannot be created, set p to new position  
            # for new position, maybe add 15 degrees clockwise (within the concentric circle) to initial position 
    # TODO: potentially use for loop (for similar reason as 'searching')

    # robot moves now
    # while she is NOT within a certain position interval (ie at or close to her destination)
        # send command for robot to move 
            # TODO: what to do if she's too close to an obstacle (probably plan a new path)
            # if she is nearing her destination, send fewer commands (ie moves slower)
        # neck angle = tan(vertical / dist from ball)
            # send command to ESP32 to adjust neck angle
            # sleep/pause this action for a few moments — avoid bugging or too many commands? (we can see about this)
        # TODO: double check how her body moves to keep track of the ball

    # ------- GRASPING — PATH PLANNING #2 -------
    # robot bends downward, keep path planning until camera cannot keep track of the ball

    # just run a function lol
    # robot moves such that its beak directly faces the ball (if not already in the position)
    # robot opens beak
    # robot bends down — TODO @meche people lol
        # keep running path planning algo as this is happening, until camera cannot keep track of the ball
        # ^ or maybe run path planning algo of up until 
    # robot drives to ball
    # robot clutches ball, rises back up 

    
    # ------- GRASPING — PATH PLANNING #2 -------
    # robot returns back to original location — similar to Path Planning #1

    




if __name__ == '__main__':
    main()