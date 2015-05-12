#!/usr/bin/env python

'''
Testing reliability of leg detector

Empty environment:
What % of messages received from leg detector have (correct) person?
1) Stand still 2m, 4m directly in front (facing robot, not facing)
2) Walk a across field of view slowly (start 2m away, walk towards robot, walk from one side to the other)

In regular room:
What % of messages received from leg detector have (correct) person?
1) Stand still 2m, 4m directly in front (facing robot, not facing)
2) Walk a across field of view slowly (start 2m away, walk towards robot, walk from one side to the other)

(Optional)
In regular room:
When does Human Follower code set wrong goal (if person is showing)?
1) Same
2) Same

use wait for message!

OBSERVATIONS:

detector takes around .7 seconds recognize a person. 
Once it recognzies a person, it rarely flickers (happened 2 times in all the records)
Does not easily lose the person as long as the person doesn't leave view

focus: try to make sure the robot does not lose the person 

should try:
	how easy is it to flicker when the robot is moving?
	

'''

import sys
import time
import os
import roslib
import rospy

from people_msgs.msg import PositionMeasurementArray
from geometry_msgs.msg import *

## Constants
# Argument types
FILE_ARG = 		"-f"
DIST_ARG =		"-d"
QUIET_ARG = 	"-q"
HELP_ARG = 		"-h"
SIMPLE_ARG =	"-s"
COMPLEX_ARG =	"-c"
STILL_ARG = 	"-n"  # not moving?
MOVE_ARG = 		"-m"
MSG_ARG = 		"-z"

## Global variables
LOG_FILE_NAME = "quiet_mode"
LOG_FILE_HANDLE = None
ENV = "S"
DIST = 0;
MODE = "STILL"
MOVEMENT = "NONE"
MSG = ""


def timer():
	running = True
	counting = False
	countedTime = 0.0

	print "press enter to start experiment. Timer not started automatically"
	raw_input(">")

	# start timer
	startTime = time.time()
	endTime = time.time()
	lastStamp = time.time()

	# print start message
	print "Timer started!"

	while (running):
		input = str(raw_input(">"))
		if (input == "q"):
			endTime = time.time()
			if (counting):
				# stop time
				countedTime += (endTime - lastStamp)
			
			log("stopped, total at: " + str(countedTime) + " / " + str(endTime-startTime))
			running = False
		else:
			counting = not counting
			# update the time
			
			if (counting == False):
				# updating time
				countedTime += (time.time() - lastStamp)
				log("paused, total at: " + str(countedTime) + " / " + str(time.time()-startTime))
			else:
				log("continue from: " + str(countedTime) + " / " + str(time.time()-startTime))

				lastStamp = time.time()
	# outside of loop
	log("total: " + str(countedTime) + "/" + str(endTime - startTime))
	return (countedTime, endTime-startTime)

def generateFileName():
	''' generates the file name for this experiment
	    based on the type and the time
	'''
	filename = str(ENV) + "_" + str(MODE) + "_" + str(DIST) + "_" + str(MOVEMENT) + "_" + str(int(time.time())) + str(MSG)
	return filename


def log(message):
	'''	logs the message to either stdout and file based on LOG_FILE_HANDLE
		If handle is None, file logging is disapled
	'''
	# print to stdout
	print message

	#print to logfile
	if (LOG_FILE_HANDLE != None):
		print >> LOG_FILE_HANDLE, message
	

def checkValidArguments(args):
	if (not((COMPLEX_ARG in args) ^ (SIMPLE_ARG in args))):
		# not xor
		print "invalid syntax! please use -s or -c. Use -h for help"
		return False

	if (not((STILL_ARG in args) ^ (MOVE_ARG in args))):
		# not xor
		print "invalid syntax! please use -m or -n. Use -h for help"
		return False

	return True


'''
argument lists

-h 			: prints help menu

-s / -c 	: simple or complex environment
-d 			: specifies the distance
-q 			: does not log file. only print to output

-m / -n 	: move or not moving.
			  move should have additional input to say what kind of motion

-f filename : specifies the file to output the data to
			  default is just generated with time and mode
'''
def printHelp():
	# prints function of program
	print "Small systematic tester written for Paul's human_follower project"
	print "I should put more things in here but eh"
	print "\n"

	# prints out the argument lists
	print HELP_ARG + "\t\t\tprints help menu"
	print ""

	print SIMPLE_ARG + "\t\t\ttest is in simple environment"
	print "\t\t\tcannot be used with " + COMPLEX_ARG
	print ""

	print COMPLEX_ARG + "\t\t\ttest is in complex environment"
	print "\t\t\tcannot be used with " + SIMPLE_ARG
	print ""

	print STILL_ARG + "\t\t\ttest subject stands still"
	print "\t\t\tcannot be used with " + MOVE_ARG
	print ""

	print MOVE_ARG + " motion\t\ttest subject moves"
	print "\t\t\tcannot be used with " + STILL_ARG
	print ""

	print DIST_ARG + " distance" "\t\ttest performed at this distance away"
	print ""

	print FILE_ARG + " filename" + "\t\tspecifies log file"
	print ""

	print QUIET_ARG + "\t\t\tdoes not print to log file"
	print ""




if __name__ == '__main__':
	if (len(sys.argv) < 2):
		print "missing arguments! run '" + sys.argv[0] + " -h' for arguments"
		quit()

	args = dict()
	# build dictionary where key is the argument, value is the position
	# argument must start with -
	for i in range(len(sys.argv)):
		if (sys.argv[i].startswith("-")):
			args[sys.argv[i]] = i

	# check for help argument
	if (HELP_ARG in args):
		# display help options
		printHelp()
		quit()	

	# test for conflicting arguments
	if (not checkValidArguments(args)):
		# bad arguments should be printed inside the function
		quit()

	# check for arguments

	if (DIST_ARG in args):
		DIST = int(sys.argv[args[DIST_ARG] + 1])


	if (QUIET_ARG not in args):
		# we want a log file
		if (FILE_ARG in args):
			# specify output file
			LOG_FILE_NAME = sys.argv[args[FILE_ARG] + 1]
		else:
			if (COMPLEX_ARG in args):
				ENV = "C"
			else:
				ENV = "S"
			LOG_FILE_NAME = generateFileName()

		# open log file		
		LOG_FILE_HANDLE = open("./" + LOG_FILE_NAME, 'a')
	else:
		print "running quietly"	

	if ((MOVE_ARG in args)):
		MODE = "MOVE"
		if(sys.argv[args[MOVE_ARG] + 1].startswith("-")):
			# movement started with -, which is not okay!
			print "Did not specify movement!"
			quit()
		else:
			MOVEMENT = sys.argv[args[MOVE_ARG] + 1]

	if (MSG_ARG in args):
		MSG = "_" + sys.argv[args[MSG_ARG] + 1]


	### Start tester ###

	log("")
	log ("Experiment " + generateFileName())

	# start timer
	timer()
	
	# close log file
	if (QUIET_ARG not in args):
		LOG_FILE_HANDLE.close()
