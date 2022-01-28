#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

import os
from subprocess import call

file_dir = os.path.dirname(os.path.abspath(__file__))

beep_file = "../sounds/polite_beep/beepbeep3sec.wav"
honk_file = "../sounds/angry_honk/182474__keweldog__car-horn.wav"
batt_file = "../sounds/low_battery/413396__flood-mix__synth-descending-tones-glitchy.wav"

call(["amixer", "-D", "pulse", "sset", "Master", "80%"])


def callback(data):
	choice = data.data

	if choice == "beep":	
		call("ffplay -nodisp -autoexit -loglevel quiet " + beep_file, shell=True)

	elif choice == "honk":
		call("ffplay -nodisp -autoexit -loglevel quiet " + honk_file, shell=True)

	elif choice == "batt":
		call("ffplay -nodisp -autoexit -loglevel quiet " + batt_file, shell=True)

	elif choice == "exit" or choice == "quit":
		pass

	else: 
		print(choice + " is not a valid option. Please try again.")



def audio(): 
	# all init stuff goes here
	rospy.init_node('audio', anonymous=True)

	rospy.Subscriber('audio_channel', String, callback)

	rospy.spin()


if __name__ == "__main__":
	audio()
