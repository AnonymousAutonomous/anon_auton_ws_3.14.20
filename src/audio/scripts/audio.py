#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

import os
from subprocess import call

file_dir = os.path.dirname(os.path.abspath(__file__))

beep_file = "../sounds/polite_beep/beepbeep3sec.wav"
honk_file = "/../sounds/angry_honk/182474__keweldog__car-horn.wav"
batt_file = "/../sounds/low_battery/413396__flood-mix__synth-descending-tones-glitchy.wav"

call(["amixer", "-D", "pulse", "sset", "Master", "80%"])
call("ffplay -nodisp -autoexit -loglevel quiet " + beep_file, shell=True)


def callback(data):
	choice = data.data

	if choice == "beep":	
		call(["ffplay", "-nodisp", "-autoexit", "-loglevel", "quiet", beep_file])

		# while beep_data:
		# 	beep_stream.write(beep_data)
		# 	beep_data = beep_f.readframes(chunk)

		# beep_f = wave.open(file_dir + r"/../sounds/polite_beep/461679__15hpanska-ruttner-jan__05-horn.wav", "rb")
		# beep_stream = p.open(format = p.get_format_from_width(beep_f.getsampwidth()), channels = beep_f.getnchannels(), rate = beep_f.getframerate(), output = True)
		# beep_data = beep_f.readframes(chunk)

	elif choice == "honk":
		subprocess.Popen("ffplay -nodisp -autoexit " + honk_file + " >/dev/null 2>&1", shell=True)

		# while honk_data:
		# 	honk_stream.write(honk_data)
		# 	honk_data = honk_f.readframes(chunk)

		# honk_f = wave.open(file_dir + r"/../sounds/angry_honk/182474__keweldog__car-horn.wav", "rb")
		# honk_stream = p.open(format = p.get_format_from_width(honk_f.getsampwidth()), channels = honk_f.getnchannels(), rate = honk_f.getframerate(), output = True)
		# honk_data = honk_f.readframes(chunk)

	elif choice == "batt":
		subprocess.Popen("ffplay -nodisp -autoexit " + batt_file + " >/dev/null 2>&1", shell=True)

		# while batt_data:
		# 	batt_stream.write(batt_data)
		# 	batt_data = batt_f.readframes(chunk)

		# batt_f = wave.open(file_dir + r"/../sounds/low_battery/413396__flood-mix__synth-descending-tones-glitchy.wav", "rb")
		# batt_stream = p.open(format = p.get_format_from_width(batt_f.getsampwidth()), channels = batt_f.getnchannels(), rate = batt_f.getframerate(), output = True)
		# batt_data = batt_f.readframes(chunk)	

	elif choice == "exit" or choice == "quit":
		# beep_stream.stop_stream()
		# beep_stream.close()	

		# honk_stream.stop_stream()
		# honk_stream.close()	

		# batt_stream.stop_stream()
		# batt_stream.close()	
		# p.terminate()
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
