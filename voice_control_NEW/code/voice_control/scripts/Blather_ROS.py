#!/usr/bin/env python2

# -- this code is licensed GPLv3
# Copyright 2013 Jezra
# Modifications by: Allan Bogh - ajbogh@allanbogh.com

import sys
import signal
import gobject
import os.path
import subprocess #used to execute commands
import shutil #used to copy plugins directory
import psutil #used for reading process ID
import time #used for keyword time option
from optparse import OptionParser

import rospy
from std_msgs.msg import String
# import math

#keywords defined in the commands.conf file
keywords = []
PERCENT_MATCH_LIMIT = 75

#where are the files?
file_dir = os.path.dirname(os.path.abspath(__file__))
# conf_dir = os.path.expanduser("~/.config/blather")
conf_dir = os.path.join(file_dir, "blather_files", "")
lang_dir = os.path.join(conf_dir, "language")
plugin_dir = os.path.join(conf_dir, "plugins")

command_file = os.path.join(file_dir, "commands.conf")
strings_file = os.path.join(conf_dir, "sentences.corpus")
history_file = os.path.join(conf_dir, "blather.history")
language_update_script = os.path.join(file_dir, "language_updater.sh")
lang_file = os.path.join(lang_dir,'lm')
dic_file = os.path.join(lang_dir,'dic')


AUTO_UPDATE_CMD_FILE = False		# Set this to True if you want to update the commands each time you run the program.


class Blather:
	def __init__(self, opts):

		# Initialize our ROS node:
		rospy.init_node('voice')
		
		# Define our publisher:
		self.voice_pub = rospy.Publisher('voice', String, queue_size=1)

		#import the recognizer so Gst doesn't clobber our -h
		from Recognizer import Recognizer
		self.ui = None
		#keep track of the opts
		self.opts = opts
		ui_continuous_listen = False
		self.continuous_listen = opts.continuous

		self.stringsFileTime = os.path.getmtime(strings_file)
		self.commands = {}
		self.read_commands()
		
		self.recognizer = Recognizer(lang_file, dic_file, opts.microphone )
		self.recognizer.connect('finished',self.recognizer_finished)
		self.matchTime = 0
		self.keywordTimeLimit = opts.keytime #set to 0 to always speak the keyword

		# Update the Language File and Commands?
		self.commandFileTime = os.path.getmtime(command_file)		
		if ((AUTO_UPDATE_CMD_FILE) or (self.commandFileTime > self.stringsFileTime)):
			# Trick the system by making it think we just created the command file:
			self.commandFileTime = time.time()	
		self.checkCommandFile()

		#read options
		if self.opts.history:
			self.history = []


	def read_commands(self):
		#read the.commands file
		file_lines = open(command_file)
		strings = open(strings_file, "w")
		self.commands = {}
		self.keywords = []
		for line in file_lines:
				# print line			
				#trim the white spaces
				line = line.strip()
				#if the line has length and the first char isn't a hash
				if len(line) and line[0]!="#":
						#this is a parsible line
						(key,value) = line.split(":",1)
						print key, value
						#get the keyword out of the commands file
						if value == "keyword" and key.strip().lower() not in self.keywords:
							self.keywords.append(key.strip().lower())
							continue
						self.commands[key.strip().lower()] = value.strip()
						strings.write( key.strip()+"\n")

		#close the strings file
		strings.close()

	def log_history(self,text):
		if self.opts.history:
			self.history.append(text)
			if len(self.history) > self.opts.history:
				#pop off the first item
				self.history.pop(0)

			#open and truncate the blather history file
			hfile = open(history_file, "w")
			for line in self.history:
				hfile.write( line+"\n")
			#close the  file
			hfile.close()

	def recognizer_finished(self, recognizer, text):
		#split the words spoken into an array
		t = text.lower()
		textWords = t.split(" ")

		#get the keys array for all commands
		biggestKey = ""
		biggestKeySet = []
		biggestKeyCount = 0

		ret = self.search_for_matches(textWords)

		biggestKey = ret['biggestKey']
		biggestKeySet = ret['biggestKeySet']
		biggestKeyCount = ret['biggestKeyCount']

		#find the match percentage
		percentMatch = self.calculate_match_percentage(biggestKeySet, biggestKeyCount)

		if self.continuous_listen and len(set(self.keywords).intersection(set(biggestKeySet))) == 0:
			biggestKeyCount = 0

		#call the process
		if biggestKeyCount > 0 and ((len(textWords) <= 2 and len(biggestKeySet) == len(textWords)) or percentMatch >= PERCENT_MATCH_LIMIT): #must be equal or a 60% match
			self.matchTime = time.time()
			print("Best match: " + biggestKey, "Detected: " + text.lower(), "Percent match: " + str(percentMatch));
			cmd = self.commands[biggestKey]
			if cmd == "cancel" and hasattr(self, 'runningProcess'):
				print("Cancelling previous command with PID "+str(self.runningProcess.pid))

				self.terminate_child_processes(self.runningProcess.pid)

				#terminate parent process
				self.runningProcess.terminate();
			elif cmd != "cancel":
				print cmd
				if ("ros/" in cmd):
					[junk, ROScmd] = cmd.split("/") 
					self.voice_pub.publish(ROScmd)
					
				elif "plugins/" in cmd:
					#execute a plugin script
					self.runningProcess = subprocess.Popen(os.path.join(conf_dir,cmd), shell=True)
				else:
					self.runningProcess = subprocess.Popen(cmd, shell=True)
				self.log_history(text)
		else:
			print("No matching command", "Percent match: " + str(percentMatch))
		#if there is a UI and we are not continuous listen
		if self.ui:
			if not self.continuous_listen:
				#stop listening
				self.recognizer.pause()
			#let the UI know that there is a finish
			self.ui.finished(t)
		#check if the command.conf file has changed.
		self.checkCommandFile()

	def run(self):
		if self.ui:
			self.ui.run()
		else:
			blather.recognizer.listen()

	def quit(self):
		sys.exit(0)

	def checkCommandFile(self):		
		stringsFileTime = os.path.getmtime(strings_file)
		if stringsFileTime < self.commandFileTime:
			print("Command.conf file modified")
			subprocess.call(language_update_script)
			print("Language file updated")
			self.read_commands()
		else:
			print 'NO NEED TO UPDATE?'
		# We'll update this time here (at the end)
		# because we used the trick above to force a re-load.
		# FIXME -- This is a really bad way to do things.
		# The problem is that the script re-writes the strings file
		# at the very beginning.  This automatically makes the 
		# strings file newer than the command file.
		self.commandFileTime = os.path.getmtime(command_file)
			
	def process_command(self, UI, command):
		print command
		if command == "listen":
			self.recognizer.listen()
		elif command == "stop":
			self.recognizer.pause()
		elif command == "continuous_listen":
			self.continuous_listen = True
			self.recognizer.listen()
		elif command == "continuous_stop":
			self.continuous_listen = False
			self.recognizer.pause()
		elif command == "quit":
			self.quit()


	def search_for_matches(self, textWords):
		#TODO: https://github.com/ajbogh/blather/issues/1
		ret = {'biggestKey':'', 'biggestKeySet':{}, 'biggestKeyCount':0}
		currentTime = time.time()
		matchLimit = 1
		for key in self.commands.keys():
			if self.commands[key] == "keyword":
				continue
			#split the keys on each word
			words = set(key.split(" "))
			#append the keyword to the command if it's not there already
			##only if the timed keyword activation is needed
			if self.keywordTimeLimit > 0 and self.continuous_listen and (currentTime - self.matchTime) > self.keywordTimeLimit and len(set(self.keywords).intersection(set(words))) == 0:
				words.update(self.keywords)
			elif len(set(self.keywords).intersection(set(textWords))) > 0:
				words.update(self.keywords)

			#find the matching words
			matches = words.intersection(set(textWords))
			#determine if the words match
			if self.continuous_listen and len(set(self.keywords).intersection(set(textWords))) > 0 and (currentTime - self.matchTime) > self.keywordTimeLimit:
				matchLimit = 2
			if len(matches) >= matchLimit and len(matches) > ret['biggestKeyCount']:
				ret['biggestKeySet'] = words
				ret['biggestKeyCount'] = len(matches)
				ret['biggestKey'] = key
		return ret

	def calculate_match_percentage(self, biggestKeySet, biggestKeyCount):
		percentMatch = 0
		if len(biggestKeySet) > 0:
			percentMatch = (biggestKeyCount/float(len(biggestKeySet))) * 100
		return percentMatch

	# terminate_child_processes kills any child processes under a parent pid.
	# It uses pgrep to list child processes, so the system must have pgrep installed in order
	# to use the 'cancel' commands
	def terminate_child_processes(self, pid):
		out = subprocess.Popen(['pgrep', '-P', str(pid)], stdout=subprocess.PIPE).communicate()[0]
		childProcesses = out.splitlines()
		# Kill any orphaned children.
		for pid in childProcesses:
			#recursive call to kill entire family tree
			self.terminate_child_processes(int(pid))
			print("Killing child with PID "+str(pid))
			p = psutil.Process(int(pid))
			p.terminate()


if __name__ == "__main__":
	#create a bunch of commandline options
	parser = OptionParser()
	parser.add_option("-i", "--interface",  type="string", dest="interface",
		action='store',
		help="Interface to use (if any). 'q' for Qt, 'g' for GTK")
	parser.add_option("-c", "--continuous",
		action="store_true", dest="continuous", default=False,
		help="starts interface with 'continuous' listen enabled")
	parser.add_option("-H", "--history", type="int",
		action="store", dest="history",
		help="number of commands to store in history file")
	parser.add_option("-m", "--microphone", type="int",
		action="store", dest="microphone", default=None,
		help="Audio input card to use (if other than system default)")
	parser.add_option("-k", "--keytime", type="int",
		action="store", dest="keytime", default=0,
		help="In continuous mode, the amount of time (in seconds) after the keyword is initially used before having to say the keyword again to activate a command.")

	# FIXME -- Above, we probably want to set the continuous option to default as "True".

	(options, args) = parser.parse_args()
	#make our blather object
	blather = Blather(options)
	#init gobject threads
	gobject.threads_init()
	#we want a main loop
	main_loop = gobject.MainLoop()
	#handle sigint
	signal.signal(signal.SIGINT, signal.SIG_DFL)
	#run the blather
	blather.run()
	#start the main loop

	try:
		main_loop.run()
	except:
		print "time to quit"
		main_loop.quit()
		sys.exit()
