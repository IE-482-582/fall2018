import random

groups = ["Akarsh", "Antonio/Brendan", "Cameron/Vincenzo", "Connor/Vincent", \
	"David/Scott", "Emily", "Kai/Pin-Chun", "Lan", \
	"Mindy/Tiffany", "Obe/Pushkaraj"]


print '*****HIT ENTER TO START*****'
raw_input()


counter = 0
while len(groups) > 0:

	randIndex = random.randint(0, len(groups)-1)

	counter += 1
	print("\t%d -- %s" % (counter, groups[randIndex]))	
	
	del groups[randIndex]
	
	print '\t*****HIT ENTER TO CONTINUE*****'
	raw_input()

	
	