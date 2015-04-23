#!/usr/bin/python

import sys
import subprocess
import os

def main(engine, model_path):
	for filename in os.listdir(model_path):
		if filename.lower().endswith('.stl'):
			print("Slicing: %s" % (filename))
			p = subprocess.Popen([engine, os.path.join(model_path, filename)], stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
			stdout, stderr = p.communicate()
			if p.wait() != 0:
				print ("Engine failed to report success on test object: %s" % (filename))
				sys.exit(1)

if __name__ == '__main__':
	if len(sys.argv) > 1:
		model_path = sys.argv[2]
	main(sys.argv[1], model_path)
