#!/usr/bin/python

import sys
import subprocess

def main(engine):
	p = subprocess.Popen([engine, '-c', 'supportAngle=60', '-c', 'supportEverywhere=1', '_tests/testModel.stl'], stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
	p.communicate()
	if p.wait() != 0:
		print "Engine failed to report success on test object slice..."
		sys.exit(1)

if __name__ == '__main__':
	main(sys.argv[1])
