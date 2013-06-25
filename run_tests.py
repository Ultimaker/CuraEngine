import sys
import os
import subprocess

def main():
	executableName = 'CuraEngine'
	if len(sys.argv) > 1:
		executableName = sys.argv[1]
	
	exitValue = 0
	for subPath in os.listdir('testcase_models'):
		print 'Running test on %s' % (subPath)
		ret = subprocess.call([executableName, os.path.join('testcase_models', subPath))
		if ret != 0:
			exitValue = 1
	
	sys.exit(exitValue)

if __name__ == '__main__':
	main()
