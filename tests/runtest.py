#!/usr/bin/python

import sys
import subprocess
import os
import time
import stat

def main(engine, model_path):
	for filename in sorted(os.listdir(model_path), key=lambda filename: os.stat(os.path.join(model_path, filename)).st_size):
		if filename.lower().endswith('.stl'):
			print("Slicing: %s" % (filename))
			t = time.time()
			p = subprocess.Popen([engine, '-vv', os.path.join(model_path, filename)], stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
			stdout, stderr = p.communicate()
			if p.wait() != 0:
				print ("Engine failed to report success on test object: %s" % (filename))
				print(stderr.decode('utf-8', 'replace').split('\n')[-5:])
				sys.exit(1)
			else:
				print("Slicing took: %f" % (time.time() - t))

if __name__ == '__main__':
	if len(sys.argv) > 1:
		model_path = sys.argv[2]
	main(sys.argv[1], model_path)
