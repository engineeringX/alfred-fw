#!/usr/bin/env python

import matplotlib.pyplot as plt
import numpy as np
import sys

def plot_motion(filename):
  data = np.loadtxt(filename, delimiter=',')

  fig = plt.figure()
  plot = fig.add_subplot(111)
  plot.set_title("Motion")
  plot.set_xlabel("Time (ms)")
  plot.set_ylabel("Motion Filter Output")

  plot.plot(data[:,0], data[:,1], color='r', label='motion filter output')

  plt.show()

if __name__=="__main__":
  if(len(sys.argv) > 1):
    plot_motion(sys.argv[1])
  else:
    print("Filename not specified")
