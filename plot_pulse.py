#!/usr/bin/env python

import matplotlib.pyplot as plt
import numpy as np
import sys

def plot_pulse(filename):
  data = np.loadtxt(filename, delimiter=',')

  fig = plt.figure()
  plot = fig.add_subplot(111)
  plot.set_title("Pulse")
  plot.set_xlabel("Time (ms)")
  plot.set_ylabel("Raw Pulse Measurement")

  plot.plot(data[:,0], data[:,1], color='r', label='pulse data')

  plt.show()

if __name__=="__main__":
  if(len(sys.argv) > 1):
    plot_pulse(sys.argv[1])
  else:
    print("Filename not specified")
