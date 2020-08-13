# -*- coding: utf-8 -*-
"""
Created on Thu Jul 27 11:50:48 2017

@author: ivan
"""

import numpy
import matplotlib.pyplot as plt
from matplotlib.widgets import Button

import warnings
import math

isSelected = False

firstP = False
calculateNow = False

warnings.filterwarnings("ignore",".*GUI is implemented.*")
fig = plt.figure()

ax = fig.add_subplot(111)

plt.axis([-1500, 1500, -1500, 1500])


plt.subplots_adjust(bottom=0.2)
initialValsX = numpy.random.random(10)
initialValsY = numpy.random.random(10)
plotHandle, = plt.plot(initialValsX,initialValsY, 'bo')
plotHandle2, = plt.plot(initialValsX,initialValsY, 'ro')

pStart = []
pEnd = []

def onclick(event):
    global ix, iy
    ix, iy = event.xdata, event.ydata
#    print (ix, iy)

    global coords, isSelected
    coords = []
    coords.append((ix, iy))
    
    isSelected = True
    
    
cid = fig.canvas.mpl_connect('button_press_event', onclick)


try:
    while True:
        plt.pause(0.0000000001)
        
        if isSelected is True:
            
            if firstP is False:
                plotHandle.set_data(coords[0][0],coords[0][1])
                pStart = numpy.array([[coords[0][0],coords[0][1]]])
                firstP = True
            else:
                plotHandle2.set_data(coords[0][0],coords[0][1])
                pEnd = numpy.array([[coords[0][0],coords[0][1]]])
                firstP = False
                
                calculateNow = True
                
            if calculateNow:
               diff = pStart - pEnd
               print(diff)
               calculateNow = False
#               row_sums = diff.sum(axis=1)
#               new_matrix = diff / row_sums[:, numpy.newaxis] 
                
                
#            print(coords[0][0],coords[0][1])
            
            isSelected = False
        
except KeyboardInterrupt:
    fig.canvas.mpl_disconnect(cid)
    print("Ended")