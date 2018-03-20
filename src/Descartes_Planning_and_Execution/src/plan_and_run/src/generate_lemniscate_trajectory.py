#!/usr/bin/env python

import numpy
import math
import matplotlib.pyplot as pyplot
from mpl_toolkits.mplot3d import Axes3D


def generateLemniscatePoints():

    # 3D plotting setup
    fig = pyplot.figure()
    ax = fig.add_subplot(111,projection='3d')

    a = 6.0
    ro = 4.0
    dtheta = 0.1
    nsamples = 200
    nlemniscates = 4
    epsilon = 0.0001

    # polar angle
    theta = numpy.concatenate([numpy.linspace(-math.pi/4 + epsilon,math.pi/4 - epsilon,nsamples/2,endpoint=True),
      numpy.linspace(3*math.pi/4 + epsilon, 5*math.pi/4- epsilon,nsamples/2,endpoint=True)])

    # offset from polar angle
    omega = numpy.linspace(0.0,math.pi,nlemniscates,endpoint=False)

    r = len(theta)*[0]
    x = (len(theta)*len(omega))*[0]
    y = (len(theta)*len(omega))*[0]
    z = (len(theta)*len(omega))*[0]

    for j in range(0,len(omega)):

      index_offset = j*len(theta)
      for i in range(0,len(theta)):      

        r[i] = math.sqrt(math.pow(a,2)*math.cos(2*theta[i]))

        index = index_offset + i
        
        phi = math.asin(r[i]/ro) if r[i] < ro else (math.pi - math.asin((2*ro-r[i])/ro) )
        x[index] = ro*math.cos(theta[i] + omega[j]) * math.sin(phi)
        y[index] = ro*math.sin(theta[i] + omega[j]) * math.sin(phi)
        z[index] = ro*math.cos(phi) 

    #print "omega array: %s"%(str(omega))
    #print "x array: %s"%(str(x))
    #print "z array: %s"%(str(z))
    axis_size = 1.2*ro
    ax.plot(x, y, z, label='parametric curve',marker='.',color='yellow', linestyle='dashed',markerfacecolor='blue')
    ax.legend()
    ax.set_xlabel('X')
    ax.set_xlim(-axis_size, axis_size)
    ax.set_ylabel('Y')
    ax.set_ylim(-axis_size, axis_size)
    ax.set_zlabel('Z')
    ax.set_zlim(-axis_size, axis_size)

    pyplot.show()


if __name__ == "__main__":

  generateLemniscatePoints()
