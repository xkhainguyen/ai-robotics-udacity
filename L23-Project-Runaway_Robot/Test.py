from robot import *
from math import *
for i in range(10):
    pf = particles(0, 0, 0., 0., 0., 10)
    pf.move
    pf.sense((1.,1.))
    print(pf.data[i])

for i in range(10):
    pf.move
    pf.sense((2.,1.))
    print(pf.data[i])    