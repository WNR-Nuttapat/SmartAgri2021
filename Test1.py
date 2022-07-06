import numpy as np
import math

x = [1,1,-1,-1];
y = [1,-1,-1,1];

def getTheta(v,w):
    a = [];
    for i in range(len(x)):
        a.append(math.atan2(y[i],(v/w - x[i])));
    return a;

def getVelocity(v,w):
    a = [];
    for i in range(len(x)):
        a.append(w*math.sqrt(math.pow(v/w - x[i],2) + math.pow(y[i],2)));
    return a;

print(getTheta(10,1));
print(getVelocity(10,1));