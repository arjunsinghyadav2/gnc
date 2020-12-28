"""
Consider a set of three points in space
[2,4,7],[6,2,1],[9,5,4] shown as x,y,z cordinates of those points

Our goal is to check for colinearity

"""
import numpy as np


def colinearity_check(p1, p2, p3):
    #convert the point into matrix
    m = np.mat([p1, p2, p3])

    #then we find area under the point and if the area is ==0 then the points are colinear
    if np.linalg.det(m) == 0:
        print("Points are colinear")
    else:
        print("Noncolinear points")

#defining the points in (x,y,z) format
#NOTE: We consider z to be 0 intentionally
point1 = np.array([1, 2, 0])
point2 = np.array([2, 3, 0])
point3 = np.array([3, 4, 0])

colinearity_check(point1,point2,point3)
#Should return True

"""
Now we will try the case where z is not 0
"""
point1 = np.array([1, 2, 1])
point2 = np.array([2, 3, 2])
point3 = np.array([3, 4, 3])

colinearity_check(point1, point2, point3)
#Should return True
