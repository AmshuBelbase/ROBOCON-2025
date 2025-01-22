import numpy as np
from itertools import combinations


def quadarea(point):
    points=np.array(point,np.float64)
    x1, y1 = points[0]
    x2, y2 = points[1]
    x3, y3 = points[2]
    x4, y4 = points[3]
    return abs(x1*y2 + x2*y3 + x3*y4 + x4*y1 - (y1*x2 + y2*x3 + y3*x4 + y4*x1)) / 2

def bestcor(coords):
    max_area=0
    best_quad=None

    for quad in combinations(coords,4):
        area = quadarea(quad)
        if(area>max_area):
            max_area=area
            best_quad=quad


    return best_quad,max_area

coords = [[596, 390],[604, 390],[714, 266],[714, 202],[692, 58],[688, 44]]
print(bestcor(coords))




# 




