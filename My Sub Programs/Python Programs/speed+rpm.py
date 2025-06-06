import math
import numpy as np

g = 9.8


def calcDist(bh, hh, rpm, a, r):
    c = (2*((rpm*(math.pi)*r*math.cos(a))**2))/(900*g)
    dp = ((c*math.tan(a))+math.sqrt(((c*math.tan(a))**2)-(4*c*(hh-bh))))/2
    return dp


def calcRpm(bh, hh, d, a, radius):
    v = math.sqrt(((g*d*d)/(2*(((d*(math.tan(a))) -
                                (hh-bh))*(math.cos(a)**2)))))
    rpm = (v*30)/((math.pi)*radius)
    return rpm


# True - calculate approx rpm from given distance
# False - calculate approx distance from given rpm
if (False):
    ds = list(np.arange(2, 8.2, 0.2))
    print(ds)
    for i in ds:
        # botheight, hoopheight, distance, angle, radius
        print("For Distance=", i, "Rpm=", calcRpm(
            1.45, 2.43, i, 1.13446, 0.065))
else:
    # rpms = [*range(1000, 4020, 20)]
    rpms = [1165.741736]
    # print(rpms)
    for i in rpms:
        # botheight, hoopheight, rpm, angle, radius
        print("For RPM=", i, "Distance=", calcDist(
            1.45, 2.43, i, 1.13446, 0.065))
