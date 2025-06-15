import math
g=9.8

#radius=
#angle in radian
def calcspeed(botheight,hoopheight,distance,angle):
    height=hoopheight-botheight
    numerator=g*distance*distance
    denominator=2*((distance*(math.tan(angle))-height)*(math.cos(angle)**2))
    velocitysq=numerator/denominator
    velocity=math.sqrt(velocitysq)
    return velocity

def calcrpm(botheight,hoopheight,distance,angle,radius):
    v=calcspeed(botheight,hoopheight,distance,angle)
    print(v)
    rpm=(v*30)/((math.pi)*radius)
    #rpm=rpm*2
    return rpm

print(calcrpm(1.45,2.43,4.7,1.13446,0.065))
#print(calcspeed(1.45,2.43,3,1.13446))
#print(calcrpm(1.45,2.43,5,0.645772,0.065))


    