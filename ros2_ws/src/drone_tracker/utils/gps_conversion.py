#import statement for necessary libraries (installed with pyserial)
import subprocess, os, keyboard, time, serial
import numpy as np
class GPS_utils:
    #from Mike
    def __init__(self):
# Geodetic System WGS 84 axes
        self.a  = 6378137.0
        self.b  = 6356752.314245
        self.a2 = self.a * self.a
        self.b2 = self.b * self.b
        self.e2 = 1.0 - (self.b2 / self.a2)
        self.e  = self.e2 / (1.0 - self.e2)
# Local ENU Origin
        self.latZero = None
        self.lonZero = None
        self.hgtZero = None
        self.xZero = None
        self.yZero = None
        self.zZero = None
        self.R = np.asmatrix(np.eye(3))
    
    def setENUorigin(self, lat, lon, height):
# Save origin lat, lon, height
        self.latZero = lat
        self.lonZero = lon
        self.hgtZero = height

# Get origin ECEF X,Y,Z
        origin = self.geo2ecef(self.latZero, self.lonZero, self.hgtZero)
        self.xZero = origin.item(0)
        self.yZero = origin.item(1)
        self.zZero = origin.item(2)
        self.oZero = np.array([[self.xZero], [self.yZero], [self.zZero]])

# Build rotation matrix
        phi = np.deg2rad(self.latZero)
        lmd = np.deg2rad(self.lonZero)

        cPhi = np.cos(phi)
        cLmd = np.cos(lmd)
        sPhi = np.sin(phi)
        sLmd = np.sin(lmd)

        self.R[0, 0] = -sLmd
        self.R[0, 1] =  cLmd
        self.R[0, 2] =  0.0
        self.R[1, 0] = -sPhi * cLmd
        self.R[1, 1] = -sPhi * sLmd
        self.R[1, 2] =  cPhi
        self.R[2, 0] =  cPhi * cLmd
        self.R[2, 1] =  cPhi * sLmd
        self.R[2, 2] =  sPhi

    def geo2ecef(self, lat, lon, height):
        phi = np.deg2rad(lat)
        lmd = np.deg2rad(lon)

        cPhi = np.cos(phi)
        cLmd = np.cos(lmd)
        sPhi = np.sin(phi)
        sLmd = np.sin(lmd)

        N = self.a / np.sqrt(1.0 - self.e2 * sPhi * sPhi)

        x = (N + height) * cPhi * cLmd
        y = (N + height) * cPhi * sLmd
        z = ((self.b2 / self.a2) * N + height) * sPhi

        return np.array([[x], [y], [z]])

    def ecef2enu(self, x, y, z):
        ecef = np.array([[x], [y], [z]])

        return self.R * (ecef - self.oZero)

    def geo2enu(self, lat, lon, height):
        ecef = self.geo2ecef(lat, lon, height)

        return self.ecef2enu(ecef.item(0), ecef.item(1), ecef.item(2))
    
    def ecef2geo(self, x, y, z):
        p = np.sqrt(x*x + y*y)
        q = np.arctan2(self.a * z, self.b * p)

        sq = np.sin(q)
        cq = np.cos(q)

        sq3 = sq * sq * sq
        cq3 = cq * cq * cq

        phi = np.arctan2(z + self.e * self.b * sq3, p - self.e2 * self.a * cq3)
        lmd = np.arctan2(y, x)
        v = self.a / np.sqrt(1.0 - self.e2 * np.sin(phi) * np.sin(phi))

        lat = np.rad2deg(phi)
        lon = np.rad2deg(lmd)
        h = (p / np.cos(phi)) - v

        return np.array([[lat], [lon], [h]])

    def enu2ecef(self, x, y, z):
        lmd = np.deg2rad(self.latZero)
        phi = np.deg2rad(self.lonZero)

        cPhi = np.cos(phi)
        cLmd = np.cos(lmd)
        sPhi = np.sin(phi)
        sLmd = np.sin(lmd)

        N = self.a / np.sqrt(1.0 - self.e2 * sLmd * sLmd)

        x0 = (self.hgtZero + N) * cLmd * cPhi
        y0 = (self.hgtZero + N) * cLmd * sPhi
        z0 = (self.hgtZero + (1.0 - self.e2) * N) * sLmd

        xd = -sPhi * x - cPhi * sLmd * y + cLmd * cPhi * z
        yd =  cPhi * x - sPhi * sLmd * y + cLmd * sPhi * z
        zd =  cLmd * y + sLmd * z

        return np.array([[x0+xd], [y0+yd], [z0+zd]])

    def enu2geo(self, x, y, z):
        ecef = self.enu2ecef(x, y, z)

        return self.ecef2geo(ecef.item(0), ecef.item(1), ecef.item(2))
#difference in position checker -- not particularly useful right now

'''
def diff():
    a_space = targlat - origlat
    print("lat difference: " + a_space)
    b_space = targlon - origlon
    print("lon difference: " + b_space)
    c_space = targalt - origalt
    print("alt difference: " + c_space)

'''