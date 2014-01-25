import urllib, json
import pprint
import numpy as np
import matplotlib.pyplot as plt
import random

def Bx(t, px, n):
    if n <= 0:
        return
    if n == 1:
        return px[0]
    else:
        return (1-t)*Bx(t, px[:n-1],n-1) + t*Bx(t, px[1:],n-1)
   
def By(t, py, n):
    if n == 1:
        return py[0]
    else:
        return (1-t)*By(t, py[:n-1],n-1) + t*By(t, py[1:],n-1)

def decode_line(encoded):
    encoded_len = len(encoded)
    index = 0
    array = []
    lat = 0
    lng = 0
    
    while index < encoded_len:

        b = 0
        shift = 0
        result = 0

        while True:
            b = ord(encoded[index]) - 63
            index = index + 1
            result |= (b & 0x1f) << shift
            shift += 5
            if b < 0x20:
                break

        if result & 1:
            dlat = ~(result >> 1)  
        else:
            dlat= result >> 1
        lat += dlat

        shift = 0
        result = 0

        while True:
            b = ord(encoded[index]) - 63
            index = index + 1
            result |= (b & 0x1f) << shift
            shift += 5
            if b < 0x20:
                break
        if result & 1:
            dlng = ~(result >> 1)  
        else:
            dlng=result >> 1
        lng += dlng

        array.append((lat * 1e-5, lng * 1e-5))

    return array

if __name__ == "__main__":
    URL2 = "http://maps.googleapis.com/maps/api/directions/json?origin=22.320996,%2087.311503&destination=22.318908,87.298554&sensor=false&mode=walking"
    lati= []
    longi= []
    max=0
    maxi=0
    min=10000
    mini=10000
    googleResponse = urllib.urlopen(URL2)
    jsonResponse = json.loads(googleResponse.read())
    s=''
    count=0
    for route in jsonResponse['routes']:
            for leg in route['legs']:
                for step in leg['steps']:
                    p= step['polyline']
                    for pnt in p['points']:
                        s+=pnt
                   
                    latlngs = decode_line(s)
                    s=""
                    for latlng in latlngs:
                        lati.append(latlng[0])
                        longi.append(latlng[1])
                        count+=1
                        if max<latlng[0]:
                            max=latlng[0]
                        if maxi<latlng[1]:
                            maxi=latlng[1]
                        if min>latlng[0]:
                            min=latlng[0]
                        if mini>latlng[1]:
                            mini=latlng[1]
    print str(count)
    for route in jsonResponse['routes']:
            for leg in route['legs']:
                for step in leg['steps']:
                    p= step['polyline']
                    for pnt in p['points']:
                        s+=pnt
                   
                    latlngs = decode_line(s)
                    s=""
                    for latlng in latlngs:
                        print str(latlng[0]) + "," + str(latlng[1])
    plt.plot(lati,longi, 'ro')
    plt.axis([min,max,mini,maxi])
    # plt.show()
    
