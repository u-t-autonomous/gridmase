"""
By using PyMASE, you agree to accept the following license agreement.

PyMASE (Beta Version) License Agreement

Contact: Ufuk Topcu, University of Texas at Austin (utopcu@utexas.edu).

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this 
list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, 
this list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

3. All advertising materials mentioning features or use of this software must
display the following acknowledgment: "This product includes software developed
by Ufuk Topcu in the Department of Aerospace Engineering and Engineering 
Mechanics at the University of Texas at Austin".

4. Neither the name of the University of Texas at Austin nor the names of its 
contributors may be used to endorse or promote products derived from this 
software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE UNIVERSITY OF TEXAS AT AUSTIN AND CONTRIBUTORS 
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, 
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
ARE DISCLAIMED. IN NO EVENT SHALL THE UNIVERSITY OF TEXAS AT AUSTIN OR CONTRIBUTORS 
BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE 
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) 
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT 
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""

import lmcp.LMCPObject
from math import pi
import inspect
from geopy.distance import vincenty
from lmcp import LMCPFactory
from afrl.cmasi import EntityState
from afrl.cmasi import AirVehicleState
from afrl.cmasi import CameraState
from afrl.cmasi import AirVehicleConfiguration
from afrl.cmasi.SessionStatus import SessionStatus
from afrl.cmasi.SimulationStatusType import SimulationStatusType
from afrl.cmasi.PointSearchTask import PointSearchTask
from afrl.cmasi import MissionCommand
from afrl.cmasi import Waypoint
from afrl.cmasi import AltitudeType
from afrl.cmasi import SpeedType
from afrl.cmasi import TurnType
import search
from search import deg2rad

def tokenize(string):
    keys = ['Type','UAV','Definition','Aux_UAV','Aux_Loc']
    return dict(zip(keys,string.split('_')))

def get_args(func):
    args = inspect.getargspec(func).args
    args = [arg for arg in args if arg != 'self']
    return args

def convert_heading(heading):

    heading = heading - 90

    if heading <= 0:
        heading = heading * (-1)
    else:
        heading = 360 - heading

    return heading


class Map:

    def __init__(self,xsize,ysize,blocklength,latcenter,longcenter):

        self.xsize = xsize
        self.ysize = ysize
        self.blocklength = blocklength
        self.latcenter = latcenter
        self.longcenter = longcenter
        self.latbase = latcenter
        self.longbase = longcenter

    def Coord2State(self, lat, longt):

        yaxis = abs(int(lat - self.latbase))/self.blocklength
        xaxis = abs(int(longt - self.longbase))/self.blocklength
        state = xaxis + (yaxis*self.xsize)
        return state;

    def AccurateCoord2State(self, lat, longt,prevs,ac=0.1):

        state = self.Coord2State(self,lat,longt)
        coord = self.State2Coord(self, state)
        upperlon = coord[1] + (((self.blocklength/200000000.))*ac)
        lowerlon = coord[1] - (((self.blocklength/200000000.))*ac)
        upperlat = coord[0] + (((self.blocklength/200000000.))*ac)
        lowerlat = coord[0] - (((self.blocklength/200000000.))*ac)

        if (lat >= lowerlat and lat <= upperlat and longt >= lowerlon and longt <= upperlon):
            return state
        else:
            return prevs

    def State2Coord(self, state):

        xaxis = state % self.xsize
        yaxis = state / self.xsize
        lat = self.latbase + (self.blocklength/2.) + (yaxis * self.blocklength ) 
        lat = float(lat) / 100000000.
        longt = self.longtbase + (self.blocklength/2.) + (xaxis * self.blocklength ) 
        longt = float(longt) / 100000000.
        return (lat,longt,50)

def toString(self):
        return "Location {}: Latitude: {} Longitude: {} Height: {} Width: {}"\
        .format(self.name,self.center_lat,self.center_lon,self.height,self.width)

class Location:

    def __init__(self,lat=0,lon=0,height=0,width=0,name=''):

        self.name = name
        self.center_lat = lat
        self.center_lon = lon
        self.height = height
        self.width = width

    def get_center_lat(self):
        return self.center_lat

    def get_center_lon(self):
        return self.center_lon
    

class UAV:

    def __init__(self,id,sock,stateMap,d_w = 0.005,d_t = 0,d_l = 0,turning_radius = .003):

        self.id = id
        self.commandr = MissionCommand.MissionCommand()
        self.commandt = MissionCommand.MissionCommand()
        self.commandf = MissionCommand.MissionCommand()
        self.commands = MissionCommand.MissionCommand()
        self.rloc = []
        self.sloc = []
        self.roam_log = 0
        self.refueling = 0
        self.searching = 0
        self.search_log = 0
        self.fuel_sensor = 1
        self.target_sensor = 0
        self.altitudeType = AltitudeType.AltitudeType.MSL
        self.turnType = TurnType.TurnType.TurnShort
        self.speedType = SpeedType.SpeedType.Airspeed
        self.sock = sock
        self.stateMap = stateMap
        self.state = 'idle'
        self.d_w = d_w
        self.d_t = d_t
        self.d_l = d_l
        self.turning_radius = turning_radius
        self.last_wp = None
        self.current_wp = None

    def getit(self,st,id):

        if id in list(self.stateMap.keys()):
            if st=='energy':
                try:
                    return self.stateMap.get(id).get_EnergyAvailable()
                except AttributeError:
                    return 100
            elif st=='status':
                return self.stateMap.get(id).get_PayloadStateList()
            elif st=='longitude':
                return self.stateMap.get(id).get_Location().get_Longitude()
            elif st=='latitude':
                return self.stateMap.get(id).get_Location().get_Latitude()
            elif st=='altitude':
                return self.stateMap.get(id).get_Location().get_Altitude()
            else:
                return 0
        else:
            return 0

    def point_search(self,location,X=0,xc=0,xr=0,s=0, step_size = 0.003):

        if self.state != 'point_search' or True:
            self.state = 'point_search'
            heading = self.stateMap.get(self.id).get_Heading()
            longtitude = self.stateMap.get(self.id).get_Location().get_Longitude()
            latitude = self.stateMap.get(self.id).get_Location().get_Latitude()
            pre_ve = (longtitude,latitude,heading)
            p = (location.center_lon,location.center_lat,deg2rad(X))
            pre_ve = (pre_ve[0],pre_ve[1],deg2rad(convert_heading(pre_ve[2])))
            paths = search.point_search(pre_ve = pre_ve, p = p,xc=deg2rad(xc),xr=deg2rad(xr),s=s,d_t=self.d_t\
                , d_l = self.d_l, turning_radius = self.turning_radius, step_size = step_size)

            self.commandr = MissionCommand.MissionCommand()
            self.commandr.set_FirstWaypoint(0)
            self.commandr.set_VehicleID(self.id)
            self.commandr.set_CommandID(0)

            self.rloc = []

            for path in paths:
                self.rloc.append(Location(path[1],path[0],0,0))

            nextWaypoint = Waypoint.Waypoint()

            for i in range(len(self.rloc)):
                nextWaypoint = Waypoint.Waypoint()
                nextWaypoint.set_Number(i)
                nextWaypoint.set_NextWaypoint(i+1)
                nextWaypoint.set_Speed(20)
                nextWaypoint.set_SpeedType(self.speedType)
                nextWaypoint.set_ClimbRate(0)
                nextWaypoint.set_TurnType(self.turnType)
                nextWaypoint.set_Longitude(self.rloc[i].center_lon)  
                nextWaypoint.set_Altitude(50)
                nextWaypoint.set_Latitude(self.rloc[i].center_lat)
                nextWaypoint.set_AltitudeType(self.altitudeType)
                self.commandr.get_WaypointList().append(nextWaypoint)

            self.sock.send(LMCPFactory.packMessage(self.commandr, True))
            return i

    def line_search(self,line, angles=None, smoothing = 10, step_size = 0.5, b = False):

        if self.state != 'line_search':
            self.state = 'line_search'
            heading = self.stateMap.get(self.id).get_Heading()
            longtitude = self.stateMap.get(self.id).get_Location().get_Longitude()
            latitude = self.stateMap.get(self.id).get_Location().get_Latitude()
            pre_ve = (longtitude,latitude,heading)
            nextWaypoint = Waypoint.Waypoint()
            paths = search.line_search(smoothing = smoothing, pre_ve = pre_ve, line = line\
                , angles = deg2rad(angles), turning_radius = self.turning_radius, step_size = step_size, b = False)
            
            self.commandr = MissionCommand.MissionCommand()
            self.commandr.set_FirstWaypoint(0)
            self.commandr.set_VehicleID(self.id)
            self.commandr.set_CommandID(0)

            self.rloc = []

            for path in paths:
                self.rloc.append(Location(path[1],path[0],0,0))

            for i in range(len(self.rloc)):
                nextWaypoint = Waypoint.Waypoint()
                nextWaypoint.set_Number(i)
                nextWaypoint.set_NextWaypoint(i+1)
                nextWaypoint.set_Speed(20)
                nextWaypoint.set_SpeedType(self.speedType)
                nextWaypoint.set_ClimbRate(0)
                nextWaypoint.set_TurnType(self.turnType)
                nextWaypoint.set_Longitude(self.rloc[i].center_lon)  
                nextWaypoint.set_Altitude(50)
                nextWaypoint.set_Latitude(self.rloc[i].center_lat)
                nextWaypoint.set_AltitudeType(self.altitudeType)
                self.commandr.get_WaypointList().append(nextWaypoint)


            self.sock.send(LMCPFactory.packMessage(self.commandr, True))
            return i

    def spiral_search(self, location, extent=.03, alpha=0.9, step = 10, step_size = 0.003):

        if self.state != 'spiral_search'+location.name:
            self.state = 'spiral_search'+location.name
            heading = self.stateMap.get(self.id).get_Heading()
            longtitude = self.stateMap.get(self.id).get_Location().get_Longitude()
            latitude = self.stateMap.get(self.id).get_Location().get_Latitude()
            pre_ve = (longtitude,latitude,heading)
            p = (location.center_lon,location.center_lat,0)
            paths = search.spiral_search(pre_ve = pre_ve, p = p,d_w = self.d_w, d_l = self.d_l\
                , alpha = alpha, extent = extent, step = deg2rad(step), turning_radius = .001\
                , step_size = step_size)
            
            self.rloc = []

            self.commandr = MissionCommand.MissionCommand()
            self.commandr.set_FirstWaypoint(0)
            self.commandr.set_VehicleID(self.id)
            self.commandr.set_CommandID(0)

            for path in paths:
                self.rloc.append(Location(path[1],path[0],0,0))

            nextWaypoint = Waypoint.Waypoint()

            for i in range(len(self.rloc)):
                nextWaypoint = Waypoint.Waypoint()
                nextWaypoint.set_Number(i)
                nextWaypoint.set_NextWaypoint(i+1)
                nextWaypoint.set_Speed(20)
                nextWaypoint.set_SpeedType(self.speedType)
                nextWaypoint.set_ClimbRate(0)
                nextWaypoint.set_TurnType(self.turnType)
                nextWaypoint.set_Longitude(self.rloc[i].center_lon)  
                nextWaypoint.set_Altitude(50)
                nextWaypoint.set_Latitude(self.rloc[i].center_lat)
                nextWaypoint.set_AltitudeType(self.altitudeType)
                self.commandr.get_WaypointList().append(nextWaypoint)

            self.sock.send(LMCPFactory.packMessage(self.commandr, True))
            return i

    def area_search(self,location=None,angle=0,alpha=0.9,p=None,xc=0,xr=0,s=0, step_size = 0.005):
        
        if self.state != 'area_search'+location.name:
            self.state = 'area_search'+location.name
            nextWaypoint = Waypoint.Waypoint()
            heading = self.stateMap.get(self.id).get_Heading()
            longtitude = self.stateMap.get(self.id).get_Location().get_Longitude()
            latitude = self.stateMap.get(self.id).get_Location().get_Latitude()
            pre_ve = (longtitude,latitude,heading)
            poly = [(location.center_lon - ((9*(10**(-6))*location.width)/2)
                ,location.center_lat - ((9*(10**(-6))*location.height)/2))

                ,(location.center_lon - ((9*(10**(-6))*location.width)/2)
                    ,location.center_lat + ((9*(10**(-6))*location.height)/2))

                ,(location.center_lon + ((9*(10**(-6))*location.width)/2)
                    ,location.center_lat - ((9*(10**(-6))*location.height)/2))

                ,(location.center_lon + ((9*(10**(-6))*location.width)/2)
                    ,location.center_lat + ((9*(10**(-6))*location.height)/2))]

            paths = search.area_search(angle = deg2rad(angle) ,pre_ve = pre_ve, poly = poly\
                ,xc=deg2rad(xc),xr=deg2rad(xr),
                s=s,d_t=self.d_t, p = p, d_w = self.d_w, d_l = self.d_l,alpha = alpha\
                , turning_radius = self.turning_radius, step_size = step_size)
            
            self.commandr = MissionCommand.MissionCommand()
            self.commandr.set_VehicleID(self.id)
            self.commandr.set_CommandID(0)
            self.commandr.set_FirstWaypoint(0)

            self.rloc = []
            for path in paths:
                self.rloc.append(Location(path[1],path[0],0,0))            

            for i in range(len(self.rloc)):
                nextWaypoint = Waypoint.Waypoint()
                nextWaypoint.set_Number(i)
                nextWaypoint.set_NextWaypoint(i+1)
                nextWaypoint.set_Speed(20)
                nextWaypoint.set_SpeedType(self.speedType)
                nextWaypoint.set_ClimbRate(0)
                nextWaypoint.set_TurnType(self.turnType)
                nextWaypoint.set_Longitude(self.rloc[i].center_lon)  
                nextWaypoint.set_Altitude(50)
                nextWaypoint.set_Latitude(self.rloc[i].center_lat)
                nextWaypoint.set_AltitudeType(self.altitudeType)
                self.commandr.get_WaypointList().append(nextWaypoint)

            self.sock.send(LMCPFactory.packMessage(self.commandr, True))
            return i

    def loiter(self,location=None,pre_ve=None,angle=0,alpha=0.9,p=None,xc=0,xr=0,s=0, step_size = 0.003):
        
        if self.state != 'loiter':
            self.state = 'loiter'
            nextWaypoint = Waypoint.Waypoint()
            heading = self.stateMap.get(self.id).get_Heading()
            longtitude = self.stateMap.get(self.id).get_Location().get_Longitude()
            latitude = self.stateMap.get(self.id).get_Location().get_Latitude()
            pre_ve = (longtitude,latitude,heading)
            poly = [(location.center_lon - ((9*(10**(-6))*location.width)/2)
                ,location.center_lat - ((9*(10**(-6))*location.height)/2))

                ,(location.center_lon - ((9*(10**(-6))*location.width)/2)
                    ,location.center_lat + ((9*(10**(-6))*location.height)/2))

                ,(location.center_lon + ((9*(10**(-6))*location.width)/2)
                    ,location.center_lat - ((9*(10**(-6))*location.height)/2))

                ,(location.center_lon + ((9*(10**(-6))*location.width)/2)
                    ,location.center_lat + ((9*(10**(-6))*location.height)/2))]

            paths = search.area_search(angle = deg2rad(angle) ,pre_ve = pre_ve, poly = poly,xc=deg2rad(xc),xr=deg2rad(xr),
                s=s,d_t=self.d_t, p = p, d_w = self.d_w, d_l = self.d_l,alpha = alpha\
                , turning_radius = self.turning_radius, step_size = step_size)
            
            self.commandr = MissionCommand.MissionCommand()
            self.commandr.set_VehicleID(self.id)
            self.commandr.set_CommandID(0)
            self.commandr.set_FirstWaypoint(0)

            self.rloc = []
            for path in paths:
                self.rloc.append(Location(path[1],path[0],0,0))            

            for i in range(len(self.rloc)):
                nextWaypoint = Waypoint.Waypoint()
                nextWaypoint.set_Number(i)
                if i+1 == len(self.rloc):
                    nextWaypoint.set_NextWaypoint(1)
                else:
                    nextWaypoint.set_NextWaypoint(i+1)
                nextWaypoint.set_Speed(20)
                nextWaypoint.set_SpeedType(self.speedType)
                nextWaypoint.set_ClimbRate(0)
                nextWaypoint.set_TurnType(self.turnType)
                nextWaypoint.set_Longitude(self.rloc[i].center_lon)  
                nextWaypoint.set_Altitude(50)
                nextWaypoint.set_Latitude(self.rloc[i].center_lat)
                nextWaypoint.set_AltitudeType(self.altitudeType)
                self.commandr.get_WaypointList().append(nextWaypoint)

            self.sock.send(LMCPFactory.packMessage(self.commandr, True))
            return i


    def roam(self,location):

        nextWaypoint = Waypoint.Waypoint()

        if self.state != 'roaming':
            counter = 0
            self.commandr = MissionCommand.MissionCommand()
            self.roam_log = 0
        else:
            counter = len(self.commandr.get_WaypointList())
        self.state = 'roaming'

        if counter%5 == 0:
            self.commandr = MissionCommand.MissionCommand()

        if self.roam_log == 0:
            self.rloc.append(Location(location.center_lat+float(location.height)/261538.0\
                ,location.center_lon+float(location.height)/261538.0,0,0))
            self.rloc.append(Location(location.center_lat+float(location.height)/261538.0\
                ,location.center_lon-float(location.height)/261538.0,0,0))
            self.rloc.append(Location(location.center_lat-float(location.height)/261538.0\
                ,location.center_lon+float(location.height)/261538.0,0,0))
            self.rloc.append(Location(location.center_lat,location.center_lon,0,0))
            self.rloc.append(Location(location.center_lat-float(location.height)/261538.0\
                ,location.center_lon-float(location.height)/261538.0,0,0))
        
        if self.roam_log == 0:
            nextWaypoint = Waypoint.Waypoint()
            self.commandr.set_FirstWaypoint(counter)
            nextWaypoint.set_Number(counter)
            nextWaypoint.set_NextWaypoint(counter+1)
            nextWaypoint.set_Speed(20)
            nextWaypoint.set_SpeedType(self.speedType)
            nextWaypoint.set_ClimbRate(0)
            nextWaypoint.set_TurnType(self.turnType)
            nextWaypoint.set_Longitude(self.rloc[self.roam_log%5].center_lon)  
            nextWaypoint.set_Altitude(50)
            nextWaypoint.set_Latitude(self.rloc[self.roam_log%5].center_lat)
            nextWaypoint.set_AltitudeType(self.altitudeType)
            self.commandr.get_WaypointList().append(nextWaypoint)
            self.commandr.set_VehicleID(self.id)
            self.commandr.set_CommandID(0)
            self.sock.send(LMCPFactory.packMessage(self.commandr, True))
            self.roam_log += 1

        if vincenty((self.getit('latitude',self.id),self.getit('longitude',self.id))\
            , (self.rloc[(self.roam_log-1)%5].center_lat,self.rloc[(self.roam_log-1)%5].center_lon)).meters < 50:
            nextWaypoint = Waypoint.Waypoint()
            self.commandr.set_FirstWaypoint(counter)
            nextWaypoint.set_Number(counter)
            nextWaypoint.set_NextWaypoint(counter+1)
            nextWaypoint.set_Speed(20)
            nextWaypoint.set_SpeedType(self.speedType)
            nextWaypoint.set_ClimbRate(0)
            nextWaypoint.set_TurnType(self.turnType)
            nextWaypoint.set_Longitude(self.rloc[self.roam_log%5].center_lon)  
            nextWaypoint.set_Altitude(50)
            nextWaypoint.set_Latitude(self.rloc[self.roam_log%5].center_lat)
            nextWaypoint.set_AltitudeType(self.altitudeType)
            self.commandr.get_WaypointList().append(nextWaypoint)
            self.commandr.set_VehicleID(self.id)
            self.commandr.set_CommandID(0)
            self.sock.send(LMCPFactory.packMessage(self.commandr, True))
            self.roam_log += 1

    def refuel(self,location):

        if self.state != 'refueling':
            nextWaypoint = Waypoint.Waypoint()
            nextWaypoint.set_Number(1)
            self.commandf.set_FirstWaypoint(1)
            nextWaypoint.set_NextWaypoint(3)
            nextWaypoint.set_Speed(20)
            nextWaypoint.set_SpeedType(self.speedType)
            nextWaypoint.set_ClimbRate(0)
            nextWaypoint.set_TurnType(self.turnType)
            nextWaypoint.set_Longitude(location.center_lon)
            nextWaypoint.set_Altitude(50)
            nextWaypoint.set_Latitude(location.center_lat)
            nextWaypoint.set_AltitudeType(self.altitudeType)
            self.commandf.get_WaypointList().append(nextWaypoint)
            self.commandf.set_VehicleID(self.id)
            self.commandf.set_CommandID(0)
        self.state = 'refueling'

        if self.refueling < 1 and vincenty((location.center_lat,location.center_lon),(self.getit('latitude',self.id)\
            ,self.getit('longitude',self.id))).meters > location.height:
            self.sock.send(LMCPFactory.packMessage(self.commandf, True))
            self.refueling += 1

        if vincenty((location.center_lat,location.center_lon),(self.getit('latitude',self.id),self.getit('longitude'\
            ,self.id))).meters < location.height:  
            entity = AirVehicleState.AirVehicleState()
            entity.set_Location(self.stateMap.get(self.id).get_Location())
            entity.set_EnergyAvailable(100)
            entity.set_ID(self.id)
            this = LMCPFactory.packMessage(entity, True)
            self.sock.send(this)
            self.refueling = 0

    def trackTarget(self,id2):

        self.state = 'tracking'
        counter = len(self.commandt.get_WaypointList())

        if counter%5 == 0:
            self.commandt = MissionCommand.MissionCommand()

        nextWaypoint = Waypoint.Waypoint();
        nextWaypoint.set_Number(counter)
        self.commandt.set_FirstWaypoint(counter)
        nextWaypoint.set_NextWaypoint(counter+1)
        nextWaypoint.set_Speed(20)
        nextWaypoint.set_SpeedType(self.speedType)
        nextWaypoint.set_ClimbRate(0)
        nextWaypoint.set_TurnType(self.turnType)
        nextWaypoint.set_Longitude(self.getit('longitude',id2))
        nextWaypoint.set_Altitude(50)
        nextWaypoint.set_Latitude(self.getit('latitude',id2))
        nextWaypoint.set_AltitudeType(self.altitudeType)

        if len(self.commandt.get_WaypointList()) <= 1:
            #self.commandt.WaypointList = []
            self.commandt.get_WaypointList().append(nextWaypoint)
            self.commandt.set_VehicleID(self.id)
            self.commandt.set_CommandID(0)
            self.sock.send(LMCPFactory.packMessage(self.commandt, True)) 

        elif len(self.commandt.get_WaypointList()) > 1:
            if (abs(self.commandt.get_WaypointList()[-1].get_Latitude() - nextWaypoint.get_Latitude())>0.00001\
             or abs(self.commandt.get_WaypointList()[-1].get_Longitude() - nextWaypoint.get_Longitude()) >0.00001)\
              and vincenty((self.commandt.get_WaypointList()[-1].get_Latitude(),self.commandt.get_WaypointList()[-1]\
                .get_Longitude()), (nextWaypoint.get_Latitude(),nextWaypoint.get_Longitude())).meters > 300:
                self.commandt.get_WaypointList().append(nextWaypoint)
                self.commandt.set_VehicleID(self.id)
                self.commandt.set_CommandID(0)
                self.sock.send(LMCPFactory.packMessage(self.commandt, True))   
        
    def searchA(self,location):

        self.state = 'searching'
        counter = len(self.commands.get_WaypointList())

        if self.searching == 0:
            self.sloc.append(Location(location.center_lat,location.center_lon,0,0))
            self.sloc.append(Location(location.center_lat+0.01,location.center_lon+0.0155,0,0))
            self.sloc.append(Location(location.center_lat-0.01,location.center_lon+0.0155,0,0))
            self.sloc.append(Location(location.center_lat+0.01,location.center_lon-0.0155,0,0))
            self.sloc.append(Location(location.center_lat-0.01,location.center_lon-0.0155,0,0))
            nextWaypoint = Waypoint.Waypoint()
            self.commands.set_FirstWaypoint(counter)
            nextWaypoint.set_Number(counter)
            nextWaypoint.set_NextWaypoint(counter+1)
            nextWaypoint.set_Speed(20)
            nextWaypoint.set_SpeedType(self.speedType)
            nextWaypoint.set_ClimbRate(0)
            nextWaypoint.set_TurnType(self.turnType)
            nextWaypoint.set_Longitude(self.sloc[self.search_log%4].center_lon)  
            nextWaypoint.set_Altitude(50)
            nextWaypoint.set_Latitude(self.sloc[self.search_log%4].center_lat)
            nextWaypoint.set_AltitudeType(self.altitudeType)
            self.commands.get_WaypointList().append(nextWaypoint)
            self.commands.set_VehicleID(self.id)
            self.commands.set_CommandID(0)
            self.sock.send(LMCPFactory.packMessage(self.commands, True))
            self.search_log += 1

        if vincenty((self.getit('latitude',self.id),self.getit('longitude',self.id)),\
         (self.sloc[self.search_log-1%4].center_lat,self.sloc[self.search_log-1%4].center_lon)).meters < 50:
            nextWaypoint = Waypoint.Waypoint()
            self.commands.set_FirstWaypoint(counter)
            nextWaypoint.set_Number(counter)
            nextWaypoint.set_NextWaypoint(counter+1)
            nextWaypoint.set_Speed(20)
            nextWaypoint.set_SpeedType(self.speedType)
            nextWaypoint.set_ClimbRate(0)
            nextWaypoint.set_TurnType(self.turnType)
            nextWaypoint.set_Longitude(self.sloc[self.search_log%4].center_lon)  
            nextWaypoint.set_Altitude(50)
            nextWaypoint.set_Latitude(self.sloc[self.search_log%4].center_lat)
            nextWaypoint.set_AltitudeType(self.altitudeType)
            self.commands.get_WaypointList().append(nextWaypoint)
            self.commands.set_VehicleID(self.id)
            self.commands.set_CommandID(0)
            self.sock.send(LMCPFactory.packMessage(self.commands, True))
            self.search_log += 1

    def get_energy(self):

        if len(self.stateMap)>0:
            try:
                return self.stateMap.get(self.id).get_EnergyAvailable()
            except AttributeError:
                return 100
        else:
            return 'wait'