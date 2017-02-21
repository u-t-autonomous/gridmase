from code_synth_ctrl import CodeSynthCtrl
import inspect

def tokenize(string):
    keys = ['Type','UAV','Definition','Aux_UAV','Aux_Loc']
    return dict(zip(keys,string.split('_')))

def stringify(lst):
    s = "_"
    return s.join(lst)

def get_args(func):
    args = inspect.getargspec(func).args
    args = [arg for arg in args if arg != 'self']
    return args

def make_xml(n_uav,uav_lat,uav_lon,n_loc,loc_lat,loc_lon,width,height):

    maplat = sum(uav_lat)/float(len(uav_lat))
    maplon = sum(uav_lon)/float(len(uav_lon))

    xmlscript = ('<?xml version="1.0" ?>\n'
    '<AMASE>\n'
      '<ScenarioData>\n'
        '<SimulationView LongExtent=".07" Latitude="{lat}" Longitude="{lon}"/>\n'
        '<ScenarioName>Test Scenario</ScenarioName>\n'
        '<Date>24/04/2008:00:00:00</Date>\n'
        '<ScenarioDuration>60000</ScenarioDuration>\n'
      '</ScenarioData>\n'
      '<ScenarioEventList>\n').format(lat = maplat, lon = maplon)

    for i in range(0,n_uav):
        xmlscript += ('<AirVehicleConfiguration Time="0.0" Series="CMASI">\n'
          '<ID>{id}</ID>\n'
          '<Label>UAV{id}</Label>\n'
          '<MinimumSpeed>15.0</MinimumSpeed>\n'
          '<MaximumSpeed>35.0</MaximumSpeed>\n'
          '<NominalFlightProfile>\n'
           ' <FlightProfile Series="CMASI">\n'
            '  <Name>Cruise</Name>\n'
             ' <Airspeed>20.0</Airspeed>\n'
              '<PitchAngle>0.0</PitchAngle>\n'
              '<VerticalSpeed>0.0</VerticalSpeed>\n'
              '<MaxBankAngle>30.0</MaxBankAngle>\n'
              '<EnergyRate>0.005</EnergyRate>\n'
            '</FlightProfile>\n'
          '</NominalFlightProfile>\n'
          '<AlternateFlightProfiles>\n'
            '<FlightProfile Series="CMASI">\n'
              '<Name>Climb</Name>\n'
              '<Airspeed>15.0</Airspeed>\n'
              '<PitchAngle>10.0</PitchAngle>\n'
              '<VerticalSpeed>5.0</VerticalSpeed>\n'
              '<MaxBankAngle>30.0</MaxBankAngle>\n'
              '<EnergyRate>0.01</EnergyRate>\n'
            '</FlightProfile>\n'
            '<FlightProfile Series="CMASI">\n'
              '<Name>Descent</Name>\n'
              '<Airspeed>25.0</Airspeed>\n'
              '<PitchAngle>-5.0</PitchAngle>\n'
              '<VerticalSpeed>-5.0</VerticalSpeed>\n'
              '<MaxBankAngle>30.0</MaxBankAngle>\n'
              '<EnergyRate>0.005</EnergyRate>\n'
            '</FlightProfile>\n'
            '<FlightProfile Series="CMASI">\n'
              '<Name>Loiter</Name>\n'
              '<Airspeed>20.0</Airspeed>\n'
              '<PitchAngle>5.0</PitchAngle>\n'
              '<VerticalSpeed>0.0</VerticalSpeed>\n'
              '<MaxBankAngle>30.0</MaxBankAngle>\n'
              '<EnergyRate>0.005</EnergyRate>\n'
            '</FlightProfile>\n'
            '<FlightProfile Series="CMASI">\n'
              '<Name>Dash</Name>\n'
              '<Airspeed>35.0</Airspeed>\n'
              '<PitchAngle>-2.0</PitchAngle>\n'
              '<VerticalSpeed>0.0</VerticalSpeed>\n'
              '<MaxBankAngle>30.0</MaxBankAngle>\n'
              '<EnergyRate>0.01</EnergyRate>\n'
            '</FlightProfile>\n'
          '</AlternateFlightProfiles>\n'
          '<AvailableTurnTypes>\n'
            '<TurnType>TurnShort</TurnType>\n'
            '<TurnType>FlyOver</TurnType>\n'
          '</AvailableTurnTypes>\n'
          '<MinimumAltitude>0.0</MinimumAltitude>\n'
          '<MaximumAltitude>1000000.0</MaximumAltitude>\n'
          '<MinAltAboveGround>0.0</MinAltAboveGround>\n'
        '</AirVehicleConfiguration>\n'
        '<AirVehicleState Time="0.0" Series="CMASI">\n'
          '<ID>{id}</ID>\n'
          '<Location>\n'
            '<Location3D Series="CMASI">\n'
              '<Altitude>50.0</Altitude>\n'
              '<Latitude>{lat}</Latitude>\n'
              '<Longitude>{lon}</Longitude>\n'
            '</Location3D>\n'
          '</Location>\n'
          '<u>0.0</u>\n'
          '<v>0.0</v>\n'
          '<w>0.0</w>\n'
          '<udot>0.0</udot>\n'
          '<vdot>0.0</vdot>\n'
          '<wdot>0.0</wdot>\n'
          '<Heading>90.0</Heading>\n'
          '<Pitch>0.0</Pitch>\n'
          '<Roll>0.0</Roll>\n'
          '<p>0.0</p>\n'
          '<q>0.0</q>\n'
          '<r>0.0</r>\n'
          '<Airspeed>0.0</Airspeed>\n'
          '<VerticalSpeed>0.0</VerticalSpeed>\n'
          '<ActualEnergyRate>0.00008</ActualEnergyRate>\n'
          '<EnergyAvailable>100.0</EnergyAvailable>\n'
          '<WindSpeed>0.0</WindSpeed>\n'
          '<WindDirection>0.0</WindDirection>\n'
          '<GroundSpeed>0.0</GroundSpeed>\n'
          '<GroundTrack>0.0</GroundTrack>\n'
          '<CurrentWaypoint>0</CurrentWaypoint>\n'
          '<CurrentCommand>0</CurrentCommand>\n'
          '<Mode>Waypoint</Mode>\n'
          '<AssociatedTasks/>\n'
          '<Time>0</Time>\n'
        '</AirVehicleState>\n').format(id=i+1,lat=uav_lat[i],lon=uav_lon[i])

    for i in range(0,n_loc):
        xmlscript +=('<AreaSearchTask Series="CMASI">\n'
          '<SearchArea>\n'
        '<Rectangle Series="CMASI">\n'
          '<CenterPoint>\n'
            '<Location3D Series="CMASI">\n'
              '<Latitude>{lat}</Latitude>\n'
             ' <Longitude>{lon}</Longitude>\n'
            '</Location3D>\n'
          '</CenterPoint>\n'
          '<Width>{width}</Width>\n'
          '<Height>{height}</Height>\n'
          '<Rotation>0.0</Rotation>\n'
        '</Rectangle>\n'
      '</SearchArea>\n'
        '<ViewAngleList/>\n'
        '<DesiredWavelengthBands/>\n'
        '<DwellTime>0</DwellTime>\n'
        '<GroundSampleDistance>0.0</GroundSampleDistance>\n'
        '<TaskID>{taskid}</TaskID>\n'
        '<Label/>\n'
        '<RevisitRate>0.0</RevisitRate>\n'
        '<Parameters/>\n'
        '<Priority>0</Priority>\n'
        '<Required>false</Required>\n'
      '</AreaSearchTask>\n').format(lat=loc_lat[i],lon=loc_lon[i],width=width[i],height=height[i],taskid = 10 + i)

    xmlscript +=('</ScenarioEventList>\n'
      '</AMASE>\n').format()

    text_file = open("auto_generated/auto_code.xml", "w+")
    text_file.write("%s" % xmlscript)
    text_file.close()


def get_spec_file():

    commands = []
    keys = ['n_loc','n_uav','loc_lat','loc_lon','height','width','uav_lat','uav_lon','commands']
    output = dict(zip(keys, [None]*len(keys)))

    with open('spec.txt','r') as f:
        for x in f:
            x = x.split()
            if len(x[0]) > 2:
                if x[0] == 'n_uav':
                    output['n_uav'] = int(x[1])
                    output['uav_lat'] = [0]*output['n_uav']
                    output['uav_lon'] = [0]*output['n_uav']
                if x[0] == 'n_loc':
                    output['n_loc'] = int(x[1])
                    output['loc_lat'] = [0]*output['n_loc']
                    output['loc_lon'] = [0]*output['n_loc']
                    output['height'] = [0]*output['n_loc']
                    output['width'] = [0]*output['n_loc']
                if x[0] == 'loc':
                    output['loc_lat'][int(x[1]) - 1] = float(x[2])
                    output['loc_lon'][int(x[1]) - 1] = float(x[3])
                    output['height'][int(x[1]) - 1] = float(x[4])
                    output['width'][int(x[1]) - 1] = float(x[5])
                if x[0] == 'uav':
                    output['uav_lat'][int(x[1]) - 1] = float(x[2])
                    output['uav_lon'][int(x[1]) - 1] = float(x[3])
                if x[0] == 'cmd':
                    commands.append(x[1:])

    output['commands'] = map(stringify,commands)

    return output


def make_script(n_uav, n_loc, height, width, loc_lon, loc_lat,ctrl_input):


    output = csynth.move(**ctrl_input)
    ctrl_in = [key for key in ctrl_input.keys() if ctrl_input[key] == 1] 
    output = [key for key in output.keys() if output[key] == 1]
    output_input = output + ctrl_in
    log = []

    script = (
        'import socket{sep}'
        'from geopy.distance import vincenty{sep}'
        'from lmcp import LMCPFactory{sep}'
        'from afrl.cmasi import EntityState{sep}'
        'from afrl.cmasi import AirVehicleState{sep}'
        'from afrl.cmasi import AirVehicleConfiguration{sep}'
        'from afrl.cmasi.SessionStatus import SessionStatus{sep}'
        'from demo_controller import ExampleCtrl{sep}'
        'from PyMASE import UAV, Location, get_args{sep}'
        'import string{sep}'
        '{sep}stateMap = dict(){sep}'
        'M1 = ExampleCtrl(){sep}'
        'configMap = dict(){sep}'
        'move_keys = get_args(M1.move){sep}'
        'ctrl_input = {{key: 0 for key in move_keys}}{sep}').format(sep='\n', ind='    ') 
    
    script += (
        '{sep}def prepare_ctrl_input(UAVs,ctrl_input_args,current_play):{sep}'
        '{sep}{ind}ctrl_input = dict(zip(ctrl_input_args, [0]*len(ctrl_input_args))){sep}').format(sep='\n', ind='    ') 

    for string in output_input:

        token = tokenize(string)
        script += ('{sep}{ind}if "{key_type}" == "M":{sep}'
                '{ind}{ind}ctrl_input["{key}"] = UAVs[{idl}].{key_def}{sep}'
                '{sep}{ind}if "{key_type}" == "P":{sep}'
                '{sep}{ind}{ind}if current_play == "{key}":{sep}'
                '{ind}{ind}{ind}ctrl_input["{key}"] = 1{sep}'
                '{sep}{ind}{ind}else:{sep}'
                '{ind}{ind}{ind}ctrl_input["{key}"] = 0{sep}'
                '').format(sep='\n', ind='    ', key=string, key_def=token["Definition"]
                , key_type=token["Type"],idl=str(int(token["UAV"])-1)) 

    script += (
        '{ind}return ctrl_input{sep}').format(sep='\n', ind='    ') 

    for string in output:

        token = tokenize(string)

        if token["Type"] == 'M':          

            if token["Type"]+token["Definition"] not in log:

                log.append(token["Type"]+token["Definition"])

                if token["Definition"] == "Fuel":

                    script += (
                    '{sep}def {monitor}_monitor(uav,uav2,loc):{sep}'
                    '{ind}fuel = uav.get_energy(){sep}'
                    '{ind}if fuel <= 90 and fuel != 0:{sep}'
                    '{ind}{ind}uav.Fuel = 0{sep}'
                    '{ind}elif fuel > 90:{sep}'
                    '{ind}{ind}uav.Fuel = 1{sep}'
                    '{ind}return uav{sep}'
                    ).format(sep='\n', ind='    ', id=token["UAV"], monitor = token["Definition"]\
                    , aux1 = token['Aux_UAV'], aux2 = token['Aux_Loc']) 


                if token["Definition"] == "Found":

                    script += (
                    '{sep}def {monitor}_monitor(uav,uav2,loc):{sep}'
                    '{ind}dist = vincenty((uav.getit("latitude",uav.id),uav.getit("longitude",uav.id)),(uav.getit("latitude",uav2+1),uav.getit("longitude",uav2+1))).meters{sep}'
                    '{ind}if dist < 600 and dist != 0:{sep}'
                    '{ind}{ind}uav.Found = 1{sep}'
                    '{ind}else:{sep}'
                    '{ind}{ind}uav.Found = 0{sep}'
                    '{ind}return uav{sep}'
                    ).format(sep='\n', ind='    ', id=token["UAV"], monitor = token["Definition"]\
                    , aux1 = token['Aux_UAV'], aux2 = token['Aux_Loc']) 


        if token["Type"] == 'B':

            if token["Type"]+token["Definition"] not in log:

                log.append(token["Type"]+token["Definition"])

                script += (
                '{sep}def {behaviour}(uav,uav2,loc):{sep}'
                '{ind}return 0{sep}'
                ).format(sep='\n', ind='    ', id=token["UAV"], behaviour = token["Definition"]\
                , aux1 = token['Aux_UAV'], aux2 = token['Aux_Loc']) 

    script +=(

        '{sep}def message_received(obj):{sep}'
        '{ind}global stateMap{sep}'
        '{ind}global configMap{sep}'
        '{ind}if isinstance(obj ,AirVehicleConfiguration.AirVehicleConfiguration):{sep}'
        '{ind}{ind}configMap[obj.get_ID()] = obj{sep}'
        '{ind}elif isinstance(obj, AirVehicleState.AirVehicleState): {sep}'
        '{ind}{ind}stateMap[obj.get_ID()] = obj{sep}'
        '{ind}elif isinstance(obj, SessionStatus):{sep}'
        '{ind}{ind}ss = obj{sep}'

        '{sep}def connect():{sep}'
        '{ind}sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM){sep}'
        '{ind}server_address = ("localhost", 5555){sep}'
        '{ind}print("connecting to %s port %s") % (server_address){sep}'
        '{ind}sock.connect(server_address){sep}'
        '{ind}print("connected"){sep}'
        '{ind}return sock{sep}'

        '{sep}sock = connect(){sep}'
        'msg = LMCPFactory.LMCPFactory(){sep}'

        '{sep}uav_n = {uav_n}{sep}'
        'loc_n = {loc_n}{sep}'
        'UAVs = []{sep}'

        '{sep}for i in range(0,uav_n):{sep}'
        '{ind}UAVs.append(UAV(i+1,sock,stateMap)){sep}'
        
        '{sep}locations = []{sep}'
        'lon = {lons}{sep}'
        'lat = {lats}{sep}'
        'height = {heights}{sep}'
        'width = {widths}{sep}'

        '{sep}for i in range(0,loc_n):{sep}'
        '{ind}locations.append(Location(lat[i],lon[i],height[i],width[i],string.ascii_uppercase[i])){sep}'

        '{sep}try:{sep}'
        '{ind}while True:{sep}'

        '{sep}{ind}{ind}message = msg.getObject(sock.recv(2224)){sep}'

        '{sep}{ind}{ind}message_received(message){sep}'
        '{ind}{ind}for i in range(0,uav_n):{sep}'        
        '{ind}{ind}{ind}UAVs[i].stateMap = stateMap{sep}'        

        ).format(sep='\n', ind='    ', loc_n = n_loc, uav_n = n_uav, heights = height\
        , lons = loc_lon, lats = loc_lat, widths = width)     


    for string in output:

        token = tokenize(string)

        if token["Type"] == 'M':

            script += (
            '{sep}{ind}{ind}UAVs[{idl}] = {monitor}_monitor(UAVs[{idl}],{aux1},{aux2}){sep}'
            ).format(sep='\n', ind='    ', id=token["UAV"], idl=str(int(token["UAV"])-1), monitor = token["Definition"]\
            , aux1 = token['Aux_UAV'], aux2 = token['Aux_Loc'])      

    script += ('{sep}{ind}{ind}ctrl_input_args = get_args(M1.move){sep}'
        '{sep}{ind}{ind}ctrl_input = prepare_ctrl_input(UAVs,ctrl_input_args,current_play){sep}'
        '{sep}{ind}{ind}output = M1.move(**ctrl_input){sep}'
        ).format(sep='\n',ind='    ')

    for string in output:

        token = tokenize(string)

        if token["Type"] == 'B':

            script += ('{sep}{ind}{ind}if output["B_{id}_{behaviour}_{aux1}_{aux2}"] == 1:'
                    '{sep}{ind}{ind}{ind}{behaviour}(UAVs[{idl}],{aux1},{aux2}){sep}')\
                    .format(sep='\n', ind='    ', id=token["UAV"], idl=str(int(token["UAV"])-1), behaviour = token["Definition"]\
                    , aux1 = token['Aux_UAV'], aux2 = token['Aux_Loc']) 

    script += (
        '{sep}finally:{sep}'
        '{ind}print("closing socket"){sep}'
        '{ind}sock.close(){sep}'
        ).format(sep='\n',ind='    ')

    text_file = open("auto_generated/auto_code.py", "w+")
    text_file.write("%s" % script)
    text_file.close()

if __name__ == "__main__":

    output = get_spec_file()
    csynth = CodeSynthCtrl()
    xml_keys = get_args(make_xml)
    script_keys = get_args(make_script)
    move_keys = get_args(csynth.move)
    ctrl_input = {key: 1 if key in output['commands'] else 0 for key in move_keys}
    output['ctrl_input'] = ctrl_input
    xml_input = { key: output[key] for key in xml_keys }
    script_input = { key: output[key] for key in script_keys }
    make_xml(**xml_input)
    make_script(**script_input)
    
    # os.chdir("../tulip")
    
    # subprocess.check_output("python controller.py", shell=True)

    # try:
    #     shutil.move("demo_controller.py", "../auto_generated")
    # except:
    #     pass