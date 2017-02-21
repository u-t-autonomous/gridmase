import socket
from geopy.distance import vincenty
from lmcp import LMCPFactory
from afrl.cmasi import EntityState
from afrl.cmasi import AirVehicleState
from afrl.cmasi import AirVehicleConfiguration
from afrl.cmasi import Play
from afrl.cmasi.SessionStatus import SessionStatus
from demo_controller import ExampleCtrl
from PyMASE import UAV, Location, get_args
import string

import random
import bisect
import threading
import sys

from inputs import get_key
import time

# This value may need to be increased when running at speeds greater than 20x
global_threshHold = .002

global_key = ("key", -1)

key_mutex = True


stateMap = dict()
playMap = dict()


UAVs = []
global_uav_count = 0
locations = []



def message_received(obj):
    global stateMap
    global configMap
    global playMap
    global commandMap
    global timer
    if isinstance(obj ,SessionStatus):
        timer = obj.get_ScenarioTime()
    if isinstance(obj ,AirVehicleConfiguration.AirVehicleConfiguration):
        configMap[obj.get_ID()] = obj
    if isinstance(obj, AirVehicleState.AirVehicleState): 
        stateMap[obj.get_ID()] = obj
    if isinstance(obj, Play.Play):
        playMap[obj.get_UAVID()] = obj
    if isinstance(obj, SessionStatus):
        ss = obj

def connect():
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_address = ("localhost", 5555)
    print("connecting to %s port %s") % (server_address)
    sock.connect(server_address)
    print("connected")
    return sock




sock = connect()



class Simulation:

    global locations

    grid = []
    agent_blocks = []
    moving_obstacles = []
    obstacle_percentages = []
    agent_percentages = []
    # slip_percentages[current_x][current_y][action](x_result, y_result, prob)
    slip_percentages = []
    log = {"agents": [], "moving_obstacles": []} 
    WINDOW_SIZE = [0, 0]
    HEIGHT = 0
    WIDTH = 0
    MARGIN = 2
    time_step = 0
    matrix_active = False
    slip_active = False

    
    def __init__(self, configFile):

        #import config file
        configFile = open(configFile, 'r')

        for line in configFile:
            line = line.split(" ")
            if line[0] == "HEIGHT:":
                self.HEIGHT = int(line[1])
            elif line[0] == "WIDTH:":
                self.WIDTH = int(line[1])
                for column in range(self.WIDTH):
                    self.grid.append([])
                    for row in range(self.HEIGHT):
                        self.grid[column].append(Empty_block())

            #elif line[0] == "MARGIN:":
            #    self.MARGIN = int(line[1])
            elif line[0] == "BLOCK:":
                if line[1] == "agent":
                    self.agent_blocks.append(Agent_block(int(line[2]),int(line[3])))
                elif line[1] == "fixed_obstacle":
                    self.grid[int(line[2])][int(line[3])] = Obstacle_block()
                elif line[1] == "moving_obstacle":
                    self.moving_obstacles.append(Moving_obstacle_block(int(line[2]), int(line[3])))
                elif line[1] == "goal":
                    self.grid[int(line[2])][int(line[3])] = Goal_block()
                else:
                    self.grid[int(line[2])][int(line[3])] = Empty_block()
    
        self.WINDOW_SIZE[0] = int(30*self.WIDTH+2)
        self.WINDOW_SIZE[1] = int(30*self.HEIGHT+2)

        for i in self.agent_blocks:
            self.agent_percentages.append({})
            self.log["agents"].append([])
        for i in self.moving_obstacles:
            self.obstacle_percentages.append({})
            self.log["moving_obstacles"].append([])


        #setup slip percentages
        for i in range(self.WIDTH):
            self.slip_percentages.append([])
            for k in range(self.HEIGHT):
                self.slip_percentages[i].append([])
                self.slip_percentages[i][k] = {'north': [], 'east': [], 'south': [], 'west': []}



        for i in range(self.WIDTH):
            locations.append([])
            for k in range(self.HEIGHT):
                locations[i].append(Location(float(k)/100, float(i)/100, .1 , .1))

        #self.load_matrix_file(matrixFile)


    def out_of_bounds(self, x_pos, y_pos):
        #return true if the position is outside of the playfield
        return not (x_pos >= 0 and x_pos < self.WIDTH and y_pos >= 0 and y_pos < self.HEIGHT)

    def move_agent(self, number, action):
        x_pos = self.agent_blocks[number].column
        y_pos = self.agent_blocks[number].row
        self.log["agents"][number].append((self.time_step, (x_pos, y_pos), action))
        if self.slip_active:
            #then slip
            #only execute of the action would keep the block within the grid
            if action == "north" and y_pos > 0 or action == "south" and y_pos < self.HEIGHT - 1 or action == "west" and x_pos > 0 or action == "east" and x_pos < self.WIDTH - 1:

                percents = []
                percents_temp = self.slip_percentages[x_pos][y_pos][action] #list of tuples in form of ((x_result, y_result), probability of landing there)

                for percent in percents_temp:
                    if not self.out_of_bounds(percent[0][0], percent[0][1]):
                        percents.append(percent)

                result = self.weighted_choice(percents)
                self.agent_blocks[number].move_to(result[0], result[1])

        else:
            if action == "north" and y_pos < self.HEIGHT - 1:
                self.agent_blocks[number].move_north()
            elif action == "south" and y_pos > 0:
                self.agent_blocks[number].move_south()
            elif action == "west" and x_pos > 0:
                self.agent_blocks[number].move_west()
            elif action == "east" and x_pos < self.WIDTH - 1:
                self.agent_blocks[number].move_east() 


    def initialize(self):
        for obstacle in self.moving_obstacles:
            obstacle.move_to(obstacle.column, obstacle.row)

        for agent in self.agent_blocks:
            agent.move_to(agent.column, agent.row)

        self.wait_for_obstacles()
        self.wait_for_agents()
        print("Done with initialization")

    def get_state(self):

        agent_list = []
        fixed_obstacle_list = []
        moving_obstacle_list = []

        for agent in self.agent_blocks:
            agent_list.append((agent.column, agent.row))

        for column in range(self.WIDTH):
            for row in range(self.HEIGHT):
                if type(self.grid[column][row]) is Obstacle_block:
                    fixed_obstacle_list.append((column,row))
        
        for obstacle in self.moving_obstacles:
            moving_obstacle_list.append((obstacle.column, obstacle.row))
        
        dict_blocks = {"agents": agent_list, "fixed_obstacles": fixed_obstacle_list, "moving_obstacles": moving_obstacle_list}

        return dict_blocks

    #action = weighted_choice([(1,0), (0,100)])

    def weighted_choice(self, choices):
        values, weights = zip(*choices)
        total = 0
        cum_weights = []
        for w in weights:
            total += w
            cum_weights.append(total)
        x = random.random() * total
        i = bisect.bisect(cum_weights, x)
        return values[i]
        
        
    def move_obstacle(self, number, action):
        x_pos = self.moving_obstacles[number].column
        y_pos = self.moving_obstacles[number].row
        self.log["moving_obstacles"][number].append((self.time_step, (x_pos, y_pos), action))
        if action == "north" and y_pos < self.HEIGHT - 1:
            self.moving_obstacles[number].move_north()
        elif action == "south" and y_pos > 0:
            self.moving_obstacles[number].move_south()
        elif action == "west" and x_pos > 0:
            self.moving_obstacles[number].move_west()
        elif action == "east" and x_pos < self.WIDTH - 1:
            self.moving_obstacles[number].move_east()

    def load_matrix_file(self, matrixFile):
        matrixFile = file(matrixFile, 'r')
        for line in matrixFile:
            if len(line) == 0:
                continue

            line = line.split(" ")

            if len(line[0]) == 1:
                if line[0] == 'a':
                    for agent_percentage in self.agent_percentages:
                        agent_percentage[(int(line[1]), int(line[2]))] = (int(line[3]), int(line[4]), int(line[5]), int(line[6]), int(line[6]))
                elif line[0] == 'o':
                    for obstacle_percentage in self.obstacle_percentages:
                        obstacle_percentage[(int(line[1]), int(line[2]))] = (int(line[3]), int(line[4]), int(line[5]), int(line[6]), int(line[6]))
                continue

            index = int(line[0][1])
            if line[0][0] == 'a' and index < len(self.agent_percentages):
                self.agent_percentages[index][(int(line[1]), int(line[2]))] = (int(line[3]), int(line[4]), int(line[5]), int(line[6]), int(line[7]))
            elif line[0][0] == 'o' and index < len(self.obstacle_percentages):
                self.obstacle_percentages[index][(int(line[1]), int(line[2]))] = (int(line[3]), int(line[4]), int(line[5]), int(line[6]), int(line[7]))

        self.matrix_active = True
        matrixFile.close()

    def load_slip_file(self, slipFile):
        slipFile = file(slipFile, "r")
        for line in slipFile:
            line = line.split(" ")
            # current_x current_y action (x_result, y_result, prob)
            #self.slip_percentages[1][2]['north'].append(1, 1, .5)
            self.slip_percentages[int(line[0])][int(line[1])][line[2]].append(((int(line[3]), int(line[4])), float(line[5])))

        self.slip_active = True
        slipFile.close()

    def move_obstacles(self):
        index = 0
        for obstacle in self.moving_obstacles:
            # Grab percentages from matrix file
            obstacle_percentages = self.obstacle_percentages[index][(obstacle.column, obstacle.row)]
            action = self.weighted_choice((("north", obstacle_percentages[0]), ("east", obstacle_percentages[1]), ("south", obstacle_percentages[2]), ("west", obstacle_percentages[3]), ("stay", obstacle_percentages[4])))
            # Line can be used for debuging: action = self.weighted_choice((("north", .25), ("east", .25), ("south", .25), ("west", .25), ("stay", 0)))
            self.move_obstacle(index, action)
            index += 1

    def move_agents_matrix(self):
        index = 0
        for agent in self.agent_blocks:
            agent_percentages = self.agent_percentages[index][(agent.column, agent.row)]
            action = self.weighted_choice((("north", agent_percentages[0]), ("east", agent_percentages[1]), ("south", agent_percentages[2]), ("west", agent_percentages[3]), ("stay", agent_percentages[4])))
            self.move_agent(index, action)
            index += 1


    def move_agent_matrix(self, index):
        agent = self.agent_blocks[index]
        agent_percentages = self.agent_percentages[index][(agent.column, agent.row)]
        action = self.weighted_choice((("north", agent_percentages[0]), ("east", agent_percentages[1]), ("south", agent_percentages[2]), ("west", agent_percentages[3]), ("stay", agent_percentages[4])))
        self.move_agent(index, action)

    def handle_events(self):

        for agent in self.agent_blocks:
            for obstacle in self.moving_obstacles:
                if agent.column == obstacle.column and agent.row == obstacle.row:
                    print("Moving obstacle hit")
                    #return True
            if isinstance(self.grid[agent.column][agent.row], Obstacle_block):
                print("Fixed obstacle hit")
                #return True
            if isinstance(self.grid[agent.column][agent.row], Goal_block):
                print("Goal block hit")
                #return True

        return False
                         
    def get_log(self):
        return self.log

    def get_history(self, steps_back):
        if steps_back > self.time_step:
            steps_back = self.time_step

        out_log = {"agents": [], "moving_obstacles": []} 
        #self.log["agents"][number].append((self.time_step, (x_pos, y_pos), action))
        for number in range(len(self.log["agents"])):
            out_log["agents"].append(self.log["agents"][number][-steps_back:])

        for number in range(len(self.log["moving_obstacles"])):
            out_log["move_obstacles"][number].append(elf.log["moving_obstacles"][number][-steps_back:])

        return out_log            

    def get_key(self):
        #TODO find another source of keyboard input
        key_presses = ["KEY_UP", "KEY_RIGHT", "KEY_DOWN", "KEY_LEFT"]


        window_time = time.time()
        done = False
        while not done:
            # not sure if mutex is necessary
            if key_mutex:
                local_key = global_key

            # only accept keys within time window
            if window_time <= local_key[1]:
                key = local_key[0]
                done = True


        print("Moving: " + key)


        if key == "KEY_UP":
            return "north"
        if key == "KEY_RIGHT":
            return "east"
        if key == "KEY_DOWN":
            return "south"
        if key == "KEY_LEFT":
            return "west"


    def update(self):
        self.move_obstacles()
        self.time_step += 1
        #self.clock.tick(2)
        return self.handle_events()

    def step_forward(self):
        self.time_step += 1
        #self.clock.tick(2)
        return self.handle_events()

    def generate_agent_matrix(self, matrixFile):
        log = self.get_log()["agents"]
        f = file(matrixFile, 'w')

        counter = 0
        for agent in log:
            event_count = []
            for column in range(self.WIDTH):
                event_count.append([])
                for row in range(self.HEIGHT):
                    event_count[column].append([0, 0, 0, 0, 0]) #north, east, south, west, stay
            
            for event in agent:
                if event[2] == "north":
                    index = 0
                elif event[2] == "east":
                    index = 1
                elif event[2] == "south":
                    index = 2
                elif event[2] == "west":
                    index = 3
                else:
                    index = 4
                
                event_count[event[1][0]][event[1][1]][index] += 1

            for x in range(len(event_count)):
                for y in range(len(event_count[x])):
                    prob_temp = event_count[x][y]
                    
                    total_count = sum(prob_temp)
                    if total_count == 0:
                        f.write ("a{} {} {} 25 25 25 25 0\n".format(counter, x, y))
                    else:
                        f.write("a{} {} {} {} {} {} {} {}\n".format(counter, x, y, 100*prob_temp[0]/total_count, 100*prob_temp[1]/total_count, 100*prob_temp[2]/total_count, 100*prob_temp[3]/total_count, 100*prob_temp[4]/total_count))
            f.write("\n")
            counter += 1


    

    def move(self, movement):
        self.move_obstacles()
        self.wait_for_obstacles()


        if (self.handle_events()): 
            return True, 0

        if movement == "matrix":
            #move according to matrix
            self.move_agents_matrix()
        elif movement == "keyboard":
            #keyboard input
            for agent in range(len(self.agent_blocks)):
                print("check")
                self.move_agent(agent, self.get_key())

        else:
            #move according to list input
            for agent in range(len(movement)):
                if movement[agent] == "keyboard":
                    self.move_agent(agent, self.get_key())
                elif movement[agent] == "matrix":
                    self.move_agent_matrix(agent)
                else:
                    self.move_agent(agent, movement[agent])

        self.wait_for_agents()
        if (self.step_forward()):
            return True, 0

        return False, self.get_state()


    def wait_for_agents(self):
        threshHold = global_threshHold
        flags = [0] * len(self.agent_blocks)

        # Keep looping until all flags are triggered
        while sum(flags) < len(flags): 
            for i in range(len(self.agent_blocks)):

                # grab correct UAV
                uav = UAVs[i]
                agent = self.agent_blocks[i]
                location = locations[agent.column][agent.row]

                #TODO remove flagged obstacles from for loop so loops aren't wasted

                # Check if uav is within thresHold
                uav_position = (uav.stateMap.get(uav.id).get_Location().get_Longitude(), uav.stateMap.get(uav.id).get_Location().get_Latitude())
                if abs(uav_position[0] - location.center_lon) < threshHold and abs(uav_position[1] - location.center_lat) < threshHold:
                    flags[i] = 1


    def wait_for_obstacles(self):
        threshHold = global_threshHold
        flags = [0] * len(self.moving_obstacles)

        # Keep looping until all flags are triggered
        while sum(flags) < len(flags): 
            for i in range(len(self.moving_obstacles)):

                # grab correct UAV
                uav = UAVs[len(self.agent_blocks) + i]
                obstacle = self.moving_obstacles[i]
                location = locations[obstacle.column][obstacle.row]

                #TODO remove flagged obstacles from for loop so loops aren't wasted

                # Check if uav is within thresHold
                uav_position = (uav.stateMap.get(uav.id).get_Location().get_Longitude(), uav.stateMap.get(uav.id).get_Location().get_Latitude())
                if abs(uav_position[0] - location.center_lon) < threshHold and abs(uav_position[1] - location.center_lat) < threshHold:
                    flags[i] = 1


                    
        


class Block():
    color = (0,0,0)
    block_size = 30

    def draw(self, surface, column, row, MARGIN):
        print("You shouldn't be trying to draw")

class Obstacle_block(Block):
    color = (255,0,0) #red


class Moving_obstacle_block(Obstacle_block):

    def __init__(self, column, row):
        global global_uav_count
        global UAVs

        self.column = column
        self.row = row
        self.UAV = UAV(global_uav_count+1, sock, stateMap, turning_radius = .001)
        UAVs.append(self.UAV)
        global_uav_count += 1
        print("Created agent at ", column, row)

    def move_to(self, column, row):
        print("Moving agent to", column, row)
        self.column = column 
        self.row = row
        self.UAV.point_search(locations[self.column][self.row])
    def move_north(self):
        self.row += +1
        self.UAV.point_search(locations[self.column][self.row])
    def move_south(self):
        self.row += -1
        self.UAV.point_search(locations[self.column][self.row])
    def move_east(self):
        self.column += 1
        self.UAV.point_search(locations[self.column][self.row])
    def move_west(self):
        self.column += -1
        self.UAV.point_search(locations[self.column][self.row])



class Goal_block(Block):
    color = (255,255,0) #yellow


class Agent_block(Block):
    color = (0,128,0) #green


    def __init__(self, column, row):
        global global_uav_count
        global UAVs

        self.column = column
        self.row = row
        self.UAV = UAV(global_uav_count+1, sock, stateMap, turning_radius = .001)
        UAVs.append(self.UAV)
        global_uav_count += 1
        print("Created agent at ", column, row)

    def move_to(self, column, row):
        print("Moving agent to", column, row)
        self.column = column 
        self.row = row
        self.UAV.point_search(locations[self.column][self.row])
    def move_north(self):
        self.row += 1
        self.UAV.point_search(locations[self.column][self.row])
    def move_south(self):
        self.row += -1
        self.UAV.point_search(locations[self.column][self.row])
    def move_east(self):
        self.column += 1
        self.UAV.point_search(locations[self.column][self.row])
    def move_west(self):
        self.column += -1
        self.UAV.point_search(locations[self.column][self.row])

class Empty_block(Block):
    color = (255,255,255)

# continuesly updates state map

def update_statemap():
    while True:
        message = msg.getObject(sock.recv(2224))
        message_received(message)
        for i in range(0, global_uav_count):
            UAVs[i].stateMap = stateMap

        # TODO possibly add delay


def update_keys():
    global global_key
    key_presses = ["KEY_UP", "KEY_RIGHT", "KEY_DOWN", "KEY_LEFT"]

    while True:
        events = get_key()
        for event in events:
            if event.code in key_presses and event.state == 1:
                print("Got input: ", event.code)
                key_mutex = False
                global_key = (event.code, time.time())
                key_mutex = True
        #time.sleep(.001)



def amase_connect():
    flag = 0
    while flag != 1:
        flag = 1
        message = msg.getObject(sock.recv(2224))
        message_received(message)
        for i in range(0, global_uav_count):
            UAVs[i].stateMap = stateMap
            if UAVs[i].stateMap.get(UAVs[i].id) is None:
                flag = 0



# Create Simulation
sim = Simulation("config.txt")

sim.load_matrix_file("matrix.txt")

# Needs to happen after creating Simulation() object
msg = LMCPFactory.LMCPFactory()

amase_connect()

# Launch state map thread
statemap_thread = threading.Thread(target=update_statemap)
statemap_thread.daemon = True
statemap_thread.start()

# Launch keyboard input thread
# TODO change to joystick
key_thread = threading.Thread(target=update_keys)
key_thread.daemon = True
key_thread.start()


# Run initial simulation
sim.initialize()

done = False

while not done:

    # Step through gridsim
    try:
        done, state = sim.move(["keyboard"])


    # Ctrl-C will terminate loop
    except KeyboardInterrupt:
        done = True
        


# Will run on loop termination
print("closing socket")
sock.close()









