import networkx as nx
import matplotlib.pyplot as plt
import numpy as np
import time
from collections import defaultdict
import heapq
import simpy
# from simpy.Simulation import *
# import pandas as pd

class RailNetwork:
    '''
    Class RailNetwork is the base map where the trains are operating on.
    '''
    def __init__(self, G = nx.Graph()):
        self.G = G
    
    @property    
    def single_track_init(self, block_length = [0.5]*100):
        corridor_path = range(length(block_length))
        self.G.add_path(corridor_path)
        for i in range(length(block_length)+1):
                    self.G[i-1][i]['dist'] = block_length[i-1]
                    self.G[i-1][i]['attr'] = None    
        
    def siding_init(self, siding = [10, 20, 30, 40, 50, 60, 70, 80, 90]):
            for i in siding:
                if isinstance(i, int):
                    self.G[i-1][i]['attr'] = 'siding'
                else:
                    self.G[i[0]][i[1]]['attr'] = 'siding'

class Simulator:
    def __init__(self, strt_t, stop_t, speed, time_log):
        ## define the feature parameters of a train object
        self.refresh = 2
        self.all_schedule = {}
        self.strt_t_ticks = time.mktime(time.strptime(strt_t, "%Y-%m-%d %H:%M:%S"))
        self.stop_t_ticks = time.mktime(time.strptime(stop_t, "%Y-%m-%d %H:%M:%S"))
    
    def scheduling(self):
        pass
    def attack_DoS(self, DoS_strt_t, DoS_stop_t, DoS_block):
        pass
class Train_generator:
    def __init__(self):
        pass


def networkX_write():
    '''
    ## generate a network graph in simple grids and save it into 'gpickle' file.
    '''
    number = 120
    G = nx.MultiGraph()
    
    pos = {}
    ## define the position (coordinates on the graph) of all nodes on the graph 
    ## it should be a dictionary
    for i in range(1, number+1):
        col = ((i-1) % 20) + 1 if ((i-1) // 20) % 2 == 0 else 20 - (i-1) % 20
        row = i // 20 if i % 20 != 0 else i // 20 - 1
        pos[i] = [col, row]

    nodes = []
    edges = []
    for i in range(1, number+1):
        nodes.append(i)
        if i < number:
            edges.append((i, i+1))
    
    siding = [15, 35, 55, 75]    
    for c in siding:
        nodes.append(c)
        G.add_path((c - 1, c + 1))
    
    ## define siding locations in the grids
    ## add corresponding links to the graph generated 
     
    G.add_nodes_from(nodes)
    G.add_edges_from(edges)
    nx.set_node_attributes(G, pos, 'pos')
    nx.write_gpickle(G, "a.gpickle")
    

def networkX_read():
    '''
    read "gpickle" file (basemap + data features 'pos')
    dynamically display the node: change the color of a node from red to green every second.
    data are stored in gpickle file together with the basemap
    '''

    G = nx.read_gpickle("a.gpickle")
    pos = nx.get_node_attributes(G, 'pos')

    ncolor = []
    for i in range(len(pos)):
        ncolor.append('r')

    #plt.ion()
    for index in range(len(ncolor)):
        plt.cla()
        ncolor[index] = 'g'
        if index > 0:
            ncolor[index-1] = 'r'
        nx.draw_networkx_nodes(G, pos, node_color=ncolor)
        nx.draw_networkx_labels(G, pos, font_size=16)
        nx.draw_networkx_edges(G, pos)

        # plt.pause(0.01)
    #plt.ioff()
    plt.show()
    #plt.pause(0.2)
    plt.cla()
    plt.close('all')
    return

'''
# code in main class
import networkX_w_r
networkX_w_r.networkX_write()
networkX_w_r.networkX_read()
'''

## Update starts here 20190313       
            

class single_train:
    '''
    many trains are generated by one control point,
    here I want to output the schedule of each train.
    '''

    def __init__(self, strt_t, stop_t, is_DoS, DoS_strt_t, DoS_stop_t, DoS_block, siding, block):
        self.all_schedule = defaultdict(lambda: {})     
        # initialize the global schedule for train Key: self.number
        
        ## load the rail network
        self.G = nx.read_gpickle("a.gpickle")   
        self.pos = nx.get_node_attributes(self.G, 'pos')
        self.labels = {}
        self.pos_labels = {}
        
        self.siding = siding    # position of sidings. A list of integer (block number)
        self.block = block      # the length of each block. A list of floats
        self.refresh = 1        # unit of refreshing time in minutes
        
        ## strt_t and stop_t are string for time, the format is '2018-01-01 00:00:00'
        self.T = time
        self.strt_t_ticks = time.mktime(time.strptime(strt_t, "%Y-%m-%d %H:%M:%S"))
        self.stop_t_ticks = time.mktime(time.strptime(stop_t, "%Y-%m-%d %H:%M:%S"))
        self.is_DoS = is_DoS    # dummy information of DoS (if DoS is in-place or not)
        self.DoS_strt_t_ticks = time.mktime(time.strptime(DoS_strt_t, "%Y-%m-%d %H:%M:%S"))
        self.DoS_stop_t_ticks = time.mktime(time.strptime(DoS_stop_t, "%Y-%m-%d %H:%M:%S"))
        self.DoS_block = DoS_block
        
        self.number = 0         ## self.number is the number of trains.
        
        self.one_schedule = {}  # we get a new self.one_schedule after each refresh
        self.one_detail = {}
        self.speed = {}
        
        self.hdw_exp_min = 10   # parameter of headway, in minutes
        self.hdw_dev_min = 2    # standard deviation of headway, in minutes
        
        self.mph_exp = 50     # parameter of speed, in miles per hour
        self.mph_dev = 10     # standard deviation of speed, in miles per hour
        
        '''state variables below: dictionaries describing the states of every train
        ## distance means x-axis coordinates, default value is 0
        ## distance is the dictionary for trains with the Key Value as train indices (integers)
        ## the current block for any train at any moment 1 is the default value for any key in this dictionary
        ## the cumulative distance used to determine the current block, default value is the length of the first block
        '''
        self.distance = defaultdict(lambda: 0.0)    # assuming every train initialize from the coordinate origin
        self.rank = defaultdict(lambda: 0)          # rank is used to determine the order of trains                  
        self.curr_block = defaultdict(lambda: 1)                
        self.sum_block_dis = defaultdict(lambda: self.block[0]) 
        self.time = {1: self.strt_t_ticks}       
        '''The dictionary showing the initialization time for each train entering the system.
        time : dictionary
            Key: train number 'tn'
            Value: strt_t + time of train number 'tn' has been traveling at this moment, unit in seconds
        '''
        # define which siding that late/fast train surpass previous/slow trains
        self.isPass = defaultdict(lambda: float('inf'))
        # in order to solve problem: the number of trains is not the rank of trains. I use dict rank[n]
        self.rank = defaultdict(lambda: 0)
        self.weight = defaultdict(lambda: 0)

    def train(self, env):
        
        def get_sum_and_curr_block(tn):
            """Encapsulated function to determine concurrent block and total blocked distance of a train:
            
            (at each moment)
            the accumulative distance traveled by a train. 
            the current block number of a train.
            
            Function or returns: 
                Update the two dictionaries (sum_block_dis) and (curr_block)
            
            sum_block_dis : dictionary
                Key: train number 'tn'
                Value: cumulative distance traveled by this train as occupied blocks (total blocked distance)
            curr_block : dictionary
                Key: train number 'tn'
                Value: current block number for this train  
                
            Notes:
            ------
                Calculate distance[tn] first, and then update the two dictionaries.
                distance[tn] is the current distance from the origin of coordinates. 
            
            To-do:
            ------
                The relationships and logics need to be normalized to eliminate the more than/less than signs. 
                Merge two directions into one single judgment logic.   
            """
            # if (distance > block_begin), block go further; if (distance < block_end), block go close.
            if tn == 0:     # avoid initializing train number 0 (int) in the default dictionaries
                pass
            else:
                while not (self.sum_block_dis[tn] - self.block[self.curr_block[tn]]) < self.distance[tn] <= (self.sum_block_dis[tn]):
                    if self.distance[tn] >= self.sum_block_dis[tn]:
                        # When updated position is at the next block (Forward direction): 
                        self.sum_block_dis[tn] += self.block[self.curr_block[tn]]
                        self.curr_block[tn] += 1
    
                    elif self.distance[tn] <= (self.sum_block_dis[tn] - self.block[self.curr_block[tn]]):
                        # When updated position is at the next block (Reverse direction): 
                        self.sum_block_dis[tn] -= self.block[self.curr_block[tn]]
                        self.curr_block[tn] -= 1
             
        '''Dictionary containing the concurrent rank for each train, with train number as keys. 
        rank : dictionary
            Key: train number 'tn'
            Value: current order of position (directional, no.1 on the 'rightmost')
        '''
        def update_rank():
            """Update the ranking for all concurrent trains in the system
            
            """
            sorted_distance = sorted(self.distance.values(),reverse=True)
            for i in self.distance.keys():
                self.rank[i] = 1 + sorted_distance.index(self.distance[i])
                           
        update_rank()           # initialize the rank dictionary
        n = 1       # n is the counter variable used to create many "one_detail", otherwise all "one_schedule" will be the same
        temp = 0    # temp is a stop watch with refreshing time, counting the reminder time before another new train is generated in minutes. 
        np.random.seed()
        #self.speed[1] = np.random.normal(self.mph_exp/60.0, self.mph_dev/60.0)      # speed in miles per minute, float division
        #self.weight[1] = np.random.randint(1, 4)        
        whileloopcount = 0
        
        while True:
            whileloopcount += 1
            
                
            '''Explain the while True loop
            '''
            '''First, draw the dynamic circle dots in the topology view 
            Notes:
            ------
                Initializing labels and colors
            ''' 
            plt.clf()
            self.labels.clear()
            self.pos_labels.clear()
            self.ncolor = []
            # default empty block to 'green'
            for i in range(len(self.pos)):
                self.ncolor.append('g')
            # default block which have siding to 'black'
            for i in self.siding:
                self.ncolor[i] = 'black'
                      
            
            # headway in minutes, local variable, randomize every loop in the while True body
            headway = np.random.normal(self.hdw_exp_min, self.hdw_dev_min)  
            # because headway > temp is possible, determine if there is a new train after a new refresh by the judgment below:
            if temp < headway:  ## no need for a new train
                temp += self.refresh
            else:               ## need a new train and clear temp
                temp = headway % self.refresh   # clearing the temp stop watch
                self.number += 1                # adding a new train, meanwhile assigning the train number as its identifier.
                self.speed[self.number] = np.random.normal(self.mph_exp/60.0, self.mph_dev/60.0)    # speed in miles per minute                
                self.time[self.number] = self.strt_t_ticks + temp * 60  # in seconds because of the ticks
                self.weight[self.number] = np.random.randint(1, 4)

                # update distance[tn] and block status for the newly generated train
                self.distance[self.number] += self.speed[self.number] * temp    # miles/min * mins
            get_sum_and_curr_block(self.number)
            
            print 'this is loop: ' +str(whileloopcount)    
            '''Starting below includes both DoS and overtaking policies. Needs to be separated.
            '''
            print self.distance.values()
            update_rank()
            for x in xrange(1, len(self.rank) + 1):     # for all current trains, starting from the first of the queue
                i = self.rank[x]                        # train number x is now ranked as i
                self.one_detail = {}                    # initialize the one_detail dictionary
                '''Dictionary containing the lifetime information and behaviors of a train, with train number as keys.
                one_detail : dictionary
                    Key: train number 'tn'
                    Value: dictionary for key-value pairs with lifetime attributes of a train 
                '''
                self.time[i] += self.refresh * 60   # update time line in seconds accrues by refresh, refresh in minutes, time in seconds
                
                '''When DoS happens:
                '''
                if self.is_DoS is True:
                    # self.time[tn] is the time for a train has been traveling + strt_t time in seconds, concurrent global time for train 'tn'
                    # so self.time[1] is the current global time. Notice that time is a dictionary, with keys as integer train identifiers.
                    if self.DoS_strt_t_ticks < self.time[1] < self.DoS_stop_t_ticks:
                        if self.curr_block[i] != self.DoS_block:
                            self.distance[i] += self.speed[i] * self.refresh
                            get_sum_and_curr_block(i)
                    else:
                        self.distance[i] += self.speed[i] * self.refresh
                        get_sum_and_curr_block(i)

                elif self.is_DoS is False:
                    self.distance[i] += self.speed[i] * self.refresh
                    get_sum_and_curr_block(i)

                '''When overtaking happens:
                Traverse the rank of all train, if low rank catch up high rank, it should follow instead of surpass. 
                Unless there is a siding.
                '''
                if x > 1:
                    # The block position of prev train and current train
                    '''
                    Overtake Policy:
                    
                    # when block small enough and speed large enough, there would be a bug
                    '''
                    if self.curr_block[self.rank[x - 1]] <= self.curr_block[self.rank[x]] + 1:
                        for j in self.siding:
                            if self.curr_block[self.rank[x - 1]] == j:
                                if self.speed[self.rank[x-1]] < self.speed[self.rank[x]]:
                                    self.rank[x], self.rank[x - 1] = self.rank[x - 1], self.rank[x]
                                    self.distance[self.rank[x]] -= self.speed[self.rank[x]] * self.refresh
                                    get_sum_and_curr_block(i)
                                break

                            elif j == self.siding[-1]:
                                self.distance[self.rank[x]] = self.sum_block_dis[self.rank[x]] - self.block[self.curr_block[self.rank[x]]]
                                #self.distance[self.rank[x]] = max(0, self.distance[self.rank[x]])
                                get_sum_and_curr_block(i)


                k = self.curr_block[i]

                # set the color of train node
                if 0 < k < len(self.pos):
                    self.ncolor[k-1] = 'r'

                if 0 < k < len(self.pos):
                    self.labels[k] = i
                    self.pos_labels[k] = self.pos[k]

                self.one_detail['time'] = round(self.speed[i], 2)
                self.one_detail['speed(miles/min)'] = round(self.speed[i], 2)
                self.one_detail['distance(miles)'] = round(self.distance[i], 2)
                self.one_detail['headway(mins)'] = round(headway, 2)
                self.one_detail['weight(1-3)'] = self.weight[i]
                time_standard = self.T.strftime("%Y-%m-%d %H:%M:%S", self.T.localtime(self.time[1]))
                self.one_schedule[time_standard] = self.one_detail
                self.all_schedule[i][time_standard] = self.one_schedule[time_standard]
                n += 1

            # draw the train map
            # nx.draw_networkx_nodes(self.G, self.pos, node_color=self.ncolor, node_size=200)
            # nx.draw_networkx_labels(self.G, self.pos_labels, self.labels, font_size=10)
            # nx.draw_networkx_edges(self.G, self.pos)
            
            # networkX pause 0.01 seconds
            # plt.pause(0.05)
            yield env.timeout(self.refresh*60)
        
    
    def run(self):
        # use simpy library to define the process of train system
        env = simpy.Environment()
        env.process(self.train(env))
        duration = self.stop_t_ticks - self.strt_t_ticks
        env.run(until=duration)

    def string_diagram(self):
        # draw the train working diagram
        '''begin comment__train stringline diagram'''
        x = []; y = []
        for i in self.all_schedule:
            x.append([])
            y.append([])

            for j in self.all_schedule[i]:
                x[i-1].append((time.mktime(time.strptime(j, "%Y-%m-%d %H:%M:%S")) - self.strt_t_ticks) / 3600)
                y[i-1].append(self.all_schedule[i][j]['distance(miles)'])

            x[i-1].sort()
            y[i-1].sort()

        plt.title('Result Analysis')
        for n in range(len(x)-1):
            if n % 4 == 0:
                plt.plot(x[n], y[n], color='green')
            if n % 4 == 1:
                plt.plot(x[n], y[n], color='blue')
            if n % 4 == 2:
                plt.plot(x[n], y[n], color='red')
            if n % 4 == 3:
                plt.plot(x[n], y[n], color='black')

        plt.legend()
        plt.xlabel('time /hours')
        plt.ylabel('distance /miles')
        plt.show()
        '''end comment__train stringline diagram'''

        # print self.all_schedule
        return self.all_schedule

'''
# single_train
from single_train import single_train

import networkX_w_r
networkX_w_r.networkX_write()

a = single_train('2018-01-01 00:00:00', '2018-01-03 00:00:00', [200, 400, 600, 800])
print a.string_diagram()
'''

class multi_dirc:
    '''
    many trains are generated by two control points,
    here I want to output the schedule of each train.
    '''

    def __init__(self, strt_t, stop_t, dis_miles, buffer_list):
        # define parameters
        self.buffer = 3
        self.all_schedule_A = {}
        self.dis = dis_miles
        self.buffer_list = buffer_list
        self.T = time
        self.strt_t_ticks = time.mktime(time.strptime(strt_t, "%Y-%m-%d %H:%M:%S"))
        self.stop_t_ticks = time.mktime(time.strptime(stop_t, "%Y-%m-%d %H:%M:%S"))
        self.number = 1
        self.one_schedule_A = {}
        self.one_schedule_B = {}
        self.one_detail_A = {}
        self.one_detail_B = {}
        self.speed_A = {}
        self.speed_B = np.random.normal(3, 0.5)
        self.distance_A = {1: 0}
        self.time = {1: self.strt_t_ticks}
        env = simpy.Environment()
        env.process(self.train(env))
        duration = self.stop_t_ticks - self.strt_t_ticks
        env.run(until=duration)

    def train(self, env):
        # n is used for create many "one_detail_A", otherwise all "one_schedule_A" will be the same
        n = 0
        index_A = 0
        index_B = len(self.buffer_list)

        while True:
            np.random.seed()
            self.speed_A[self.number] = np.random.normal(3, 0.5) # miles per minute
            headway = np.random.normal(20, 5)
            self.all_schedule_A[self.number] = {}
            self.time[self.number] = self.strt_t_ticks
            self.distance_A[self.number] = 0

            for i in xrange(1, self.number+1):
                self.one_detail_A[n] = {}
                self.time[i] += headway * 60
                self.distance_A[i] += self.speed_A[i] * headway
                distance_B = (self.speed_B * (self.time[1] - self.strt_t_ticks)) / 60
                if i > 1:
                    if self.distance_A[i] > self.distance_A[i-1] - self.speed_A[i-1] * self.buffer:
                        self.distance_A[i] = self.distance_A[i-1] - self.speed_A[i-1] * self.buffer
                dirc = 'A'
                self.one_detail_A[n]['dirc'] = dirc
                self.one_detail_A[n]['speed_A(mils/min)'] = round(self.speed_A[i], 2)
                self.one_detail_A[n]['distance_A(miles)'] = round(self.distance_A[i], 2)
                self.one_detail_A[n]['headway(mins)'] = round(headway, 2)

                # get A and B pass through which buffer
                for x in range(len(self.buffer_list)):
                    if self.buffer_list[x] < self.distance_A[i]:
                        index_A = x + 1
                for x in range(len(self.buffer_list), 0):
                    if self.dis - self.buffer_list[x] < distance_B:
                        index_B = x + 1
                # A and B
                if index_B - index_A == 2:
                    time_arrive_buffer_A = self.buffer_list[index_A-1] / self.speed_A[i]
                    time_arrive_buffer_B = (self.dis - self.buffer_list[index_B-1]) / self.speed_B
                    if time_arrive_buffer_A < time_arrive_buffer_B:
                        self.distance_A[i] -= (time_arrive_buffer_B - time_arrive_buffer_A) * self.speed_A[i]
                    else:
                        distance_B -= (time_arrive_buffer_A - time_arrive_buffer_B) * self.speed_B

                self.one_detail_A[n]['buffer_index'] = index_A
                time_standard = self.T.strftime("%Y-%m-%d %H:%M:%S", self.T.localtime(self.time[i]))
                self.one_schedule_A[time_standard] = self.one_detail_A[n]
                self.all_schedule_A[i][time_standard] = self.one_schedule_A[time_standard]
                n += 1
            self.number += 1
            yield env.timeout(headway * 60)

    def string_diagram(self):
        return self.all_schedule_A

'''
# code in main class

from multi_dirc import multi_dirc
a = multi_dirc('2018-01-01 00:00:00', '2018-01-02 00:00:00', 1000, [500, 1000, 1500, 2000, 2500])
print a.string_diagram()
'''

if __name__ == '__main__':
    a = single_train('2018-01-01 00:00:00', '2018-01-01 12:00:00', False, '2018-01-01 09:00:00', '2018-01-01 10:30:00', 23, [20, 30, 40, 60, 80, 100], [20] * 5000)
    a.run()
    a.string_diagram()
