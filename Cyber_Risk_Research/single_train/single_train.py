import simpy
import numpy as np
import time
import networkx as nx
import matplotlib.pyplot as plt
import collections


class single_train:
    '''
    many trains are generated by one control point,
    here I want to output the schedule of each train.
    '''

    def __init__(self, begin, end):
        self.buffer = 3
        self.all_schedule = {}
        self.begin = begin
        self.end = end
        self.T = time
        self.begin_ticks = time.mktime(time.strptime(self.begin, "%Y-%m-%d %H:%M:%S"))
        self.end_ticks = time.mktime(time.strptime(self.end, "%Y-%m-%d %H:%M:%S"))
        self.number = 1
        self.one_schedule = {}
        self.one_detail = {}
        self.speed = {}
        self.distance = {}
        self.time = {1: self.begin_ticks}
        self.G = nx.read_gpickle("a.gpickle")
        self.pos = nx.get_node_attributes(self.G, 'pos')
        self.ncolor = []
        # self.labels = collections.defaultdict()
        self.labels = {}
        self.pos_labels = {}
        env = simpy.Environment()
        env.process(self.train(env))
        duration = self.end_ticks - self.begin_ticks
        env.run(until=duration)

    def train(self, env):
        # n is used for create many "one_detail", otherwise all "one_schedule" will be the same
        n = 0

        while True:
            plt.close('all')
            self.labels.clear()
            self.pos_labels.clear()
            self.ncolor = []
            for i in range(len(self.pos)):
                self.ncolor.append('r')
            np.random.seed()
            self.speed[self.number] = np.random.normal(3, 0.5)  # miles per second
            headway = np.random.normal(15, 3)
            self.all_schedule[self.number] = {}
            self.time[self.number] = self.begin_ticks

            for i in xrange(1, self.number + 1):
                self.one_detail[n] = {}
                self.time[i] += headway * 60
                self.distance[i] = (self.speed[i] * (self.time[i] - self.begin_ticks)) / 60

                if i > 1:
                    if self.distance[i] > self.distance[i - 1] - self.speed[i - 1] * self.buffer:
                        self.distance[i] = self.distance[i - 1] - self.speed[i - 1] * self.buffer

                # dynamic color of networkX
                k = 0
                plt.cla()
                for m in range(self.number):
                    if self.distance[i] > m * 25:
                        k = m

                # set the color of train node
                self.ncolor[k] = 'g'

                if len(self.pos) > k > 0:
                    self.labels[k+1] = i
                    self.pos_labels[k+1] = self.pos[k+1]

                self.one_detail[n]['speed(mils/min)'] = round(self.speed[i], 2)
                self.one_detail[n]['distance(miles)'] = round(self.distance[i], 2)
                self.one_detail[n]['headway(mins)'] = round(headway, 2)
                time_standard = self.T.strftime("%Y-%m-%d %H:%M:%S", self.T.localtime(self.time[i]))
                self.one_schedule[time_standard] = self.one_detail[n]
                self.all_schedule[i][time_standard] = self.one_schedule[time_standard]
                n += 1

            # draw the train map
            nx.draw_networkx_nodes(self.G, self.pos, node_color=self.ncolor)
            nx.draw_networkx_labels(self.G, self.pos_labels, self.labels, font_size=10)
            nx.draw_networkx_edges(self.G, self.pos)

            self.number += 1

            # networkX pause 0.1 seconds
            plt.pause(0.5)
            yield env.timeout(headway * 60)

    def generate_all(self):
        return self.all_schedule
