#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""
    PyRailSim
    Copyright (C) 2019  Zezhou Wang

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
"""
import copy
import logging
import random
from collections.abc import MutableSequence
from datetime import datetime, timedelta
from itertools import combinations, permutations

import networkx as nx
import numpy as np

from rail_networkx import all_simple_paths, shortest_path
from infrastructure import BigBlock, Track, Yard
from signaling import Aspect, AutoPoint, AutoSignal, CtrlPoint, HomeSignal
from train import Train, TrainList


class CorridorState():
    def __init__(self, sys):
        self.sys = sys

    def meetings(self, ):
        pass

    def passings(self, ):
        pass

class System():
    """
        Parameters
        ----------
        :headway: (**kw), seconds
            Traffic headway in seconds for unidirectional trains. 500 by default.
        :dos_pos: (MP1, MP2) (**kw)
            Tuple of MPs to be attacked by DoS. (None,None) by default (no DoS).
        :refresh_time: int (**kw), seconds
            Seconds between two consecutive traverse calculations of the simulation.
        :sp_containter: list (**kw), mph
            A list of randomized speed values for trains to initialize by. 
        :acc_container: list (**kw), miles/(sec)^2
            A list of randomized acceleration values for trains to initialize by. 
        :dcc_container: list (**kw), miles/(sec)^2
            A list of randomized deceleration values for trains to brake by."""

    def __init__(self, init_time, *args, **kwargs):
        super().__init__()

        self.sys_time = init_time.timestamp()
        # CPU format time in sec, transferable to numerical value or str values
        self.init_time = init_time.timestamp()
        self.term_time = float('inf') \
            if kwargs.get('term_time') is None \
            else kwargs.get('term_time').timestamp()
        self.G_origin = self.graph_constructor()
        self.G_skeleton = self.graph_extractor(self.G_origin)

        self.signal_points = list(self.G_origin.nodes())
        # list of all SignalPoints, including AutoPoints and CtrlPoints
        self.ctrl_points = list(self.G_skeleton.nodes())
        # list of all CtrlPoints. Indices are different from signal_points.
        self.vertex_points = [cp for cp in self.ctrl_points 
            if cp.vertex == True]
        # list of all vertex CtrlPoints where trains can initiate/terminate.
        self.tracks = [data['instance']
            for (u, v, data) in list(self.G_origin.edges(data=True))]
        # list of all Tracks.
        self.bigblocks = [data['instance']
            for (u, v, data) in list(self.G_skeleton.edges(data=True))]
        # list of all BigBlocks.
        self.dos_period = [datetime.strptime(t, "%Y-%m-%d %H:%M:%S").timestamp()
            for t in kwargs.get('dos_period') if type(t) == str]
        self.dos_pos = (None,None) \
            if kwargs.get('dos_pos') is None else kwargs.get('dos_pos')

        self._trains = TrainList()
        _min_spd, _max_spd, _min_acc, _max_acc = \
            0.01, 0.02, 2.78e-05 * 0.85, 2.78e-05 * 1.15
        self.headway = 500 if kwargs.get('headway') is None \
            else kwargs.get('headway')
        self.last_train_init_time = self.sys_time
        self.spd_container = args[0]\
            if args else [random.uniform(_min_spd, _max_spd) for i in range(20)]
        self.acc_container = args[1]\
            if args else [random.uniform(_min_acc, _max_acc) for i in range(20)]
        self.dcc_container = args[2]\
            if args else [random.uniform(self.sys_min_dcc*1.15, self.sys_min_dcc*1.25) for i in range(20)]
        self.dcc_container = [i if i >= self.sys_min_dcc else self.sys_min_dcc
            for i in self.dcc_container]
        self.refresh_time = 1 if kwargs.get('refresh_time') is None \
            else kwargs.get('refresh_time')
        self.dispatcher = None
        # self.register(self.blocks)
        # register method links the observation relationships

    @property
    def sys_min_dcc(self):
        '''
            Absolute value, minimum brake acceleration of all trains required by
            the system setup. If train's maximum braking deceleration is smaller
            than this value, it may violate some signal/speed limits at extreme 
            scenarios. When violated, the program will throw AssertionErrors at 
            braking distance/speed limit check.'''
        _signal_speeds = sorted(
            [spd for _, spd in Aspect.COLOR_SPD_DICT.items()])
        _speed_diff_pairs = [(_signal_speeds[i], _signal_speeds[i + 1])
                             for i in range(len(_signal_speeds) - 1)]
        _max_diff_square_of_spd = max(
            [abs(i[0]**2 - i[1]**2) for i in _speed_diff_pairs])
        _min_track_length = min([t.length for t in self.tracks])
        return _max_diff_square_of_spd / (2 * _min_track_length)

    @property
    def trains(self):
        '''
            List of all trains inside the system.'''
        return self._trains

    @property
    def train_num(self):
        return len(self.trains)

    @property
    def curr_routing_paths(self):
        '''
            A list of all currently cleared routing path lists inside the system.
            Each routing path list has the direction and segments information of
            the limit of movement authority cleared for a certain train.
            Each routing path list consists of routing tuples (2-element-tuple)'''

        def has_repeating_routing_paths(rplist, traversed):
            '''
                local function to determine if routing path list has repeating
                elements.'''
            for i in traversed:
                if i in rplist:
                    rplist.remove(i)
            rplist_copy = rplist
            for rp in rplist_copy:
                for _rp in rplist:
                    if connectable(rp, _rp) or connectable(_rp, rp):
                        return True
            return False

        def connectable(rp1, rp2):
            '''
                local function to determine if two lists of routing paths can be 
                connected together: linking two joint movement authorities.'''
            if rp1 and rp2:
                rp1_head, rp1_tail = rp1[0][0][0], rp1[-1][-1][0]
                rp2_head, rp2_tail = rp2[0][0][0], rp2[-1][-1][0]
                rp1_head_port, rp1_tail_port = rp1[0][0][1], rp1[-1][-1][1]
                rp2_head_port, rp2_tail_port = rp2[0][0][1], rp2[-1][-1][1]
                if rp1_tail == None or rp2_head == None:
                    return False
                elif rp1_tail == rp2_head:
                    assert rp1_tail.current_routes == rp2_head.current_routes
                    if (rp1_tail_port,
                            rp2_head_port) in rp1_tail.current_routes:
                        return True
            return False

        def add_cleared_routing_external_virtual_bblk():
            '''
                Add routing of initiating/terminalting routing path without a 
                materialized bigblock outside the vertex CtrlPoints.'''
            for cp in self.vertex_points:
                if cp.current_routes:
                    for r in cp.current_routes:
                        if cp.track_by_port.get(r[0]):
                            _routing_list.append([((cp, r[1]), (None, None))])
                        if cp.track_by_port.get(r[1]):
                            _routing_list.append([((None, None), (cp, r[0]))])

        _routing_list = [i for i in [getattr(_bblk, 'self_routing_path') 
            for _bblk in self.bigblocks]if i]
        add_cleared_routing_external_virtual_bblk()
        _traversed = []
        while has_repeating_routing_paths(_routing_list, _traversed):
            for i in range(len(_routing_list)):
                for j in range(len(_routing_list)):
                    if connectable(_routing_list[i], _routing_list[j]):
                        _routing_list[i].extend(_routing_list[j])
                        _traversed.append(_routing_list[j])
                    elif connectable(_routing_list[j], _routing_list[i]):
                        _routing_list[j].extend(_routing_list[i])
                        _traversed.append(_routing_list[i])
        return _routing_list

    @property
    def curr_routing_paths_cp(self):
        _routing_paths_cp = []
        for rp in self.curr_routing_paths:
            _cp_rp = []
            for ((p1, port1), (p2, port2)) in rp:
                if p1 is None or isinstance(p1, CtrlPoint):
                    _cp_rp.append([(p1, port1),None])
                if p2 is None or isinstance(p2, CtrlPoint):
                    _cp_rp[-1][1] = (p2, port2)
                    _cp_rp[-1] = tuple(_cp_rp[-1])
            _routing_paths_cp.append(_cp_rp)
        return _routing_paths_cp

    @property
    def topo(self):
        _topolist = []
        for t in self.tracks:
            if t.yard not in _topolist:
                pass

    @property
    def statelist(self):
        _statelist = []
        pass

    def get_track_by_point_port_pairs(self, p1, p1_port, p2, p2_port):
        for t in self.tracks:
            if p1 in (t.L_point, t.R_point) and p2 in (t.L_point, t.R_point):
                if p1_port in (t.L_point_port, t.R_point_port) \
                        and p2_port in (t.L_point_port, t.R_point_port):
                    return t
        return None

    def graph_constructor(self, node={}, track={}):
        '''Initialize the MultiGraph object with railroad components 
        (CP, AT as nodes, Tracks as edges)'''
        # TODO: construct the nbunch and ebunch list for Graph in network_constructor.py
        # TODO: automation of port connecting and index assignment
        # TODO: to be achieved in network_constructor.py
        TEST_SIDINGS = [Yard(self), Yard(self), Yard(self), Yard(self)]

        TEST_NODE = {   0: CtrlPoint( self, idx=0, ports=[0, 1], MP=0.0),
                        1: AutoPoint(    self, idx=1, MP=5.0),
                        2: AutoPoint(    self, idx=2, MP=10.0),
                        3: CtrlPoint( self, idx=3, ports=[0, 1, 3], ban_ports_by_port={1: [3], 3: [1]}, MP=15.0),
                        4: CtrlPoint( self, idx=4, ports=[0, 2, 1], ban_ports_by_port={0: [2], 2: [0]}, MP=20.0),
                        5: AutoPoint(    self, idx=5, MP=25.0),
                        6: CtrlPoint( self, idx=6, ports=[0, 1, 3], ban_ports_by_port={1: [3], 3: [1]}, MP=30.0),
                        7: CtrlPoint( self, idx=7, ports=[0, 2, 1], ban_ports_by_port={0: [2], 2: [0]}, MP=35.0),
                        8: AutoPoint(    self, idx=8, MP=40.0),
                        9: AutoPoint(    self, idx=9, MP=45.0),
                        10: CtrlPoint(self, idx=10, ports=[0, 1], MP=50.0)
        }   # yapf: disable

        TEST_TRACK = [
            Track(self, TEST_NODE[0], 1, TEST_NODE[1], 0),
            Track(self, TEST_NODE[1], 1, TEST_NODE[2], 0),
            Track(self, TEST_NODE[2], 1, TEST_NODE[3], 0),
            Track(self, TEST_NODE[3], 1, TEST_NODE[4], 0, edge_key=0, yard=TEST_SIDINGS[1]),
            Track(self, TEST_NODE[3], 3, TEST_NODE[4], 2, edge_key=1, yard=TEST_SIDINGS[1]),
            Track(self, TEST_NODE[4], 1, TEST_NODE[5], 0),
            Track(self, TEST_NODE[5], 1, TEST_NODE[6], 0),
            Track(self, TEST_NODE[6], 1, TEST_NODE[7], 0, edge_key=0, yard=TEST_SIDINGS[2]),
            Track(self, TEST_NODE[6], 3, TEST_NODE[7], 2, edge_key=1, yard=TEST_SIDINGS[2]),
            Track(self, TEST_NODE[7], 1, TEST_NODE[8], 0),
            Track(self, TEST_NODE[8], 1, TEST_NODE[9], 0),
            Track(self, TEST_NODE[9], 1, TEST_NODE[10], 0)
        ]   # yapf: disable

        TEST_SIDINGS = [Yard(self), Yard(self), Yard(self), Yard(self), Yard(self), Yard(self)]

        TEST_NODE = {   0: CtrlPoint( self, idx=0, ports=[0, 1], ban_ports_by_port={0: [0], 1: [1]}, MP=0.0),
                        1: AutoPoint(    self, idx=1, MP=5.0),
                        2: CtrlPoint( self, idx=2, ports=[0, 1, 3], ban_ports_by_port={1: [1, 3], 3: [3, 1]}, MP=10.0),
                        3: CtrlPoint( self, idx=3, ports=[0, 1, 3], ban_ports_by_port={1: [1, 3], 3: [3, 1]}, MP=15.0),
                        4: CtrlPoint( self, idx=4, ports=[0, 2, 1], ban_ports_by_port={0: [0, 2], 2: [2, 0]}, MP=20.0),
                        5: CtrlPoint( self, idx=5, ports=[0, 1, 3], ban_ports_by_port={1: [1, 3], 3: [3, 1]}, MP=25.0),
                        6: CtrlPoint( self, idx=6, ports=[0, 1, 3], ban_ports_by_port={1: [1, 3], 3: [3, 1]}, MP=30.0),
                        7: CtrlPoint( self, idx=7, ports=[0, 2, 1], ban_ports_by_port={0: [0, 2], 2: [2, 0]}, MP=35.0),
                        8: CtrlPoint( self, idx=8, ports=[0, 2, 1], ban_ports_by_port={0: [0, 2], 2: [2, 0]}, MP=40.0),
                        9: AutoPoint(    self, idx=9, MP=45.0),
                        10: CtrlPoint(self, idx=10, ports=[0, 1], ban_ports_by_port={0: [0], 1: [1]}, MP=50.0),
                        11: AutoPoint(   self, idx=11, MP=30.0),
                        12: AutoPoint(   self, idx=12, MP=35.0),
                        13: CtrlPoint(self, idx=13, ports=[0, 1], ban_ports_by_port={0: [0], 1: [1]}, MP=20.0),
                        14: CtrlPoint(self, idx=14, ports=[0, 1, 3], ban_ports_by_port={1: [1, 3], 3: [3, 1]}, MP=5.0),
                        15: AutoPoint(   self, idx=15, MP=10.0),
                        16: CtrlPoint(self, idx=16, ports=[0, 2, 1], ban_ports_by_port={0: [0, 2], 2: [2, 0]}, MP=15.0),
        }   # yapf: disable

        TEST_TRACK = [
            Track(self, TEST_NODE[0], 1, TEST_NODE[1], 0, mainline=True),
            Track(self, TEST_NODE[1], 1, TEST_NODE[2], 0, mainline=True),
            Track(self, TEST_NODE[2], 1, TEST_NODE[3], 0, mainline=True),
            Track(self, TEST_NODE[3], 1, TEST_NODE[4], 0, edge_key=0, yard=TEST_SIDINGS[1], mainline=True),
            Track(self, TEST_NODE[3], 3, TEST_NODE[4], 2, edge_key=1, yard=TEST_SIDINGS[1]),
            Track(self, TEST_NODE[4], 1, TEST_NODE[5], 0, mainline=True),
            Track(self, TEST_NODE[5], 1, TEST_NODE[6], 0, mainline=True),
            Track(self, TEST_NODE[6], 1, TEST_NODE[7], 0, edge_key=0, yard=TEST_SIDINGS[2], mainline=True),
            Track(self, TEST_NODE[6], 3, TEST_NODE[7], 2, edge_key=1, yard=TEST_SIDINGS[2]),
            Track(self, TEST_NODE[7], 1, TEST_NODE[8], 0, mainline=True),
            Track(self, TEST_NODE[8], 1, TEST_NODE[9], 0, mainline=True),
            Track(self, TEST_NODE[9], 1, TEST_NODE[10],0, mainline=True),
            Track(self, TEST_NODE[5], 3, TEST_NODE[11],0, yard=TEST_SIDINGS[2]),
            Track(self, TEST_NODE[11],1, TEST_NODE[12],0, yard=TEST_SIDINGS[2]),
            Track(self, TEST_NODE[12],1, TEST_NODE[8], 2, yard=TEST_SIDINGS[2]),
            Track(self, TEST_NODE[2], 3, TEST_NODE[14],0, mainline=True),
            Track(self, TEST_NODE[14],3, TEST_NODE[15],0, yard=TEST_SIDINGS[3]),
            Track(self, TEST_NODE[15],1, TEST_NODE[16],2, yard=TEST_SIDINGS[3]),
            Track(self, TEST_NODE[14],1, TEST_NODE[16],0, yard=TEST_SIDINGS[3], mainline=True),
            Track(self, TEST_NODE[16],1, TEST_NODE[13],0, mainline=True),
        ]   # yapf: disable

        _node = TEST_NODE if not node else node
        nbunch = [_node[i] for i in range(len(_node))]
        _track = TEST_TRACK if not track else track
        ebunch = [_track[i] for i in range(len(_track))]

        # _node and _track will be parameters passed from outside in the future development
        G = nx.MultiGraph()
        for n in nbunch:
            G.add_node(n, attr=n.__dict__, instance=n)
            # __dict__ of instances (CPs, ATs, Tracks) is pointing the same
            # attribute dictionary as the node in the MultiGraph

        for t in ebunch:
            G.add_edge(t.L_point,
                       t.R_point,
                       key=t.edge_key,
                       attr=t.__dict__,
                       instance=t)
            # __dict__ of instances (CPs, ATs, Tracks) is pointing the same
            # attribute dictionary as the edge in the MultiGraph
            # key is the index of parallel edges between two nodes
            t.tracks.append(t)
            t.L_point.track_by_port[t.L_point_port] = t.R_point.track_by_port[
                t.R_point_port] = t
            G[t.L_point][t.R_point][t.edge_key]['weight_mainline'] \
                = t.mainline_weight

        for i in G.nodes():  # register neighbor nodes as observers to each node
            i.neighbor_nodes.extend([n for n in G.neighbors(i)])
            for n in G.neighbors(i):
                i.add_observer(n)
        return G

    def graph_extractor(self, G):
        '''
        Extract the skeletion MultiGraph with only CtrlPoints and Bigblocks
        ----------
        Parameter:
            G: MultiGraph instance of the raw network with Track as edges.
        ----------
        Return:
            F: MultiGraph instance with BigBlock as edges.
        '''
        F = G.copy()

        # F is a shallow copy of G: attrbutes of G/F components
        # are pointing at the same memory.
        def _node_vars(node):
            at_neighbor = [j for j in F.neighbors(node)]
            assert len(at_neighbor) == len(F.edges(node)) == 2
            edgetrk_L_points = [
                F[at_neighbor[0]][node][0]['instance'].L_point,
                F[node][at_neighbor[1]][0]['instance'].L_point
            ]
            edgetrk_R_points = [
                F[at_neighbor[0]][node][0]['instance'].R_point,
                F[node][at_neighbor[1]][0]['instance'].R_point
            ]
            edgetrk_L_points.remove(node)
            edgetrk_R_points.remove(node)
            new_L_point, new_R_point = edgetrk_L_points[0], edgetrk_R_points[0]
            old_L_trk = F[new_L_point][node][0]['instance']
            old_R_trk = F[node][new_R_point][0]['instance']
            return new_L_point, new_R_point, old_L_trk, old_R_trk

        for i in G.nodes():
            # only use G.nodes() instead of F.nodes() to get original nodes
            # to avoid dictionary size changing issues.
            # all the following graph updates are targeted on F
            if i.type == 'at':
                new_L_point, new_R_point, old_L_trk, old_R_trk = _node_vars(i)
                new_track = Track(self,
                                  new_L_point,
                                  F[new_L_point][i][0]['instance'].L_point_port,
                                  new_R_point,
                                  F[i][new_R_point][0]['instance'].R_point_port,
                                  edge_key=0)
                if len(old_L_trk.tracks) == 1 and old_L_trk in old_L_trk.tracks:
                    new_track.tracks.append(old_L_trk)
                else:
                    new_track.tracks.extend([t for t in old_L_trk.tracks 
                                                if t not in new_track.tracks])
                if len(old_R_trk.tracks) == 1 and old_R_trk in old_R_trk.tracks:
                    new_track.tracks.append(old_R_trk)
                else:
                    new_track.tracks.extend([t for t in old_R_trk.tracks 
                                                if t not in new_track.tracks])
                F.remove_node(i)
                F.add_edge(new_L_point,
                           new_R_point,
                           attr=new_track.__dict__,
                           instance=new_track)
                # MultiGraph parallel edges are auto-keyed (0, 1, 2...)
                # default 0 as mainline, idx as track number

        for (u, v, k) in F.edges(keys=True):
            _L_point, _R_point = \
                F[u][v][k]['instance'].L_point, F[u][v][k]['instance'].R_point
            big_block_instance = BigBlock(self,
                                        _L_point,
                                        F[u][v][k]['instance'].L_point_port,
                                        _R_point,
                                        F[u][v][k]['instance'].R_point_port,
                                        edge_key=k,
                                        raw_graph=G,
                                        cp_graph=F)
            _L_point.bigblock_by_port[F[u][v][k]
                            ['instance'].L_point_port] = big_block_instance
            _R_point.bigblock_by_port[F[u][v][k]
                            ['instance'].R_point_port] = big_block_instance
            for t in F[u][v][k]['instance'].tracks:
                t.bigblock = big_block_instance
                if t not in big_block_instance.tracks:
                    big_block_instance.tracks.append(t)
            # get the list of track unit components of a bigblock, 
            # and record in the instance
            for t in big_block_instance.tracks:
                t.bigblock = big_block_instance
            big_block_instance.mainline = True if all([t.mainline 
                    for t in big_block_instance.tracks]) else False
            F[u][v][k]['attr'] = big_block_instance.__dict__
            F[u][v][k]['instance'] = big_block_instance
            F[u][v][k]['weight_mainline'] = big_block_instance.mainline_weight
            
        return F

    def generate_train(self, init_point, init_port, dest_point, dest_port, **kwargs):
        '''
            Generate train only.'''
        _new_train = None
        length = 1 if kwargs.get('length') is None else kwargs.get('length')
        if self.capacity_enterable(init_point, dest_point):
            init_segment = ((None, None), (init_point, init_port)) \
                if not init_point.track_by_port.get(init_port)\
                else (( init_point.track_by_port[init_port]
                        .shooting_point(point=init_point),
                        init_point.track_by_port[init_port]
                        .shooting_port(point=init_point)),
                      (init_point, init_port))
            dest_segment = ((dest_point, dest_port), (None, None)) \
                if not dest_point.track_by_port.get(dest_port)\
                else (( dest_point.track_by_port[dest_port]
                        .shooting_point(point=dest_point),
                        dest_point.track_by_port[dest_port]
                        .shooting_port(point=dest_point)),
                      (dest_point, dest_port))
            init_track = self.get_track_by_point_port_pairs(
                init_segment[0][0], init_segment[0][1], 
                init_segment[1][0], init_segment[1][1])
            if not init_track:
                _new_train = Train(
                    system=self,
                    init_segment=init_segment,
                    dest_segment=dest_segment,
                    max_spd=random.choice(self.spd_container),
                    max_acc=random.choice(self.acc_container),
                    max_dcc=random.choice(self.dcc_container),
                    length=length)
            elif init_track.is_Occupied:
                print('\tWarning: cannot generate train: track is occupied. \
                    Hold new train for track availablity.')
            elif not init_track.routing:
                _new_train = Train(
                    system=self,
                    init_segment=init_segment,
                    dest_segment=dest_segment,
                    max_spd=random.choice(self.spd_container),
                    max_acc=random.choice(self.acc_container),
                    max_dcc=random.choice(self.dcc_container),
                    length=length)
            elif Train.sign_MP(init_segment) == init_track.sign_routing(
                    init_track.routing):
                _new_train = Train(
                    system=self,
                    init_segment=init_segment,
                    dest_segment=dest_segment,
                    max_spd=random.choice(self.spd_container),
                    max_acc=random.choice(self.acc_container),
                    max_dcc=random.choice(self.dcc_container),
                    length=length)
            else:
                print('\tWarning: cannot generate train: confliting routing. \
                    Hold new train for routing availablity.')
        else:
            print('\tWarning: cannot generate train: Capacity Maxed-out. \
                Hold new train for capacity.')
        return _new_train

    def capacity_enterable(self, init_point, dest_point):
        '''
            Determines if a train could cross init_point towards dest_point.'''
        _parallel_tracks = self.num_parallel_tracks(init_point, dest_point)
        _outbound_trains = self.get_trains_between_points(from_point=init_point,
                                                          to_point=dest_point,
                                                          obv=True)
        _inbound_trains = self.get_trains_between_points(from_point=dest_point,
                                                         to_point=init_point,
                                                         obv=True)
        _occupied_parallel_tracks = self.num_occupied_parallel_tracks(
            init_point, dest_point)
        return True if min(len(_outbound_trains), len(_inbound_trains))\
            <= _parallel_tracks - _occupied_parallel_tracks else False

    def num_parallel_tracks(self, init_point, dest_point):
        _mainline_path = shortest_path(self.G_origin, source=init_point, 
                                target=dest_point, weight='weight_mainline')
        _head = _mainline_path.pop(0)
        count = 0
        _traversed = []
        while _mainline_path:
            for t in _traversed:
                if t in _mainline_path:
                    _mainline_path.remove(t)
            for p in _mainline_path:
                if len(list(all_simple_paths(self.G_origin, _head, p))) == 1:
                    _traversed.append(p)
                    continue
                else:
                    count += len(list(all_simple_paths(self.G_origin, 
                                                        _head, p))) - 1
                    _traversed.append(p)
                    _head = p
                    break
        return count

    def num_occupied_parallel_tracks(self, init_point, dest_point):
        '''
        要分情况讨论的。
        '''
        _all_trains = self.get_trains_between_points(from_point=init_point,
                                                     to_point=dest_point,
                                                     obv=True,
                                                     rev=True)
        test_G = self.G_origin.copy()
        count = 0
        for t in _all_trains:
            test_G.remove_edge(t.curr_routing_path_segment[0][0],
                               t.curr_routing_path_segment[1][0])
            if nx.has_path(test_G, init_point, dest_point) and \
                Train.sign_MP(t.curr_routing_path_segment) * \
                    (dest_point.MP-init_point.MP) > 0:
                count += 1
        return count

    def get_trains_between_points(self,
                                  from_point,
                                  to_point,
                                  obv=False,
                                  rev=False):
        '''
            Given a pair of O-D in the system, return all trains running between 
            this pair of O-D nodes.
            @option: filter trains running at the obversed/reversed direction 
            compared with the from-to path.'''
        all_paths = list(all_simple_paths(self.G_origin, from_point, to_point))
        _trains_all = []
        _trains_obv_dir = []
        _trains_rev_dir = []
        for p in all_paths:
            for i in range(len(p) - 1):
                for k in list(self.G_origin[p[i]][p[i + 1]]):
                    for t in self.G_origin[p[i]][p[i + 1]][k]['instance'].train:
                        if t.curr_routing_path_segment[0][0] in (p[i], p[i+1]) \
                        and t.curr_routing_path_segment[1][0] in (p[i], p[i+1]):
                            if t not in _trains_all:
                                _trains_all.append(t)
                            if (t.curr_routing_path_segment[0][0],
                                    t.curr_routing_path_segment[1][0]) == \
                                        (p[i], p[i + 1]):
                                if t not in _trains_obv_dir:
                                    _trains_obv_dir.append(t)
                            if (t.curr_routing_path_segment[0][0],
                                    t.curr_routing_path_segment[1][0]) == \
                                        (p[i + 1], p[i]):
                                if t not in _trains_rev_dir:
                                    _trains_rev_dir.append(t)
        if obv == True and rev == True: return _trains_all
        elif obv == True: return _trains_obv_dir
        elif rev == True: return _trains_rev_dir
        else: return []

    def launch(self, launch_duration, auto_generate_train=False):
        logging.info("Thread %s: starting", 'simulator')
        while self.sys_time - self.init_time <= launch_duration:
            for t in self.trains:
                try:
                    t.request_routing()
                    t.update_acc()
                except:
                    print(t)
                    raise(ValueError('Raise Error to Stop Simulation'))
            if auto_generate_train:
                if self.sys_time+self.refresh_time - self.last_train_init_time \
                        >= self.headway:
                    if not self.signal_points[0].curr_train_with_route.keys():
                        if all([t.curr_routing_path_segment != 
                                ((None,None),(self.signal_points[0],0)) 
                                for t in self.trains.all_trains]):
                            if not self.tracks[0].train:
                                t = self.generate_train(self.signal_points[0],
                                                        0, 
                                                        self.signal_points[10], 
                                                        1, 
                                                        length=1)
            self.sys_time += self.refresh_time
        logging.info("Thread %s: finishing", 'simulator')

    def update_routing(self):
        '''
            TODO: Combine dispatcher actions'''
        for trn in self.trains.all_trains:
            if not trn.curr_sig:
                pass
            elif not trn.curr_sig.route:
                if self.capacity_enterable(trn.curr_sigpoint,
                                           trn.intended_sigpoint):
                    trn.curr_sigpoint.open_route(
                        (trn.curr_sigport, trn.intended_sigport))

    def refresh(self):
        '''
            TODO: Combine dispatcher actions; routing update actions.'''
        # self.generate_train()
        self.update_routing()
        for t in self.trains.all_trains:
            t.update_acc()
        for i, tr in enumerate(self.trains):
            tr.rank = i
        self.sys_time += self.refresh_time


if __name__ == '__main__':
    sim_init_time = datetime.strptime('2018-01-10 10:00:00',"%Y-%m-%d %H:%M:%S")
    sim_term_time = datetime.strptime('2018-01-10 15:30:00',"%Y-%m-%d %H:%M:%S")
    spd_container = [random.uniform(0.01, 0.02) for i in range(20)]
    acc_container = [0.5*random.uniform(2.78e-05*0.85, 2.78e-05*1.15) 
                        for i in range(20)]
    dcc_container = [0.2*random.uniform(2.78e-05*0.85, 2.78e-05*1.15) 
                        for i in range(20)]
    headway = 300 + random.random() * 400
    sys = System(sim_init_time, spd_container, acc_container, dcc_container,
                term_time=sim_term_time,
                dos_period=['2018-01-10 11:30:00', '2018-01-10 12:30:00'],  
                dos_pos=(15,20),
                headway=headway, 
                refresh_time=50)
