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
import os
import random
import sys
sys.path.append(
    'D:\\Users\\Hegxiten\\workspace\\Rutgers_Railway_security_research\\OOD_Train'
)
from collections.abc import MutableSequence
from datetime import datetime, timedelta

import numpy as np

from signaling import AutoPoint, AutoSignal, CtrlPoint, HomeSignal
from rail_networkx import all_simple_paths, shortest_path


class TrainList(MutableSequence):
    """
        A list-like container for Train instances wihtin a simulation system.
        TODO: implement customized attributes of TrainList: 
            append, insert, __getitem__, __setitem__, __delitem__, etc."""
    def __init__(self):
        self._uptrains = []
        self._downtrains = []

    @property
    def uptrains(self):   
        self._uptrains.sort()
        return self._uptrains
    
    @property
    def downtrains(self):
        self._downtrains.sort()
        return self._downtrains

    @property
    def all_trains(self):
        _all_trains = self.uptrains + self.downtrains
        _all_idx = [t.train_idx for t in _all_trains]
        return [t for _,t in sorted(zip(_all_idx,_all_trains))]
    
    @property
    def all_trains_by_MP(self):
        _all_MP = [t.curr_MP for t in self.all_trains]
        return [t for _,t in sorted(zip(_all_MP,self.all_trains))]

    def __str__(self):
        return str(self.all_trains)

    def __repr__(self):
        return "<{0} {1}>".format(self.__class__.__name__, self.all_trains)

    def __len__(self):
        """List length"""
        return len(self.all_trains)

    def __getitem__(self, ii):
        return self.all_trains[ii]

    def __delitem__(self, ii):
        _to_del = self.all_trains[ii]
        if _to_del in self._uptrains:
            _idx = self._uptrains.index(_to_del)
            del self._uptrains[_idx]
        if _to_del in self._downtrains:
            _idx = self._downtrains.index(_to_del)
            del self._downtrains[_idx]
            
    def __setitem__(self, ii, trn):
        raise Exception("Cannot Set Directly in ")

    def insert(self, trn):
        if trn.uptrain:
            self._uptrains.append(trn)
        if trn.downtrain:
            self._downtrains.append(trn)
    
    append = insert


class Train():
    """
        Constructor Parameters
        ----------
        :system: System instance
            System the train is operating at. Must be registered when initialize.
        :length: (**kw), miles
            Train length that can occupy multiple tracks. 0 by default.
        :init_segment: 2-element-tuple: ((Point, Port), (Point, Port))
            Describes the initial direction and section of track of the train."""

    @staticmethod
    def abs_brake_distance(curr_spd, tgt_spd, brake_dcc):
        '''
            :curr_spd:  Current speed in miles/sec.
            :tgt_spd:   Current target speed in miles/sec.
            :brake_dcc: Maximum decleration in miles/(sec)^2.
            @return:
                Absolute value of brake distance in miles at input status.'''
        if brake_dcc == 0:
            return 0
        else:
            return abs((tgt_spd) ** 2 - (curr_spd) ** 2)/(2*abs(brake_dcc)) \
                if abs(tgt_spd) < abs(curr_spd)\
                else 0

    @staticmethod
    def sign_MP(rp_seg):
        '''
            :rp_seg: 2-element-tuple: ((Point, Port),(Point, Port))
                Routing path segment of a train, describing its direction.
            @return:
                The sign (+1/-1) of train's direction (speed).'''
        if rp_seg[0][0] and rp_seg[1][0]:
            if rp_seg[1][0].signal_by_port[rp_seg[1][1]].MP > rp_seg[0][
                    0].signal_by_port[rp_seg[0][1]].MP:
                return 1
            elif rp_seg[1][0].signal_by_port[rp_seg[1][1]].MP < rp_seg[0][
                    0].signal_by_port[rp_seg[0][1]].MP:
                return -1
            else:
                raise ValueError('Undefined MP direction')
        elif not rp_seg[0][0]:  # initiating
            if rp_seg[1][0].signal_by_port[rp_seg[1][1]].MP == \
                    min(rp_seg[1][0].track_by_port[rp_seg[1][0].opposite_port(rp_seg[1][1])].MP):
                return 1
            elif rp_seg[1][0].signal_by_port[rp_seg[1][1]].MP == \
                    max(rp_seg[1][0].track_by_port[rp_seg[1][0].opposite_port(rp_seg[1][1])].MP):
                return -1
            else:
                raise ValueError('Undefined MP direction')
        elif not rp_seg[1][0]:  # terminating
            if rp_seg[0][0].signal_by_port[rp_seg[0][1]].MP == \
                    max(rp_seg[0][0].track_by_port[rp_seg[0][0].opposite_port(rp_seg[0][1])].MP):
                return 1
            elif rp_seg[0][0].signal_by_port[rp_seg[0][1]].MP == \
                    min(rp_seg[0][0].track_by_port[rp_seg[0][0].opposite_port(rp_seg[0][1])].MP):
                return -1
            else:
                raise ValueError('Undefined MP direction')

    def __init__(self, system, init_segment, dest_segment,
                    max_spd, max_acc, max_dcc, **kwargs):
        self.system = system
        self.init_segment , self.dest_segment = init_segment, dest_segment
        self.init_pointport = init_segment[1]
        self.dest_pointport = dest_segment[0]
        self.max_spd = max_spd
        self.max_acc = max_acc
        self.max_dcc = max_dcc
        
        self.length = 1 \
            if kwargs.get('length') is None else kwargs.get('length')

        self._curr_routing_path_segment = self.init_segment
        self._curr_occupying_routing_path = [self._curr_routing_path_segment]
        self._curr_MP = self.curr_sig.MP
        self._rear_curr_MP = self.curr_MP - self.length * \
            self.sign_MP(self.curr_routing_path_segment)
        self.train_idx = len(self.system.trains)
        self.symbol = 2 * self.train_idx \
            if self.uptrain else 2 * self.train_idx + 1
        self.system.trains.append(self)

        self._curr_speed = 0
        self._curr_acc = 0
        # initial speed limit, considering both cases:
        # with current track and without current tracks
        # default 20 mph even with both ends restricting. Non-zero value
        self._curr_spd_lmt_abs = min(20/3600, self.curr_track.allow_sp) \
            if self.curr_track else min(20/3600, float('inf'))
        self._stopped = True

        self.time_pos_list = []
        self.rear_time_pos_list = []
        self.pos_spd_list = []

        if self.curr_track:
            if self not in self.curr_track.train:
                if self.curr_track.train:
                    print(
                        '\t Train initialization Warning: adding new train to a track already occupied!'
                    )
                self.curr_track.train.append(self)
        self.system.last_train_init_time = self.system.sys_time

    def __repr__(self):
        return 'train idx:{} occupying:{} head MP:{} rear MP:{}'\
            .format(str(self.train_idx).rjust(2,' '),
                    self.curr_occupying_routing_path,
                    str("%.2f" % round(self.curr_MP, 2)).rjust(5, ' '),
                    str("%.2f" % round(self.rear_curr_MP, 2)).rjust(5, ' '))
    @property
    def same_way_trains(self):
        if self.uptrain:
            return self.system.trains.uptrains
        if self.downtrain:
            return self.system.trains.downtrains
    
    @property
    def rank(self):
        '''
            rank of the train starting from the first train to the last. 
            First: 0; Last: len(self.system.trains) - 1
            TODO: Implement rank for both directions.'''
        return self.same_way_trains.index(self)

    @property
    def curr_sign(self):    return self.sign_MP(self.curr_routing_path_segment)

    @property
    def uptrain(self):      return True if self.curr_sign == -1 else False

    @property
    def downtrain(self):    return True if self.curr_sign == +1 else False

    @property
    def terminated(self):
        '''
            Status property shows if the train has exited the rail network
            @return: True or False'''
        return True if (not self.curr_routing_path_segment[1][0]
                        and not self.rear_curr_track) else False

    @property
    def stopped(self):
        '''
            Status property shows if the train is stopped because of:
                1. terminated out of the system;
                2. no and awating for signal to clear;
            @return: True or False'''
        if self.terminated:
            # terminated trains are stopped trains
            self._stopped = True
        elif self.curr_speed == 0 and self.curr_target_spd_abs == 0:
            # trains with 0 speed and 0 target speed are stopped trains
            # Note: 0 speed trains with non-zero target speed (clear signal ahead)
            # are not stopped. Therefore, trains with non-zero acceleration are
            # not stopped trains.
            self._stopped = True
        else:
            self._stopped = False
        return self._stopped

    @property
    def curr_track(self):
        '''
            The current Track instance the head of the train is moving upon'''
        return self.system.get_track_by_point_port_pairs(
            self.curr_prev_sigpoint, self.curr_prev_sigport, self.curr_sigpoint,
            self.curr_sigport)

    @property
    def rear_curr_track(self):
        '''
            The current Track instance the rear of the train is moving upon'''
        return self.system.get_track_by_point_port_pairs(
            self.rear_curr_prev_sigpoint, self.rear_curr_prev_sigport,
            self.rear_curr_sigpoint, self.rear_curr_sigport)

    @property
    def curr_occupying_routing_path(self):
        '''
            A list of routing path tuples being occupied by the train.
            The routing path tuples are in order from train head to rear.'''
        return self._curr_occupying_routing_path \
            if self.length != 0 else [self.curr_routing_path_segment]

    @property
    def curr_tracks(self):
        '''
            A list of Track instances being occupied by the train.
            The Track instances are in order from train head to rear.'''
        _curr_tracks = []
        for r in self.curr_occupying_routing_path:
            _track = self.system.get_track_by_point_port_pairs(
                r[0][0], r[0][1], r[1][0], r[1][1])
            if _track:
                _curr_tracks.append(_track)
        return _curr_tracks

    @property
    def curr_bigblock_routing(self):
        '''
            The routing tuple of the bigblock where the train head is moving on.'''
        return self.curr_track.bigblock.routing

    @property
    def curr_routing_path(self):
        '''
            A list of tuples describing the current routing of the train.
            Routing segments tuples include both ahead and behind the train.
            The tuples starts from the head of its movement authority limit
            shooting to the end of the limit. 
            Each tuple is a routing path segment.'''
        return self.curr_track.curr_routing_path if self.curr_track else None

    @property
    def curr_routing_path_ahead(self):
        '''
            A list of tuples describing the current routing ahead of the train.
            The routing tuples (movement authority limits), even granted behind
            the train, are ignored because of no effects to itself.'''

        return self.curr_routing_path[
            self.curr_routing_path.index(self.curr_routing_path_segment):] \
                if self.curr_routing_path else None

    @property
    def curr_routing_path_segment(self):
        '''
            Return a tuple of the current segment the train head is moving upon;
            expressed in a 2-element-tuple: ((Point1, Port1), (Point2, Port2)).
                Point2/Port2: SignalPoint/Port the train head is operating to.
                Point1/Port1: the opposite SignalPoint/Port of the track segment.
            None: undefined SignalPoint/Port in the network.
                Mostly appears when initialting or terminating.
            When a train exists, it has a True curr_routing_path_segment even if
            the train is not granted for any routing paths.
                With no routing paths granted: the tuple describes the position
                and its potential of movement (moving direction) of the train;
                With routing paths granted: the tuple describes the position with
                a valid routing paths.'''
        return self._curr_routing_path_segment

    @curr_routing_path_segment.setter
    def curr_routing_path_segment(self, new_segment):
        '''
            Setter for curr_routing_path_segment.
            Once set, the direction of the train & occupied track are confined.'''
        assert isinstance(new_segment, tuple) and len(new_segment) == 2
        self._curr_routing_path_segment = new_segment

    @property
    def all_paths_ahead(self):
        return list(all_simple_paths(self.system.G_origin, 
            self.curr_sigpoint, self.dest_pointport[0]))

    @property
    def all_routes_ahead(self):
        return list(self.system.dispatcher.all_routes_generator(self.curr_sigpoint, 
            self.curr_sigport, self.dest_pointport[0], self.dest_pointport[1]))

    @property
    def curr_sigpoint(self):
        '''
            The SignalPoint instance the train head is moving towards currently.'''
        return self.curr_routing_path_segment[1][0]

    @property
    def curr_prev_sigpoint(self):
        '''
            The SignalPoint instance train head has just passed.'''
        return self.curr_routing_path_segment[0][0]

    @property
    def curr_sigport(self):
        '''
            The port of curr_sigpoint train head is moving towards.'''
        return self.curr_routing_path_segment[1][1]

    @property
    def curr_prev_sigport(self):
        '''
            The port of curr_prev_sigpoint that facing towards the train head.
            Note:
                this is not the port that the train head has just passed.
                It is rather the port on the BACK of the port of the signal the
                train head has just passed/acquired aspect from.'''
        return self.curr_routing_path_segment[0][1]

    @property
    def rear_curr_sigpoint(self):
        '''
            The SignalPoint instance the train rear is moving towards currently.'''
        return self.curr_occupying_routing_path[-1][1][0]

    @property
    def rear_curr_prev_sigpoint(self):
        '''
            The SignalPoint instance the train rear has just passed.'''
        return self.curr_occupying_routing_path[-1][0][0]

    @property
    def rear_curr_sigport(self):
        '''
            The port of rear_curr_sigpoint the train rear is moving towards.'''
        return self.curr_occupying_routing_path[-1][1][1]

    @property
    def rear_curr_prev_sigport(self):
        '''
            The port of rear_curr_prev_sigpoint that facing towards the rear of the train.
            Note:
                this is not the port that the train rear has just passed. It is rather the
                port on the BACK of the port the train rear has just passed/cleared signal.'''
        return self.curr_occupying_routing_path[-1][0][1]

    @property
    def curr_ctrl_point(self):
        '''
            The closest CtrlPoint instance the train head is moving towards.'''
        return self.curr_track.bigblock.shooting_point(sign_MP=self.sign_MP(self.curr_routing_path_segment))\
            if self.curr_track else self.curr_sigpoint

    @property
    def curr_ctrl_pointport(self):
        '''
            The port of curr_ctrl_point the train head is moving towards.'''
        return self.curr_track.bigblock.shooting_port(sign_MP=self.sign_MP(self.curr_routing_path_segment))\
            if self.curr_track else self.curr_sigport

    @property
    def curr_home_sig(self):
        '''
            The closest HomeSignal instance the train head is moving towards.'''
        return self.curr_ctrl_point.signal_by_port[self.curr_ctrl_pointport]\
            if self.curr_ctrl_point\
            else None

    @property
    def curr_route_cancelable(self):
        if not self.curr_home_sig: return True
        if not self.curr_home_sig.route: return True
        if self.curr_target_spd_abs == 0: return True
        return False

    @property
    def curr_sig(self):
        '''
            The Signal instance the train head is moving towards.'''
        return self.curr_sigpoint.signal_by_port[self.curr_sigport] \
            if self.curr_sigpoint \
            else None

    @property
    def rear_curr_sig(self):
        '''
            The Signal instance the train rear is moving towards.'''
        return self.rear_curr_sigpoint.signal_by_port[self.rear_curr_sigport] \
            if self.rear_curr_sigpoint \
            else None

    @property
    def curr_MP(self):  # in miles
        '''
            The current MilePost of the train head, in miles.'''
        if self.curr_track:
            assert min(self.curr_track.MP) <= self._curr_MP <= max(
                self.curr_track.MP)
        return self._curr_MP

    @curr_MP.setter
    def curr_MP(self, new_MP):
        '''
            Setter for the curr_MP. 
            Setting under different conditions with actions to trigger.'''
        _delta_s = new_MP - self._curr_MP
        # since the intended new_MP to set is based on distance delta, back-calculate the delta_s
        if self.curr_sigpoint:
            # 1 the train head is not out of the system (has a current SignalPoint)
            if self.curr_track:
                # 1.1 the train head is within the system (has a occupied track)
                if min(self.curr_track.MP) < new_MP < max(self.curr_track.MP):
                    # 1.1.1 train head is confined in the same track after the update
                    self._curr_MP = self._curr_MP + _delta_s
                else:
                    # 1.1.2 train head entering a new track (crossing current SignalPoint)
                    # Note: new track may have a separate MP system (switching corridors)
                    _new_prev_sig_MP = self.curr_sigpoint.signal_by_port[
                        self.curr_sig.route[1]].MP
                    _curr_sig_MP = self.curr_sigpoint.signal_by_port[
                        self.curr_sig.route[0]].MP
                    self.cross_sigpoint(self.curr_sigpoint, self._curr_MP,
                                        new_MP)
                    self._curr_MP = _new_prev_sig_MP + (new_MP - _curr_sig_MP)
                    # if entering a separate MP Track, interpolate distance at MP changing point
            else:
                # 1.2 the train head is initiating, crossing the entry SignalPoint (CtrlPoint)
                _new_prev_sig_MP = self.curr_sigpoint.signal_by_port[
                    self.curr_sig.route[1]].MP
                _curr_sig_MP = self.curr_sigpoint.signal_by_port[
                    self.curr_sig.route[0]].MP
                self.cross_sigpoint(self.curr_sigpoint, self._curr_MP, new_MP)
                self._curr_MP = _new_prev_sig_MP + (new_MP - _curr_sig_MP)
                # if entering a separate MP Track, interpolate distance at MP changing point
        else:
            # 2 the train head is out of the system (going to terminate)
            self._curr_MP = self._curr_MP + _delta_s
        self.rear_curr_MP = self.rear_curr_MP + _delta_s
        # set the new rear MP using the same delta_s
        self.time_pos_list.append([self.system.sys_time, self.curr_MP])

    @property
    def rear_curr_MP(self):  # in miles
        '''
            The current MilePost of the train rear, in miles.'''
        return self._rear_curr_MP

    @rear_curr_MP.setter
    def rear_curr_MP(self, new_rear_MP):
        '''
            Setter for the rear_curr_MP. 
            Setting under different conditions with actions to trigger.'''
        if not self.length:
            # 1 if the train is simplified with 0 length, set rear_curr_MP the same as curr_MP
            self._rear_curr_MP = self.curr_MP
        else:
            # 2 the train has non-zero length
            _delta_s = new_rear_MP - self._rear_curr_MP
            # since the intended new_MP to set is based on distance delta, back-calculate the delta_s
            if self.rear_curr_sigpoint:
                # 2.1 the train rear is not out of the system (rear has a current SignalPoint)
                if self.rear_curr_track:
                    # 2.1.1 not initiating
                    if min(self.rear_curr_track.MP) < new_rear_MP < max(
                            self.rear_curr_track.MP):
                        # 2.1.1.1 train rear is confined in the same track after the update
                        self._rear_curr_MP = self._rear_curr_MP + _delta_s
                    else:
                        # 2.1.1.2 train rear is entering a new track (crossing its current SignalPoint)
                        # Note: new track of the train rear may have a separate MP system
                        assert len(self.curr_occupying_routing_path) >= 2
                        _rear_route = (
                            self.curr_occupying_routing_path[-1][1][1],
                            self.curr_occupying_routing_path[-2][0][1])
                        _new_rear_prev_sig_MP = self.rear_curr_sigpoint.signal_by_port[
                            _rear_route[1]].MP
                        _rear_curr_sig_MP = self.rear_curr_sigpoint.signal_by_port[
                            _rear_route[0]].MP
                        self.rear_cross_sigpoint(self.rear_curr_sigpoint,
                                                 self._rear_curr_MP,
                                                 new_rear_MP)
                        self._rear_curr_MP = _new_rear_prev_sig_MP + \
                            (new_rear_MP - _rear_curr_sig_MP)
                        # if entering a separate MP Track, interpolate distance at MP changing point
                else:
                    # 2.2 initiating, train rear is going to cross the entry CtrlPoint
                    assert len(self.curr_occupying_routing_path) >= 2
                    if (new_rear_MP - self.rear_curr_sig.MP) * (
                            self._rear_curr_MP - self.rear_curr_sig.MP) < 0:
                        # 2.2.1 rear entering the new track (crossing entry CP)
                        # Note: the train rear is sure to acquire a new MP system
                        _rear_route = \
                            (self.curr_occupying_routing_path[-1][1][1],
                             self.curr_occupying_routing_path[-2][0][1])
                        _new_rear_prev_sig_MP = self.rear_curr_sigpoint.signal_by_port[
                            _rear_route[1]].MP
                        _rear_curr_sig_MP = self.rear_curr_sigpoint.signal_by_port[
                            _rear_route[0]].MP
                        self.rear_cross_sigpoint(self.rear_curr_sigpoint,
                                                 self._rear_curr_MP,
                                                 new_rear_MP)
                        self._rear_curr_MP = _new_rear_prev_sig_MP + \
                            (new_rear_MP - _rear_curr_sig_MP)
                        # interpolate distance at the MP changing point
                    else:
                        # 2.2.2 not crossing the entry CtrlPoint; train rear not in the system yet
                        self._rear_curr_MP += _delta_s
            else:
                raise ValueError(
                    'Setting Undefined rear MP: original:{0}, new:{1}, \
                        rear track:{2}'
                    .format(self._rear_curr_MP, 
                            new_rear_MP,
                            self.rear_curr_track))
        self.rear_time_pos_list.append(
            [self.system.sys_time, self.rear_curr_MP])

    @property
    def curr_speed(self):  # in miles/sec, with (+/-)
        '''
            The current speed of the train. Consistant for the whole length.'''
        return self._curr_speed

    @curr_speed.setter
    def curr_speed(self, new_speed):
        '''
            Setter for curr_speed. Setting the speed value under different conditions:
            ----------
            1: new speed after update maintains the same sign as old speed.
                1.1: the train is accelerating:
                    1.1.1: the train is accelerating into its current speed limit.
                        current speed value crossing the absolute value of its current speed limit
                        set the new current speed as the speed limit with sign.
                    1.1.2: the train is undergoing normal acceleration and not reaching its speed limit.
                1.2: the train is decelerating:
                    1.2.1: the train is decelerating into its current target speed. 
                        current speed value crossing the absolute value of its current target speed.
                        stop deceleration, maintain current speed (with sign) at its target speed value.
                    1.2.2: the train is undergoing normal deceleration and not reaching its target speed.
                1.3: the train is neither accelerating nor decelerating. new_speed == _old_speed.
            2: new speed after update has the opposite sign to old_speed (speed value crossing zero):
                the train is decelerating into a complete stop, setting the speed as 0.'''
        assert self.curr_brake_distance_abs <= self.curr_dis_to_curr_sig_abs
        assert abs(self.curr_speed) <= self.curr_spd_lmt_abs
        # assert always-on braking distance/speed limit satisfaction to find bugs
        _old_speed = self._curr_speed
        # store the old speed value internally for logical processing
        # the intended new_speed to set is based on speed delta acquired by its 
        # current acceleration
        if new_speed * _old_speed >= 0:  # 1
            if abs(new_speed) > abs(_old_speed):  # 1.1
                if (self.curr_spd_lmt_abs - abs(new_speed)) * (
                        self.curr_spd_lmt_abs - abs(_old_speed)) < 0:
                    # 1.1.1
                    self._curr_speed = self.curr_spd_lmt_abs if _old_speed >= 0 else -self.curr_spd_lmt_abs
                else:  # 1.1.2
                    self._curr_speed = new_speed
            elif abs(new_speed) < abs(_old_speed):  # 1.2
                if (self.curr_target_spd_abs - abs(new_speed)) * (
                        self.curr_target_spd_abs - abs(_old_speed)) < 0:
                    # 1.2.1
                    self._curr_speed = self.curr_target_spd_abs if _old_speed >= 0 else - \
                        self.curr_target_spd_abs
                else:  # 1.2.2
                    self._curr_speed = new_speed
            else:  # 1.3
                self._curr_speed = new_speed
        elif new_speed * _old_speed < 0:  # 2
            self._curr_speed = 0
        else:
            raise ValueError(
                'Setting Undefined speed value: original:{0}, new:{1}'
                .format(_old_speed, new_speed))
        assert self.curr_brake_distance_abs <= self.curr_dis_to_curr_sig_abs
        assert abs(self.curr_speed) <= self.curr_spd_lmt_abs
        # assert always-on braking distance/speed limit satisfaction (after newly set speed value) to find bugs

    @property
    def curr_acc(self):  # acceleration in miles/(second)^2, with (+/-)
        '''
            The current acceleration value of the train, assumed consistant acceleration for its whole length.
            The acceleration value is fully determined by its current status with different conditions:
            ----------
            1: stopped trains has to maintain 0 acceleration, otherwise it is not stopped.
            2: the train is running, not stopped.
                2.1: if the train is running at its speed limit.
                    2.1.1: if the train's current target speed is higher than its current speed limit, 
                        the train shall neither accelerate nor decelerate.
                    2.1.2: if the train's current target speed is not higher than its current speed limit,
                        at every refreshing cycle, the train needs to determine if it needs to brake or not.
                        2.1.2.1: if it is ok to maintain current speed for the next refreshing cycle without
                            any violations, maintain its speed.
                        2.1.2.2: if it is not ok to maintain current speed for the next refreshing cycle period,
                            (some violations may happen if speed maintained), start to brake at its maximum effort
                2.2: if the train is running below its current speed limit.
                    2.2.1: if the train is not crossing its current signal if it maintains its current acceleration:
                        either accelerating or decelerating.
                        2.2.1.1: if the train's target speed >= its current speed limit > its current speed,
                            accelerate at its maximum acceleration effort.
                        2.2.1.2: if the train's current speed < its target speed < its current speed limit,
                            accelerate at its maximum acceleration effort.
                        2.2.1.3: if both the train's target speed and its current speed are less than (<) speed limit
                            the train has to determine whether to accelerate of decelerate at every refreshing cycle:
                            2.2.1.3.1: accelerate at maximum effort if it is ok to do so in the next refreshing cycle.
                            2.2.1.3.2: if it is not ok to accelerate at maximum effort for the next refreshing cycle,
                                maintain its current speed if it is okay to not to brake for the next refreshing cycle.
                            2.2.1.3.3: brake at maximum effort if cannot maintain speed for the next refreshing cycle.
                    2.2.2: if the train will cross its current signal if it maintains current acceleration (acc/dcc):
                        2.2.2.1 accelerate at maximum effort to cross the signal if target speed > current speed limit.
                        2.2.2.2 if target speed > current speed and the target speed < current speed limit,
                            accelerate at maximum effort to cross the signal.
                        2.2.2.3 if target speed <= current speed and the target speed < current speed limit,
                            decelerate at maximum effort to cross the signal.
                        '''
        _direction_sign = self.sign_MP(self.curr_routing_path_segment)
        # sign of traveling direction (MP increment)
        _delta_s = self.curr_speed * self.system.refresh_time + \
            0.5 * self._curr_acc * self.system.refresh_time ** 2
        # *intended* MP increment with current acceleration value after the next refreshing cycle
        _tgt_MP = self.curr_sig.MP if self.curr_sig else _direction_sign * \
            float('inf')
        # MilePost of its current signal. If no current signal, set the milepost as infinite.
        if self.stopped:  # 1
            self._curr_acc = 0
        else:  # 2
            if abs(self.curr_speed) == self.curr_spd_lmt_abs:  # 2.1
                if self.curr_target_spd_abs >= self.curr_spd_lmt_abs:  # 2.1.1
                    self._curr_acc = 0
                else:  # 2.1.2
                    if self.hold_speed_before_dcc(self.curr_MP,
                                                  self.curr_sig.MP,
                                                  self.curr_speed,
                                                  self.curr_target_spd_abs):
                        self._curr_acc = 0  # 2.1.2.1
                    else:
                        self._curr_acc = self.max_dcc * \
                            (-1) * _direction_sign  # 2.1.2.2
            elif abs(self.curr_speed) < self.curr_spd_lmt_abs:  # 2.2
                if (_tgt_MP - self.curr_MP) * (
                        _tgt_MP - (self.curr_MP + _delta_s)) >= 0:  # 2.2.1
                    if self.curr_target_spd_abs >= self.curr_spd_lmt_abs:  # 2.2.1.1
                        self._curr_acc = self.max_acc * _direction_sign
                    # 2.2.1.2
                    elif self.curr_target_spd_abs > abs(self.curr_speed):
                        self._curr_acc = self.max_acc * _direction_sign
                    # 2.2.1.3
                    elif self.curr_target_spd_abs <= abs(self.curr_speed):
                        if self.acc_before_dcc(self.curr_MP, self.curr_sig.MP,
                                               self.curr_speed,
                                               self.curr_target_spd_abs):
                            self._curr_acc = self.max_acc * _direction_sign  # 2.2.1.3.1
                        elif self.hold_speed_before_dcc(
                                self.curr_MP, self.curr_sig.MP, self.curr_speed,
                                self.curr_target_spd_abs):
                            self._curr_acc = 0  # 2.2.1.3.2
                        else:  # 2.2.1.3.3
                            self._curr_acc = self.max_dcc * \
                                (-1) * _direction_sign
                else:  # 2.2.2
                    if self.curr_target_spd_abs >= self.curr_spd_lmt_abs:  # 2.2.2.1
                        self._curr_acc = self.max_acc * _direction_sign
                    # 2.2.2.2
                    elif self.curr_target_spd_abs > abs(self.curr_speed):
                        # Note 1: speed setter guarantees not to violate speed limit albeit the acceleration value
                        # Note 2: current speed limit is the limit before crossing the signal. The speed limit will
                        # be updated as the train acquires the new signal aspect by crossing over it.
                        self._curr_acc = self.max_acc * _direction_sign
                    # 2.2.2.3
                    elif self.curr_target_spd_abs <= abs(self.curr_speed):
                        self._curr_acc = self.max_dcc * (-1) * _direction_sign
        return self._curr_acc

    @property
    def curr_target_spd_abs(self):  # in miles/sec, + only
        '''
            The current target speed of the train in aboslute value. 
            Corresponding to its current siganling speed and track allowable speeds.'''
        _curr_sig_trgt_speed_abs = float('inf')
        _curr_track_allow_sp_abs = getattr(self.curr_track, 'allow_sp',
                                           float('inf'))
        _curr_sig_permit_track_allow_sp = float('inf')
        # if exit the system, target speed is infinite (not defined).
        if self.curr_sig:  # waiting to initiate
            _curr_sig_trgt_speed_abs = self.curr_sig.aspect.target_speed
            if self.curr_sig.permit_track:  # initiating
                _curr_sig_permit_track_allow_sp = self.curr_sig.permit_track.allow_sp
        _tgt_spd = min(_curr_sig_trgt_speed_abs, _curr_track_allow_sp_abs,
                       _curr_sig_permit_track_allow_sp)
        # both current track allowable speed and permit track allowable speed are considered.
        assert _tgt_spd >= 0.0
        return _tgt_spd

    @property
    def curr_spd_lmt_abs(self):  # in miles/sec, + only
        '''
            The current speed limit in absolute value of the train.'''
        return self._curr_spd_lmt_abs

    @curr_spd_lmt_abs.setter
    def curr_spd_lmt_abs(self, spd_lmt):
        '''
            Setter for the current speed limit. Recommended to be called only 
            by the other methods/functions, not manually.'''
        if self.curr_track:
            self._curr_spd_lmt_abs = self.curr_track.allow_sp \
                if spd_lmt > self.curr_track.allow_sp \
                else spd_lmt
        else:
            self._curr_spd_lmt_abs = spd_lmt

    @property
    def curr_brake_distance_abs(self):  # in miles, + only
        '''
            The current braking distance in absolute value of the train if 
            engaging maximum brake effort.'''
        return self.abs_brake_distance(self.curr_speed,
                                       self.curr_target_spd_abs, self.max_dcc)

    @property
    def curr_dis_to_curr_sig_abs(self):  # in miles, + only
        '''
            The current distance to its curr_sig instance in absolute value 
            from the train head.'''
        return abs(self.curr_sig.MP -
                   self.curr_MP) if self.curr_sig else float('inf')

    @property
    def trains_ahead_same_dir(self):
        '''
            A list of other trains ahead of the train with the same direction. 
            The lower the list index, the closer with the train.'''
        return self.system.get_trains_between_points(
            self.curr_sigpoint, self.dest_pointport[0], obv=True)

    @property
    def trains_behind_same_dir(self):
        '''
            A list of other trains behind the train with the same direction. 
            The lower the list index, the closer with the train.'''
        return self.system.get_trains_between_points(
            self.rear_curr_prev_sigpoint,
            self.init_pointport[0],
            rev=True)

    @property
    def trains_ahead_oppo_dir(self):
        '''
            A list of other trains ahead of the train with the opposite direction. 
            The lower the list index, the closer with the train.'''
        return self.system.get_trains_between_points(
            self.curr_sigpoint, self.dest_pointport[0], rev=True)

    @property
    def trains_behind_oppo_dir(self):
        '''
            A list of other trains behind the train with the opposite direction. 
            The lower the list index, the closer with the train.'''
        return self.system.get_trains_between_points(
            self.rear_curr_prev_sigpoint,
            self.init_pointport[0],
            obv=True)

    @property
    def trn_follow_behind(self):
        return self.same_way_trains[self.rank + 1]

    @property
    def dist_to_trn_behind(self): 
        return abs(self.rear_curr_MP - self.trn_follow_behind.curr_MP)

    @property
    def pending_route(self):
        '''
            Status property shows if the train is pending a route at its current
            CtrlPoint for further proceeding.
            @return: True of False'''
        if not self.curr_sigpoint:
            return False
        elif self.curr_sigpoint == self.curr_ctrl_point:
            if not self.curr_sig.route:
                return True
        elif not self.system.get_trains_between_points(
                self.curr_sigpoint, self.curr_ctrl_point, obv=True):
            if not self.curr_ctrl_point.signal_by_port[
                    self.curr_ctrl_pointport].route:
                return True
        return False

    @property
    def any_paths_ahead_enterable(self):
        for p in self.all_paths_ahead:
            if all([True 
                    if self.system.capacity_enterable(self.curr_ctrl_point, n) 
                    else False for n in p]):
                return True
        return False

    def cross_sigpoint(self, sigpoint, curr_MP, new_MP):
        '''
            Method to update attributes and properties when the train's head
            crosses an interlocking signal point.  
            @return: None'''
        # TODO: implement geographical spans within interlocking points
        assert self.curr_sig.route in sigpoint.current_routes
        assert self.curr_sig in [
            sig for p, sig in sigpoint.signal_by_port.items()
        ]
        assert min(curr_MP, new_MP) <= self.curr_sig.MP <= max(curr_MP, new_MP)
        assert not self.stopped

        _route = getattr(self.curr_sig, 'route')
        _permit_track = getattr(self.curr_sig, 'permit_track')
        _next_enroute_sigpoint = getattr(self.curr_sig, 'next_enroute_sigpoint')
        _next_enroute_sigpoint_port = getattr(self.curr_sig,
                                              'next_enroute_sigpoint_port')
        terminate = False if _next_enroute_sigpoint else True
        initiate = False if self.curr_prev_sigpoint else True

        # if self.curr_speed != 0:
        #     timestamp = self.system.sys_time + abs(curr_MP - self.curr_sig.MP)/abs(self.curr_speed)
        # else:
        #     timestamp = self.system.sys_time
        # # record the time when the train crosses an interlocking point for God's sake
        # self.time_pos_list.append([timestamp, self.curr_sig.MP])

        if initiate:
            assert isinstance(sigpoint, CtrlPoint)
            assert len(self.curr_sig.permit_track.train) == 0
            print('train {0} initiated, entering into {1}'
                    .format(self, _permit_track))
            self.curr_spd_lmt_abs = self.curr_target_spd_abs  # update current speed limit
            self.curr_sig.permit_track.train.append(
                self)  # occupy the track to enter
            # occupy the route of interlocking point
            sigpoint.curr_train_with_route[self] = _route
            # close the route to protect interlocking
            sigpoint.close_route(_route)
            self.curr_routing_path_segment = \
                ((sigpoint, _route[1]), (_next_enroute_sigpoint,
                                         _next_enroute_sigpoint_port))
            self.curr_occupying_routing_path.insert(
                0, self.curr_routing_path_segment)
            # record the time of entering the system, serving train generation's
            # time separation
            self.init_time = self.system.sys_time

        elif not initiate and not terminate:
            assert len(self.curr_sig.permit_track.train) == 0
            self.curr_spd_lmt_abs = self.curr_target_spd_abs  # update current speed limit
            self.curr_sig.permit_track.train.append(
                self)  # occupy the track to enter
            # occupy the route of interlocking point
            sigpoint.curr_train_with_route[self] = _route
            # only close the route of CtrlPoint along the way
            if isinstance(sigpoint, CtrlPoint):
                # AutoPoints have no method to close route
                sigpoint.close_route(_route)
            self.curr_routing_path_segment = \
                ((sigpoint, _route[1]), (_next_enroute_sigpoint,
                                         _next_enroute_sigpoint_port))
            self.curr_occupying_routing_path.insert(
                0, self.curr_routing_path_segment)

        elif terminate:
            # no track to occupy because of terminating
            assert isinstance(sigpoint, CtrlPoint)
            self.curr_spd_lmt_abs = self.curr_target_spd_abs  # update current speed limit
            # occupy the route of interlocking point
            sigpoint.curr_train_with_route[self] = _route
            # close the route to protect interlocking
            sigpoint.close_route(_route)
            self.curr_routing_path_segment = ((sigpoint, _route[1]), (None,
                                                                      None))
            self.curr_occupying_routing_path.insert(
                0, self.curr_routing_path_segment)

        else:
            raise Exception('{} crossing {} failed unexpectedly'
                            .format(self, sigpoint))

    def rear_cross_sigpoint(self, sigpoint, rear_curr_MP, new_rear_MP):
        '''
            Method to update attributes and properties when the train's rear end
            crosses an interlocking signal point.  
            @return: None
        '''
        # TODO: implement geographical spans within interlocking points
        assert self in sigpoint.curr_train_with_route.keys()
        assert self.rear_curr_sig in [
            sig for p, sig in sigpoint.signal_by_port.items()
        ]
        assert min(rear_curr_MP, new_rear_MP) <= self.rear_curr_sig.MP <= max(
            rear_curr_MP, new_rear_MP)
        assert not self.stopped

        # if self.curr_speed != 0:
        #     timestamp = self.system.sys_time + abs(rear_curr_MP - self.rear_curr_sig.MP)/abs(self.curr_speed)
        # else:
        #     timestamp = self.system.sys_time
        # self.rear_time_pos_list.append([timestamp, self.rear_curr_sig.MP])

        del sigpoint.curr_train_with_route[self]
        if self.rear_curr_track:
            self.rear_curr_track.train.remove(self)
        self.curr_occupying_routing_path.pop(-1)
        # TODO:----dispatching logic may need to modify here
        # ---------to determine if further bigblock actions are needed

    def hold_speed_before_dcc(self, MP, tgt_MP, spd, tgt_spd):
        '''
            Method to determine if the train can hold its current speed 
            for another refreshing cycle at its concurrent properties: 
                MP, target speed, current speed, and target speed
            @return: True or False'''
        delta_s = spd * self.system.refresh_time
        # False, when hold the speed and the train will
        # cross its current facing signal in the upcoming cycle.
        if (tgt_MP - MP) * (tgt_MP - (MP + delta_s)) < 0:
            return False
        # After holding speed within the cycle if the brake distance still hold:
        # return True
        # brake distance is by its maximum deceleration value.
        elif abs(tgt_MP - (MP + delta_s)) > \
                self.abs_brake_distance(spd, tgt_spd, self.max_dcc) and abs(tgt_MP - MP) > abs(delta_s):
            return True
        return False

    def acc_before_dcc(self, MP, tgt_MP, spd, tgt_spd):
        '''
            Method to determine if the train can accelerate at maximum acceleration
            for another refreshing cycle at its concurrent properties: 
                MP, target speed, current speed, and target speed
            @return: True or False'''
        assert self.curr_sig
        # prerequisite: the train can hold its current speed for another refreshing cycle.
        if not self.hold_speed_before_dcc(MP, tgt_MP, spd, tgt_spd):
            return False
        else:
            _direction_sign = self.sign_MP(self.curr_routing_path_segment)
            delta_s = spd * self.system.refresh_time + 0.5 * \
                (_direction_sign*self.max_acc) * self.system.refresh_time**2
            delta_spd = (_direction_sign * self.max_acc) * \
                self.system.refresh_time
            # False, if the train accelerates and its will
            # cross its current facing signal in the upcoming cycle.
            if (tgt_MP - MP) * (tgt_MP - (MP + delta_s)) < 0:
                return False
            # True, after acceleration within the cycle if the brake distance still satisfies.
            # brake distance is by its maximum deceleration value.
            elif abs(tgt_MP - (MP + delta_s)) > self.abs_brake_distance(spd + delta_spd, tgt_spd, self.max_dcc) \
                    and abs(tgt_MP - MP) > abs(delta_s):
                return True
        return False

    def update_acc(self):
        '''
            Method to be called by a simulator/system, updating the train's MP, 
            speed & acceleration properties at each refreshing cycle. 
            Update properties under different conditions and status.
            @return: None'''
        if not self.stopped:
            self.curr_speed = self.curr_speed + self.curr_acc * self.system.refresh_time
            delta_s = self.curr_speed * self.system.refresh_time + \
                0.5 * self.curr_acc * self.system.refresh_time ** 2
            self.curr_MP += delta_s
            self.pos_spd_list.append([
                self.curr_MP, self._curr_speed, self.curr_spd_lmt_abs,
                self.curr_target_spd_abs
            ])
        elif not self.terminated:
            self.time_pos_list.append([self.system.sys_time+self.system.refresh_time, self.curr_MP])
            self.rear_time_pos_list.append(
                [self.system.sys_time+self.system.refresh_time, self.rear_curr_MP])
        else:
            pass

    def request_routing(self):
        '''
            Method of the train to call the closest CtrlPoint to clear a route. 
            Serve the myopic dispatch logic where trains only calls the cloest CPs.
            @return: None'''
        if self.pending_route and self.any_paths_ahead_enterable:
            _pending_route_to_open = \
                self.curr_ctrl_point.find_route_for_port(
                                            port=self.curr_ctrl_pointport,
                                            dest_pointport=self.dest_pointport)
            if _pending_route_to_open is None:
                return
            if _pending_route_to_open not in self.curr_ctrl_point.current_invalid_routes:
                if not self.curr_track or not self.curr_track.yard:
                    print('{}, requested {} at {}'
                    .format(self, _pending_route_to_open, 
                            self.curr_ctrl_point.MP))
                    self.curr_ctrl_point.open_route(_pending_route_to_open)
                elif self.curr_track.yard:
                    if not self.currently_passable(max_passes=1):
                        print('{}, requested {} at {}'
                        .format(self, _pending_route_to_open,
                                self.curr_ctrl_point.MP))
                        self.curr_ctrl_point.open_route(_pending_route_to_open)
                    elif self.currently_passable(max_passes=1):
                        # no actions of its current CP (no route to open)
                        for cp in self.system.ctrl_points:
                            for (p1,p2) in cp.current_routes:
                                if cp.bigblock_by_port.get(p2):
                                    if self in cp.bigblock_by_port[p2].train:
                                        _route_to_change = \
                                            cp.find_route_for_port(
                                                port=p1, 
                                                dest_pointport=self.dest_pointport)
                                        cp.open_route(_route_to_change)

    def currently_passable(self, max_passes=1):
        '''
            determine if the train is slower than the one behind and needs to be
            put on the siding to let the follower pass it.
            Rank minus initial index is the number of trains have passed it.
            @return: True or False
            TODO: implement better judgment to consider more conditions, such as
                priority, proximity (to the follower), etc.'''
        # the last train is not passable by any train
        if self.rank == len(self.same_way_trains) - 1:
            return False
        # for any train that is not the last one:
        if not self.curr_track or not self.rear_curr_track:
            return False    # not passable when not fully entered yet
        if not (self.max_spd < self.trn_follow_behind.max_spd):
            return False    # not passable if not slower than the one behind
        if self.rank - self.train_idx >= max_passes:
            return False    # not passable if has already been passed by once
        if self.curr_track.yard:
            _all_trains = self.curr_track.yard.all_trains
            _rest_trains = [t for t in _all_trains if t != self]
            if self.stopped and any([not t.stopped for t in _rest_trains]):
                return True # during the pass, hold the stopped train from move
            if all([trk.train for trk in self.curr_track.yard.tracks]):
                if abs(self.max_spd) == min([abs(trn.max_spd) 
                        for trn in self.curr_track.yard.all_trains]):
                    return True
            if self.curr_track.yard.available_tracks >= 1 and self.dist_to_trn_behind <= 10:
                return True
        return False

    def is_during_dos(self, dos_pos):
        '''
            Whether the train is during the dos pos and time period
            @return: True or False'''
        return dos_pos == self.curr_track and self.system.dos_period[
            0] <= self.system.sys_time <= self.system.dos_period[1]

    def __lt__(self, other):
        '''
            implement __lt__ to sort trains based on their current MilePost.
            If MilePosts are the same, compare max_spd.
            TODO: implement better algorithms to compare train priority.'''
        if not self.terminated:
            if self.curr_MP > other.curr_MP:
                return True if self.downtrain else False
            elif self.curr_MP < other.curr_MP:
                return False if self.downtrain else True
            # when MP is the same, compare max speed as a priority indicator
            elif self.curr_MP == other.curr_MP:
                if self.max_spd > other.max_spd:
                    return True if self.downtrain else False
                elif self.max_spd < other.max_spd:
                    return False if self.downtrain else True
        if self.terminated and other.terminated:
            _self_term_time = max([time for [time,_] in self.time_pos_list])
            _other_term_time = max([time for [time,_] in other.time_pos_list])
            if _self_term_time < _other_term_time:
                return True
        if self.terminated and not other.terminated:
            return True
