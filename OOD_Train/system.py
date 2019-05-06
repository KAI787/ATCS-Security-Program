from datetime import datetime, timedelta
from block import Block
import numpy as np
from train import Train


exp_buffer, var_buffer = 10, 0.5

class System():
    def __init__(self, init_time, blk_length_list, tracks=[], dos_period=['2017-01-01 02:00:00', '2017-01-01 02:30:00'], dos_pos=-1, refresh_time=1):
        self.sys_time = init_time.timestamp()  
        # CPU format time in seconds, transferable between numerical value and M/D/Y-H/M/S string values 
        self.blocks = []
        self.multi_tracks_blk = []
        self.length = 0
        for i in range(len(blk_length_list)):
            self.blocks.append(Block(i, blk_length_list[i], tracks[i]))
            if(tracks[i] > 1):
                self.multi_tracks_blk.append(i)
            self.length += blk_length_list[i]
        self.up_trains = []
        self.down_trains = []
        self.dos_period = [datetime.strptime(t, "%Y-%m-%d %H:%M:%S").timestamp() for t in dos_period if type(t) == str]
        self.dos_pos = dos_pos
        self.block_intervals = []

        # interval is the two-element array containing mile posts of boundaries 
        for i in range(len(blk_length_list)):
            if i == 0:
                self.block_intervals.append([0, blk_length_list[0]])
            else:
                left = self.block_intervals[i - 1][1]
                right = left + blk_length_list[i]
                self.block_intervals.append([left, right]) 
        self.last_up_train_init_time = self.sys_time
        self.last_down_train_init_time = self.sys_time
        self.refresh_time = refresh_time

    def generate_new_train(self, track_idx, up_or_down):
        if up_or_down == "UP":
            new_train = Train(len(self.up_trains), len(self.up_trains), self.sys_time, track_idx, self, True)
            self.up_trains.append(new_train)
            self.last_up_train_init_time = self.sys_time
            new_train.enter_block(0, track_idx)
        elif up_or_down == "DOWN":
            new_train = Train(len(self.down_trains), len(self.down_trains), self.sys_time, track_idx, self, False)
            self.down_trains.append(new_train)
            self.last_down_train_init_time = self.sys_time
            new_train.enter_block(-1, track_idx)

    def update_block_trgt_speed(self):
        # update the trgt_speed of every block.
        for i in range(len(self.blocks) - 2,-1,-1):
            if i <= len(self.blocks) - 2 and not self.blocks[i + 1].has_available_track():
                self.blocks[i].set_stop_speed()

            if i <= len(self.blocks) - 3 \
                and self.blocks[i + 1].has_available_track()\
                and not self.blocks[i + 2].has_available_track():
                self.blocks[i].set_approaching_speed()
            
            if i <= len(self.blocks) - 4 \
                and self.blocks[i + 1].has_available_track()\
                and self.blocks[i + 2].has_available_track()\
                and not self.blocks[i + 3].has_available_track():
                self.blocks[i].set_middle_approaching_speed()

            if i <= len(self.blocks) - 5 \
                and self.blocks[i + 1].has_available_track()\
                and self.blocks[i + 2].has_available_track()\
                and self.blocks[i + 3].has_available_track()\
                and not self.blocks[i + 4].has_available_track():
                self.blocks[i].set_clear_speed()
            
    def generate_train(self, headway):
        if len(self.up_trains) == 0:
            track_idx = self.blocks[0].find_available_track()
            self.generate_new_train(track_idx, "UP")

        if len(self.down_trains) == 0:
            track_idx = self.blocks[-1].find_available_track()
            self.generate_new_train(track_idx, "DOWN")

        if self.sys_time - self.last_up_train_init_time >= headway and self.blocks[0].has_available_track():
            track_idx = self.blocks[0].find_available_track()
            self.generate_new_train(track_idx, "UP")

        if self.sys_time - self.last_down_train_init_time >= headway and self.blocks[len(self.blocks) - 1].has_available_track():
            track_idx = self.blocks[-1].find_available_track()
            self.generate_new_train(track_idx, "DOWN")

    def refresh(self):
        headway = 300#np.random.normal(exp_buffer, var_buffer)
        # If the time slot between now and the time of last train generation
        # is bigger than headway, it will generate a new train at start point.
        self.generate_train(headway)

        # TODO: 火车更新顺序问题待讨论解决。
        up_size = len(self.up_trains)
        down_size = len(self.down_trains)
        i = 0
        j = 0
        while i < up_size or j < down_size:
            if i < up_size:
                self.up_trains[i].update_up(self.dos_pos)
                i += 1
            if j < down_size:
                self.down_trains[j].update_down(self.dos_pos)
                j += 1

        self.up_trains.sort()
        self.down_trains.sort()
        
        for i, tr in enumerate(self.up_trains):
            tr.rank = i

        for i, tr in enumerate(self.down_trains):
            tr.rank = i

        self.sys_time += self.refresh_time

    def exist_oppo_train_between_blk(self, start_blk, end_blk):
        if start_blk > end_blk:
            start_blk, end_blk = end_blk, start_blk

        for blk_idx in range(start_blk + 1, end_blk):
            if self.blocks[blk_idx].exist_train():
                return True
        return False
    
    def next_multi_tracks_blk(self, curr_blk, direction):
        idx = self.multi_tracks_blk.index(curr_blk)
        if idx == 0 or idx == len(self.multi_tracks_blk) - 1:
            return -1
        if direction == "UP":
            return self.multi_tracks_blk[idx + 1]
        else:
            return self.multi_tracks_blk[idx - 1]