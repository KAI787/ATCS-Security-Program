import random
import numpy as np

TRAIN_SPEED_CONTAINER = [0.02, 0.02, 0.03, 0.06, 0.04, 0.02, 0.02, 0.03, 0.02, 0.07, 0.08, 0.07, 0.06, 0.07, 0.06, 0.03, 0.07, 0.02, 0.07, 0.03, 0.06, 0.05, 0.03, 0.1, 0.06, 0.05, 0.09, 0.05, 0.03, 0.1, 0.07, 0.1, 0.03, 0.03, 0.03, 0.1, 0.03, 0.06, 0.1, 0.03, 0.04, 0.04, 0.05, 0.07, 0.07, 0.1, 0.04, 0.04, 0.09, 0.02] 
TRAIN_INIT_TIME = []

class Train():
    def __init__(self, idx, rank, init_time, curr_track, system):
        self.curr_pos = 0
        self.max_speed = random.randint(2,10) / 100
        self.curr_speed = self.max_speed
        self.acc = 0.001
        self.curr_acc = self.acc
        self.curr_blk = 0
        self.status = 1
        self.train_idx = idx
        self.rank = rank
        self.system = system
        self.blk_time = [[init_time]]
        self.time_pos_list = [[self.blk_time[0][0], self.system.blk_interval[0][0]]]  # not yet implemented interpolation
        self.curr_track = curr_track

    def __lt__(self, other):
        if self.curr_pos > other.curr_pos:
            return True
        elif self.curr_pos < other.curr_pos:
            return False
        elif self.max_speed < other.max_speed:
            return False
        else:
            return True

    def stop(self):
        self.curr_speed = 0
        self.status = 0
     
    def start(self):
        self.curr_speed = self.max_speed
        self.status = 1
    
    def terminate(self):
        self.status = 2
        
    def proceed(self, dest=None):
        self.start()
        if not dest:
            self.curr_pos += self.curr_speed * self.system.refresh_time
        else:
            self.curr_pos = dest
        self.time_pos_list.append([self.system.sys_time+self.system.refresh_time, self.curr_pos])
        
    def proceed_acc(self, delta_s, dest=None):
        if self.curr_speed + self.curr_acc * self.system.refresh_time > self.max_speed:
            self.curr_speed = self.max_speed
        else:
            self.curr_speed += self.curr_acc * self.system.refresh_time
        if not dest:
            self.curr_pos += delta_s
        else:
            self.curr_pos = dest
        self.time_pos_list.append([self.system.sys_time+self.system.refresh_time, self.curr_pos])
    
    def stop_at_block_end(self):
        # assert self.curr_pos + self.curr_speed * system.refresh_time >= self.blk_interval[self.curr_blk][1]
        if self.curr_speed > 0:
            interpolate_time = (self.system.blk_interval[self.curr_blk][1]-self.curr_pos)/self.curr_speed + self.system.sys_time
            self.curr_pos = self.system.blk_interval[self.curr_blk][1]
            self.time_pos_list.append([interpolate_time, self.curr_pos])
        if self.curr_speed == 0:
            self.curr_pos = self.curr_pos
        self.time_pos_list.append([self.system.sys_time+self.system.refresh_time, self.curr_pos])
        self.stop()
        
    def leave_block(self, blk_idx):
        self.system.blocks[blk_idx].free_track(self.curr_track)
        self.blk_time[blk_idx].append(self.system.sys_time)
        # interpolate the time moment when the train leaves the system
        if blk_idx == len(self.system.blocks)-1:
            interpolate_time = (self.system.blk_interval[blk_idx][1] - self.curr_pos) / self.curr_speed + self.system.sys_time
            self.curr_pos = self.system.blk_interval[blk_idx][1]
            self.time_pos_list.append([self.system.sys_time, self.curr_pos])
        
    def enter_block(self, blk_idx, next_block_ava_track):
        self.system.blocks[blk_idx].occupied_track(next_block_ava_track, self)
        self.curr_track = next_block_ava_track
        self.blk_time.append([self.system.sys_time])
    
    def update(self, dos_pos=-1):
        # update self.curr_pos
        # update self.curr_speed
        # if the train already at the end of the railway, do nothing. (no updates on (time,pos))
        if self.curr_pos == self.system.blk_interval[-1][1]:
            pass
        # If the train arrives at the end of all the blocks, the train will leave the system.
        elif self.curr_pos + self.curr_speed * self.system.refresh_time >= self.system.blk_interval[-1][1]:
            self.leave_block(len(self.system.blk_interval) - 1)
            self.curr_blk = None
            self.proceed(dest=self.system.blk_interval[-1][1])
        # The train will still stay in current block in next refresh time, so continue the system.
        elif self.curr_pos + self.curr_speed * self.system.refresh_time < self.system.blk_interval[self.curr_blk][1]:
            self.curr_blk = self.curr_blk
            self.proceed()
        # If the next block has no available tracks 
        # the train will stop at end of current block.
        elif (not self.system.blocks[self.curr_blk+1].has_available_track()): 
            self.stop_at_block_end()
        # If or there is a dos at the end of current block
        # the train will stop at end of current block.
        elif dos_pos == self.curr_blk and self.system.dos_period[0] <= self.system.sys_time <= self.system.dos_period[1]:
            self.stop_at_block_end()
        #If next train is faster than this train, the postion of previous train is behind the start
        # of this block, let this train stop at the end of block.
        elif self.curr_pos + self.max_speed * self.system.refresh_time >= self.system.blk_interval[self.curr_blk][1]\
            and self.rank < self.system.train_num - 1\
            and self.max_speed < self.system.trains[self.rank + 1].max_speed\
            and self.system.trains[self.rank + 1].curr_pos >=\
                self.system.block_intervals[self.system.trains[self.rank].curr_blk - 1][0]\
            and self.system.blocks[self.curr_blk].has_available_track():
                self.stop_at_block_end()
        # If the train will enter the next block in next refresh time,
        # update the system info and the train info.
        elif self.curr_pos + self.curr_speed * self.system.refresh_time >= self.system.blk_interval[self.curr_blk][1]: 
            self.leave_block(self.curr_blk)
            next_block_ava_track = self.system.blocks[self.curr_blk + 1].find_available_track()
            self.enter_block(self.curr_blk+1, next_block_ava_track)
            self.curr_blk += 1
            self.proceed()
   
    def select_move_model(self):
        # print("current block index: {}".format(self.curr_blk))
        if self.curr_blk == None:
            return 0
        curr_block = self.system.blocks[self.curr_blk]
        if self.curr_speed + self.curr_acc * self.system.refresh_time > self.max_speed:
            self.curr_acc = 0
            return self.max_speed * self.system.refresh_time
        break_distance = (self.curr_speed ** 2 - self.system.blocks[self.curr_blk].trgt_speed ** 2) / (2 * self.acc)
        
        # assert break_distance <= self.blk_interval[self.curr_blk][1] - self.curr_pos
        
        if self.curr_speed < curr_block.trgt_speed:
            self.curr_acc = self.acc
        elif self.curr_speed > curr_block.trgt_speed:
            if break_distance >= self.system.blk_interval[self.curr_blk][1] - self.curr_pos:
                self.curr_acc = - self.acc
            elif break_distance < self.system.blk_interval[self.curr_blk][1] - self.curr_pos:
                self.curr_acc = self.acc
        else:
            self.curr_acc = 0
        
        delta_s = self.curr_speed * self.system.refresh_time + 0.5 * self.curr_acc * system.refresh_time ** 2
        print(delta_s)
        return delta_s

    def select_move_model_simple(self):
        # print("current block index: {}".format(self.curr_blk))
        if self.curr_blk == None:
            return 0
        curr_block = self.system.blocks[self.curr_blk]
        
        if self.curr_speed < curr_block.trgt_speed:
            self.curr_acc = self.acc
        elif self.curr_speed > curr_block.trgt_speed:
            self.curr_acc = - self.acc
        else:
            self.curr_acc = 0
        
        delta_s = self.curr_speed * self.system.refresh_time + 0.5 * self.curr_acc * self.system.refresh_time ** 2
        print(delta_s)
        return delta_s

    def update_acc(self, dos_pos=-1):
        delta_s = self.select_move_model_simple()
        # update self.curr_pos
        # update self.curr_speed
        # if the train already at the end of the railway, do nothing. (no updates on (time,pos))
        if self.curr_pos == self.system.blk_interval[-1][1]:
            pass
        # If the train arrives at the end of all the blocks, the train will leave the system.
        elif self.curr_pos + delta_s >= self.system.blk_interval[-1][1]:
            self.leave_block(len(self.system.blk_interval) - 1)
            self.curr_blk = None
            self.proceed_acc(delta_s, dest=self.system.blk_interval[-1][1])
        # The train will still stay in current block in next refresh time, so continue the system.
        elif self.curr_pos + delta_s < self.system.blk_interval[self.system.curr_blk][1]:
            self.curr_blk = self.curr_blk
            self.proceed_acc(delta_s)
        # If the next block has no available tracks 
        # the train will stop at end of current block.
        elif (not self.system.blocks[self.curr_blk+1].has_available_track()): 
            self.stop_at_block_end()
        # If or there is a dos at the end of current block
        # the train will stop at end of current block.
        elif dos_pos == self.curr_blk and self.system.dos_period[0] <= self.system.sys_time <= self.system.dos_period[1]:
            self.stop_at_block_end()
        #If next train is faster than this train, the postion of previous train is behind the start
        # of this block, let this train stop at the end of block.
        elif self.curr_pos + self.max_speed * self.system.refresh_time >= self.system.blk_interval[self.curr_blk][1]\
            and self.rank < self.system.train_num - 1\
            and self.max_speed < self.system.trains[self.rank + 1].max_speed\
            and self.system.trains[self.rank + 1].curr_pos >=\
                self.system.block_intervals[self.system.trains[self.rank].curr_blk - 1][0]\
            and self.system.blocks[self.curr_blk].has_available_track():
                self.stop_at_block_end()
        # If the train will enter the next block in next refresh time,
        # update the system info and the train info.
        elif self.curr_pos + delta_s >= self.system.blk_interval[self.curr_blk][1]: 
            self.leave_block(self.curr_blk)
            next_block_ava_track = self.system.blocks[self.curr_blk + 1].find_available_track()
            self.enter_block(self.curr_blk+1, next_block_ava_track)
            self.curr_blk += 1
            self.proceed_acc(delta_s)

    def print_blk_time(self):
        print(self.blk_time)