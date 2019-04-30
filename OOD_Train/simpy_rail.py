import simpy

TRAIN_SPEED_CONTAINER = [0.02, 0.02, 0.03, 0.06, 0.04, 0.02, 0.02, 0.03, 0.02, 0.07, 0.08, 0.07, 0.06, 0.07, 0.06, 0.03, 0.07, 0.02, 0.07, 0.03, 0.06, 0.05, 0.03, 0.1, 0.06, 0.05, 0.09, 0.05, 0.03, 0.1, 0.07, 0.1, 0.03, 0.03, 0.03, 0.1, 0.03, 0.06, 0.1, 0.03, 0.04, 0.04, 0.05, 0.07, 0.07, 0.1, 0.04, 0.04, 0.09, 0.02] 
TRAIN_ACC_CONTAINER = [0.001] * 50
TRAIN_INIT_TIME = []

class Train(object):
    def __init__(self, env, system, idx, rank, init_time, init_pos, init_speed, max_speed, curr_track):
        self.env = env
        self.sys = system
        self.idx = idx
        self.curr_pos = init_pos
        self.curr_blk = 0
        self.max_spd = max_speed
        self.curr_spd = init_speed
        self.max_acc = 0.001
        self.curr_acc = self.acc
        self.status = 1
        self.train_idx = idx
        self.rank = rank
        self.blk_time = [[init_time]]
        self.time_pos_list = [[self.blk_time[0][0], sys.blk_interval[0][0]]]  # not yet implemented interpolation
        self.curr_track = curr_track
        
        
    def check_block_access(self,block):
        with block.track.request() as request:
            yield request       # wait for access, once accessed, continue to the line below.
            # gained the access to a track in the block
            print('%s gained access to Block %s at %.2f.' % (self.name, block.get_name(), self.env.now))
            yield self.env.timeout(block.get_time(self))
            print('%s leaves Block %s at %.2f.' % (self.name, block.get_name(), self.env.now))
    
class Block(object):
    def __init__(self, env, name, length):
        self.env = env
        self.name = name
        self.track = simpy.Resource(env)
        self.length = length
        
    def get_name(self):
        return self.name
    
    def get_time(self, train):
        return self.length / train.speed
        
        
print('Railroad Simulation \n')

env = simpy.Environment()
block_0 = Block(env, "block 0", 5)
block_1 = Block(env, "block 1", 5)
block_2 = Block(env, "block 2", 10)
block_3 = Block(env, "block 3", 5)
block_4 = Block(env, "block 4", 5)
train_west = Train(env, idx)

def train_west_path(env):
    while True:
        yield env.process(train_west.check_block_access(block_0))
        yield env.process(train_west.check_block_access(block_1))
        yield env.process(train_west.check_block_access(block_2))
        yield env.process(train_west.check_block_access(block_3))
        yield env.process(train_west.check_block_access(block_4))

env.process(train_west_path(env))
env.run(until=100)