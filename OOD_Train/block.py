class Block():

    def __init__(self, index, length, track_number=1):
        self.index = index
        self.length = length
        # There is track_number tracks in this block.
        self.track_number = track_number
        self.tracks = [None] * self.track_number
        self.track_isOccupied = [False] * self.track_number    # needs to be re-written into a list-like format
        self.topo = [[] for i in range(self.track_number+1)]
        self.max_speed = 0.09
        self.trgt_speed = self.max_speed
        self.sys_pos = [0, self.length]

    def has_available_track(self):
        for oc in self.track_isOccupied:
            if not oc:
                return True
        return False

    def find_available_track(self):
        assert self.has_available_track()
        for idx, oc in enumerate(self.track_isOccupied):
            if not oc:
                return idx

    def occupied_track(self, idx, train):
        assert not self.track_isOccupied[idx]
        self.track_isOccupied[idx] = True
        self.tracks[idx] = train
    
    def free_track(self, idx):
        assert self.track_isOccupied[idx]
        self.track_isOccupied[idx] = False
        self.tracks[idx] = None

    def set_clear_speed(self):
        self.trgt_speed = self.max_speed

    def set_middle_approaching_speed(self):
        self.trgt_speed = self.max_speed * 3 / 4

    def set_approaching_speed(self):
        self.trgt_speed = self.max_speed / 2

    def set_stop_speed(self):
        self.trgt_speed = 0
    
    def exist_train(self):
        for tck in self.tracks:
            if tck != None:
                return True
        return False