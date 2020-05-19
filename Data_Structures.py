import numpy as np


class Box:
    def __init__(self, width, height, depth):
        self.width = width
        self.height = height
        self.depth = depth
        self.volume = self.width * self.height * self.depth
        self.x = -1
        self.y = -1
        self.z = -1

    def set_pos(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

    def get_pos(self):
        return self.x, self.y, self.z

    def get_end_x(self):
        return self.x + self.width

    def get_end_y(self):
        return self.y + self.height

    def get_end_z(self):
        return self.z + self.depth


class Bin:
    def __init__(self, width, height, depth):
        self.width = width
        self.height = height
        self.depth = depth
        self.volume = self.width * self.height * self.depth

    def get_volume(self):
        return self.volume

class PalletizationModel:
    def __init__(self, bin, boxList):
        self.boxList = boxList
        self.bin = bin

    def get_l1_bound(self):
        pass

    def get_l1_w_h(self):
        W = self.bin.width
        H = self.bin.height
        D = self.bin.depth
        j_w_h = [box for box in self.boxList if (box.width >= W/2) and (box.height >= H/2)]
        j_w_h_d = len([box for box in j_w_h if (box.depth > D/2)])
        max_val = 0
        for p in range(1, int(np.ceil(D/2))+1):
            Jl = [box for box in j_w_h if D-p >= box.depth > D/2]
            Js = [box for box in j_w_h if D/2 >= box.depth >= p]
            first_parameter = np.ceil((1/D)*(np.sum([box.depth for box in Js]) - (len(Jl)*D - np.sum([box.depth for box in Jl]))))
            second_parameter = np.ceil((len(Js) - (len(Jl) * D - np.sum([np.floor(0.5*(D - box.depth)) for box in Jl]))) / np.floor(D / p))
            max_tmp = max(first_parameter, second_parameter)
            if max_val < max_tmp:
                max_val = max_tmp
        return j_w_h_d + max_val
