import numpy as np


class Box:
    def __init__(self, width, height, depth):
        self.width = width
        self.height = height
        self.depth = depth
        self.volume = self.width * self.height * self.depth

        self.position = Point3D(-1, -1, -1)

    def get_width(self):
        return self.width

    def get_height(self):
        return self.height

    def get_depth(self):
        return self.depth

    def set_pos(self, x, y, z):
        self.position = Point3D(x, y, z)

    def get_pos(self):
        return self.position

    def get_pos_x(self):
        return self.position[0]

    def get_pos_y(self):
        return self.position[1]

    def get_pos_z(self):
        return self.position[2]

    def get_end_x(self):
        return self.position.get_x() + self.width

    def get_end_y(self):
        return self.position.get_y() + self.height

    def get_end_z(self):
        return self.position.get_z() + self.depth


class Point2D:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def get_x(self):
        return self.x

    def get_y(self):
        return self.y

    def set_x(self, x):
        self.x = x

    def set_y(self, y):
        self.y = y

    def get_point(self):
        return self


class Point3D:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

    def get_x(self):
        return self.x

    def get_y(self):
        return self.y

    def get_z(self):
        return self.z

    def set_x(self, x):
        self.x = x

    def set_y(self, y):
        self.y = y

    def set_z(self, z):
        self.z = z

    def get_point(self):
        return self

class Bin:
    def __init__(self, width, height, depth):
        self.width = width
        self.height = height
        self.depth = depth
        self.volume = self.width * self.height * self.depth

    def get_volume(self):
        return self.volume

    def get_width(self):
        return self.width

    def get_height(self):
        return self.height

    def get_depth(self):
        return self.depth


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

    def order_box_set(self, box_set: [Box]):
        box_set = sorted(box_set, key=lambda box: box.get_end_x(), reverse=True)  # check if it works properly
        box_set = sorted(box_set, key=lambda box: box.get_end_y(), reverse=True) #check if it works properly

        return box_set


    def two_dimensional_corners(self, box_set: [Box], J: [Box], bin: Bin):
        if box_set is None or len(box_set) == 0:
            return Point2D(0, 0)

        # must identify the extreme boxes
        border = 0
        extreme_boxes = []
        for box in box_set:
            if box.get_end_x() > border:
                extreme_boxes.append(box)
                border = box.get_end_x()
        # end of identifying extreme boxes

        # determine the corner points
        corners = [Point2D(0, extreme_boxes[0].get_end_y())]
        for index in range(1, len(extreme_boxes)):
            corners.append(Point2D(corners[index-1].get_end_x(), extreme_boxes[index].get_end_y()))
        corners.append(Point2D(extreme_boxes[-1].get_end_x(), 0))

        # remove all infeasible corners
        # a corner is infeasible if in one of the two directions there is no possible box that fits there
        minimum_w = J[0].get_width()
        minimum_h = J[0].get_height()
        for box in J:
            if box.get_height() > minimum_h:
                minimum_h = box.get_height()
            if box.get_width() > minimum_w:
                minimum_w = box.get_width()

        final_corners = []
        for corner in corners:
            if (corner.get_x() + minimum_w) <= bin.get_width() and (corner.get_y() + minimum_h) <= bin.get_height():
                final_corners.append(corner)

        return final_corners


    def three_dimensional_corners(self, box_set: [Box], J: [Box], bin: Bin):
        if box_set is None or len(box_set) == 0:
            return Point3D(0, 0, 0)

        # must order the I set for future uses (in the for loop above)
        box_set = self.order_box_set(box_set)

        T = [0]
        for box in box_set:
            T.append(box.get_end_z())

        T = sorted(T, reverse=False) # devo gestire casi di profondità doppie? non penso pero...

        # find the minimum depth in the J boxes list
        minimum_d = J[0].get_depth()
        for box in J:
            if box.get_depth() > minimum_d:
                minimum_d = box.get_depth()

        total_corners = []
        incremental_corners = []
        k = 0
        for depth in T:
            if depth + minimum_d > bin.get_depth():
                break

            I_k = [box for box in box_set if box.get_end_z() > depth]
            incremental_corners.append(self.two_dimensional_corners(I_k, J, bin))

            for point in incremental_corners[-1]:
                found = False
                for already_seen_point in incremental_corners[-2]:
                    if point.get_x() == already_seen_point.get_x() and point.get_y() == already_seen_point.get_y():
                        found = True
                        break
                if not found:
                    total_corners.add(Point3D(point.get_x(), point.get_y(), depth))

            k = k + 1

        return total_corners