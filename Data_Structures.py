import numpy as np

F_INITIAL_VALUE = -1

DEBUG = True


class Box:
    def __init__(self, width, height, depth):
        self.width = width
        self.height = height
        self.depth = depth
        self.volume = self.width * self.height * self.depth
        self.assigned = False
        self.position = NOT_PLACED_POINT

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

    def get_volume(self):
        return self.volume

    def is_placed(self):
        return self.position == NOT_PLACED_POINT


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

    def __eq__(self, other):
        return isinstance(other, Point2D) and other.x == self.x \
               and other.y == self.y


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

    def __eq__(self, other):
        return isinstance(other, Point3D) and other.x == self.x \
               and other.y == self.y \
               and other.z == self.z


NOT_PLACED_POINT = Point3D(-1, -1, -1)


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
        self.list_w_h = [[box.width, box.height, box.depth] for box in self.boxList]
        self.list_w_d = [[box.width, box.depth, box.height] for box in self.boxList]
        self.list_h_d = [[box.height, box.depth, box.width] for box in self.boxList]
        self.l1_w_h = None
        self.l1_w_d = None
        self.l1_h_d = None
        self.l1 = None
        self.l2 = None
        self.calculate_l1_bound()
        self.calculate_l2_bound()

    def get_bin(self):
        return self.bin

    def calculate_l1_bound(self):
        W = self.bin.width
        H = self.bin.height
        D = self.bin.depth
        self.l1_w_h = self.get_l1_p_max(self.list_w_h, W, H, D)
        self.l1_w_d = self.get_l1_p_max(self.list_w_d, W, D, H)
        self.l1_h_d = self.get_l1_p_max(self.list_h_d, H, D, W)
        self.l1 = max(self.l1_w_h, self.l1_w_d, self.l1_h_d)

    def get_l1_bound(self):
        return self.l1

    def get_l1_p_max(self, value_list, v1, v2, v3):
        return max([self.get_l1_p(p, value_list, v1, v2, v3) for p in range(1, int(np.ceil(v3 / 2)) + 1)])

    def get_l1_p(self, p, value_list, v1, v2, v3):
        j_set = [box for box in value_list if (box[0] > v1 / 2) and (box[1] > v2 / 2)]
        j_set_card = len([box for box in j_set if (box[2] > v3 / 2)])
        Jl = [box[2] for box in j_set if v3 - p >= box[2] > v3 / 2]
        Js = [box[2] for box in j_set if v3 / 2 >= box[2] >= p]
        first_parameter = np.ceil(
            (1 / v3) * (np.sum([vv3 for vv3 in Js]) - (len(Jl) * v3 - np.sum([vv3 for vv3 in Jl]))))
        second_parameter = np.ceil(
            (len(Js) - (np.sum([np.floor((v3 - vv3) / p) for vv3 in Jl]))) / np.floor(v3 / p))
        max_val = max(first_parameter, second_parameter)
        return j_set_card + max_val

    def calculate_l2_bound(self):
        W = self.bin.width
        H = self.bin.height
        D = self.bin.depth
        width_values = []
        height_values = []
        depth_values = []
        for box in self.boxList:
            if box.width not in width_values and box.width <= W / 2:
                width_values.append(box.width)
            if box.height not in height_values and box.height <= H / 2:
                height_values.append(box.height)
            if box.depth not in depth_values and box.depth <= D / 2:
                depth_values.append(box.depth)
        if len(width_values) == 0 or len(height_values) == 0 or len(depth_values) == 0:
            return self.get_l1_bound()
        l2_w_h = self.get_l2_p_q_max(width_values, height_values, self.list_w_h, W, H, D, self.l1_w_h)
        l2_w_d = self.get_l2_p_q_max(width_values, depth_values, self.list_w_d, W, D, H, self.l1_w_d)
        l2_h_d = self.get_l2_p_q_max(height_values, depth_values, self.list_h_d, H, D, W, self.l1_h_d)
        self.l2 = max(l2_w_h, l2_w_d, l2_h_d)

    def get_l2_bound(self):
        return self.l2

    def get_l2_p_q_max(self, p_array, q_array, value_list, v1, v2, v3, l1_val):
        return max([self.get_l2_p_q(p, q, value_list, v1, v2, v3, l1_val) for p in p_array for q in q_array])

    def get_l2_p_q(self, p, q, value_list, v1, v2, v3, l1_val):
        Kv = [box for box in value_list
              if (box[0] > (v1 - p)) and (box[1] > (v2 - q))]
        Kl = [box for box in value_list
              if (box not in Kv) and (box[0] > v1 / 2) and (box[1] > v2 / 2)]
        Ks = [box for box in value_list
              if (box not in (Kv + Kl)) and (box[0] >= p) and (box[1] >= q)]
        alpha = sum(b[0] * b[1] * b[2] for b in (Kl + Ks))
        beta = v1 * v2 * ((v3 * l1_val) - sum(b[2] for b in Kv))
        value = np.ceil((alpha - beta) / self.bin.get_volume())
        return l1_val + max(0, value)


class SingleBinProblem:

    def __init__(self, bin):
        self.bin = bin
        self.boxList = []
        self.open = True
        self.F = F_INITIAL_VALUE

    def order_box_set(self, box_set):
        box_set = sorted(box_set, key=lambda box: box.get_end_x(), reverse=True)  # check if it works properly
        box_set = sorted(box_set, key=lambda box: box.get_end_y(), reverse=True)  # check if it works properly

        return box_set

    def two_dimensional_corners(self, box_set, J, bin):
        if box_set is None or len(box_set) == 0:
            return [Point2D(0, 0)]

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
            corners.append(Point2D(extreme_boxes[index - 1].get_end_x(), extreme_boxes[index].get_end_y()))
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

    def compute_area(self, points2D, bin):
        if len(points2D) == 1 and points2D[0] == Point2D(0, 0):
            return bin.get_height() * bin.get_width()

        area = points2D[0].get_x() * bin.get_height
        for index in range(1, len(points2D)):
            area = area + (points2D[index].get_x() - points2D[index - 1].get_x()) * points2D[index - 1].get_y()
        area = area + (bin.get_width() - points2D[-1].get_x()) * points2D[-1].get_y()

        return area

    def three_dimensional_corners(self, box_set, J, bin):
        if box_set is None or len(box_set) == 0:
            return [Point3D(0, 0, 0)], 0

        # must order the I set for future uses (in the for loop above)
        box_set = self.order_box_set(box_set)

        T = [0]
        for box in box_set:
            T.append(box.get_end_z())

        T = sorted(T, reverse=False)  # devo gestire casi di profondità doppie? non penso però...

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
                if len(incremental_corners) > 1:
                    for already_seen_point in incremental_corners[-2]:
                        if point.get_x() == already_seen_point.get_x() and point.get_y() == already_seen_point.get_y():
                            found = True
                            break
                if not found:
                    total_corners.append(Point3D(point.get_x(), point.get_y(), depth))

            k = k + 1

        volume = 0.0
        for box in box_set:
            volume = volume + box.get_volume()

        return total_corners, volume

    def branch_and_bound_filling(self, placed_boxes, not_placed_boxes):
        if len(not_placed_boxes) == 0:
            return True
        points, VI = self.three_dimensional_corners(placed_boxes, not_placed_boxes, self.bin)
        if not self.check_backtrack_condition(placed_boxes, VI):
            return False
        for p in points:
            # Dall'articolo é meglio ordinare le scatole per volume decrescente
            for box in not_placed_boxes:
                # piazzo una scatola
                box.position = p
                # qui si dovrá aggiungere il vincolo del peso
                if (box.get_end_x() <= self.bin.width and
                        box.get_end_y() <= self.bin.height and
                        box.get_end_z() <= self.bin.depth):
                    new_placed_boxes = [b for b in placed_boxes] + [box]
                    new_not_placed_boxes = [b for b in not_placed_boxes if not b == box]
                    if self.branch_and_bound_filling(new_placed_boxes, new_not_placed_boxes):
                        return True
                # ripristino la scatola, in quanto ho fallito
                box.position = NOT_PLACED_POINT
        self.update_best_filling_value(sum([b.volume for b in placed_boxes]))
        return False

    def fillBin(self):
        self.reset_problem()
        result = self.branch_and_bound_filling([], self.boxList)
        if result:
            return True, self.boxList
        else:
            return False, []

    def reset_problem(self):
        for b in self.boxList:
            b.position = NOT_PLACED_POINT
        self.F = F_INITIAL_VALUE

    def update_best_filling_value(self, new_F):
        if self.F < new_F:
            self.F = new_F

    def check_backtrack_condition(self, placed_boxes, VI):
        placed_volume = sum([b.volume for b in placed_boxes])
        to_check_value = placed_volume + (self.bin.volume - VI)
        if to_check_value <= self.F:
            return False
        else:
            return True
