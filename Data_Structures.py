# coding=utf-8
import numpy as np
import random

F_INITIAL_VALUE = -1

DEBUG = False
DEFAULT_MAX_WEIGHT = 1e10
DEFAULT_WEIGHT = 1
DEFAULT_ITEM_NAME = "box"


class Box:
    def __init__(self, width, height, depth):
        self.width = width
        self.height = height
        self.depth = depth
        self.volume = self.width * self.height * self.depth
        self.weight = DEFAULT_WEIGHT
        self.position = NOT_PLACED_POINT
        self.maximumWeight = DEFAULT_MAX_WEIGHT
        self.itemName = DEFAULT_ITEM_NAME
        self.id = -1

    def copy(self):
        new_box = Box(self.width, self.height, self.depth)
        new_box.weight = self.weight
        new_box.position = Point3D(self.position.x, self.position.y, self.position.z)
        new_box.maximumWeight = self.maximumWeight
        new_box.itemName = self.itemName
        new_box.id = self.id
        return new_box

    def get_maximumWeight(self):
        return self.maximumWeight

    def get_weight(self):
        return self.weight

    def get_below_boxes(self):
        return self.belowBoxes

    def set_weight(self, weight):
        self.weight = weight

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
        return self.position.get_x()

    def get_pos_y(self):
        return self.position.get_y()

    def get_pos_z(self):
        return self.position.get_z()

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

    # def __eq__(self, other):
    #     return isinstance(other, Box) and self.position == other.position and self.width == other.width \
    #            and self.height == other.height and \
    #            self.depth == other.depth

    def get_overlapping_area(self, box2):
        x1min, x1max, z1min, z1max = self.position.x, self.position.x + self.width, self.position.z, self.position.z + self.depth
        x2min, x2max, z2min, z2max = box2.position.x, box2.position.x + box2.width, box2.position.z, box2.position.z + box2.depth
        dz = min(z1max, z2max) - max(z1min, z2min)
        dx = min(x1max, x2max) - max(x1min, x2min)
        if (dx >= 0) and (dz >= 0):
            return dx * dz
        return None


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
        return isinstance(other, Point3D) \
               and other.x == self.x \
               and other.y == self.y \
               and other.z == self.z


NOT_PLACED_POINT = Point3D(-1, -1, -1)


class Bin:
    def __init__(self, width, height, depth, maxWeight=1e10):
        self.width = width
        self.height = height
        self.depth = depth
        self.volume = self.width * self.height * self.depth
        self.maxWeight = maxWeight

    def get_maxWeight(self):
        return self.maxWeight

    def set_maxWeight(self, maxWeight):
        self.maxWeight = maxWeight

    def get_volume(self):
        return self.volume

    def get_width(self):
        return self.width

    def get_height(self):
        return self.height

    def get_depth(self):
        return self.depth


class PalletizationModel:

    def __init__(self, bin, boxList, openBins=None, minDict={}, maxDict={}):
        if openBins is None:
            openBins = []
        self.boxList = boxList
        self.bin = bin
        self.M = openBins

        self.minDict = minDict
        self.maxDict = maxDict

    def final_state(self):
        return len(self.boxList) == 0

    def __copy__(self):
        copy = PalletizationModel(self.bin, [], openBins=[b.__copy__() for b in self.M])
        return copy

    def get_bin(self):
        return self.bin

    def calculate_l1_bound(self, list_w_h, list_w_d, list_h_d):
        W = self.bin.width
        H = self.bin.height
        D = self.bin.depth
        l1_w_h = self.get_l1_p_max(list_w_h, W, H, D)
        l1_w_d = self.get_l1_p_max(list_w_d, W, D, H)
        l1_h_d = self.get_l1_p_max(list_h_d, H, D, W)
        l1 = max(l1_w_h, l1_w_d, l1_h_d)
        return l1_w_h, l1_w_d, l1_h_d, l1

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

    def calculate_l2_bound(self, boxList, list_w_h, list_w_d, list_h_d, l1_w_h, l1_w_d, l1_h_d, l1):
        W = self.bin.width
        H = self.bin.height
        D = self.bin.depth
        width_values = [1, W / 2]
        height_values = [1, H / 2]
        depth_values = [1, D / 2]
        for box in boxList:
            if box.width not in width_values and box.width <= W / 2:
                width_values.append(box.width)
            if box.height not in height_values and box.height <= H / 2:
                height_values.append(box.height)
            if box.depth not in depth_values and box.depth <= D / 2:
                depth_values.append(box.depth)
        if len(width_values) == 0 or len(height_values) == 0 or len(depth_values) == 0:
            return l1
        l2_w_h = self.get_l2_p_q_max(width_values, height_values, list_w_h, W, H, D, l1_w_h)
        l2_w_d = self.get_l2_p_q_max(width_values, depth_values, list_w_d, W, D, H, l1_w_d)
        l2_h_d = self.get_l2_p_q_max(height_values, depth_values, list_h_d, H, D, W, l1_h_d)
        return max(l2_w_h, l2_w_d, l2_h_d)

    def get_l2_bound(self, boxList):
        list_w_h = [[box.width, box.height, box.depth] for box in boxList]
        list_w_d = [[box.width, box.depth, box.height] for box in boxList]
        list_h_d = [[box.height, box.depth, box.width] for box in boxList]
        l1_w_h, l1_w_d, l1_h_d, l1 = self.calculate_l1_bound(list_w_h, list_w_d, list_h_d)
        l2 = self.calculate_l2_bound(boxList, list_w_h, list_w_d, list_h_d, l1_w_h, l1_w_d, l1_h_d, l1)
        return l2

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

    #ALGORITMO PER CERCARE DI CHIUDERE IL BIN i-ESIMO, VEDI Main Branching Tree sull'articolo
    def try_to_close(self, i, optimized = False):
        if DEBUG:
            print "sto provando a chiudere una scatola"
        sp = self.M[i]
        try_to_place_list = []
        for j in self.boxList:
            to_check = sp.boxList + [j]
            if not self.get_l2_bound(to_check) >= 2:
                try_to_place_list.append(j)
        if len(try_to_place_list) == 0:
            sp.open = False
        else:
            if not self.get_l2_bound(sp.placement_best_solution + try_to_place_list) >= 2:
                box_in_bin = [box.copy() for box in sp.placement_best_solution]
                sb_list = H2(box_in_bin + try_to_place_list, self.bin, optimized=optimized)
                if len(sb_list) == 1:
                    sb_list[0].open = False
                    self.M[i] = sb_list[0]
                    for box in try_to_place_list:
                        self.boxList.remove(box)

    def get_closed_bins(self):
        return len([m for m in self.M if m.open == False])

    def check_item_count(self):
        if len(self.maxDict.keys()) and (len(self.minDict.keys())) == 0:
            return True
        for sb in self.M:
            box_list = sb.placement_best_solution
            for key in self.minDict.keys():
                items = len([b for b in box_list if b.itemName == key])
                if items < self.minDict[key]:
                    return False
            for key in self.maxDict.keys():
                items = len([b for b in box_list if b.itemName == key])
                if items > self.maxDict[key]:
                    return False
        return True

    def check_item_upper(self):
        if len(self.maxDict.keys()) == 0:
            return True
        for sb in self.M:
            box_list = sb.placement_best_solution
            for key in self.maxDict.keys():
                items = len([b for b in box_list if b.itemName == key])
                if items > self.maxDict[key]:
                    return False
        return True


class SingleBinProblem:

    def __init__(self, bin):
        self.bin = bin
        self.boxList = []
        self.open = True
        self.F = F_INITIAL_VALUE
        self.placement_best_solution = []
        self.withWeight = False
        self.node_count = 0
        self.max_nodes = 5000
        self.m_cut = False
        self.m = 4
        self.not_placed_boxes = []

    def __copy__(self):
        copy = SingleBinProblem(self.bin)
        copy.F = self.F
        copy.boxList = [box for box in self.boxList]
        copy.open = self.open
        copy.withWeight = self.withWeight
        copy.placement_best_solution = [t for t in self.placement_best_solution]
        return copy

    # this method orders a given box set such that the ys are not increasing. If two or more boxes have the same y then
    # the x value is used as a discriminant (in a non increasing way)
    def order_box_set(self, box_set):
        box_set = sorted(box_set, key=lambda box: box.get_end_x(), reverse=True)  # check if it works properly
        box_set = sorted(box_set, key=lambda box: box.get_end_y(), reverse=True)  # check if it works properly

        return box_set

    def two_dimensional_corners(self, box_set, J):
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
            if (corner.get_x() + minimum_w) <= self.bin.get_width() and (
                    corner.get_y() + minimum_h) <= self.bin.get_height():
                final_corners.append(corner)

        return final_corners

    def compute_area(self, points2D):
        if len(points2D) == 1 and points2D[0] == Point2D(0, 0):
            return self.bin.get_height() * self.bin.get_width()
        if len(points2D) > 0:
            area = points2D[0].get_x() * self.bin.get_height
            for index in range(1, len(points2D)):
                area = area + (points2D[index].get_x() - points2D[index - 1].get_x()) * points2D[index - 1].get_y()
            area = area + (self.bin.get_width() - points2D[-1].get_x()) * points2D[-1].get_y()
            return area
        return 0

    def three_dimensional_corners(self, box_set, J):
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
            if box.get_depth() < minimum_d:
                minimum_d = box.get_depth()

        total_corners = []
        incremental_corners = []

        k = 0
        for depth in T:
            if depth + minimum_d > self.bin.get_depth():
                break

            I_k = [box for box in box_set if box.get_end_z() > depth]

            incremental_corners.append(self.two_dimensional_corners(I_k, J))
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

        # start modifiche
        # volume2 = 0.0
        # if len(incremental_corners) > 0:
        #     for index in range(1, len(incremental_corners)):
        #         tmp = [Point2D(point3D.get_x(), point3D.get_y()) for point3D in incremental_corners[index - 1]]
        #         if len(tmp) > 0:
        #             volume2 = volume2 + (self.compute_area(tmp) * (T[index] - T[index - 1]))
        #     # end modifiche
        #     volume2 = volume2 + (self.bin.get_depth() - T[-1]) * self.compute_area(
        #         [Point2D(point3D.get_x(), point3D.get_y()) for point3D in incremental_corners[-1]])
        # print(volume-volume2)

        # NOTA BENE CHE STO USANDO VOLUME2, DA ALCUNI PRIMI TEST SIMULATIVI NON SEMBRANO ESSERCI DIFFERENZE.
        # SE NOTI PROBLEMI SWITCHA A VOLUME E BASTA
        # return total_corners, volume

        return total_corners, volume

    def check_weight_condition(self, box, current_weight):
        if len(box.get_below_boxes()) > 0:
            for below_box in box.get_below_boxes():
                if below_box.get_maximumWeight() < current_weight:
                    return False

                if not self.check_weight_condition(below_box, current_weight + below_box.get_weight()):
                    return False

        return True

    def reset_problem(self):
        self.F = -1
        for b in self.boxList:
            b.position = NOT_PLACED_POINT
        self.placement_best_solution = []

    def update_best_filling_value(self, new_F):
        if self.F < new_F:
            self.F = new_F

            if DEBUG:
                print 'new best volume achieved -----> ' + str(self.F)

    def check_backtrack_condition(self, placed_boxes, VI):
        placed_volume = sum([b.volume for b in placed_boxes])
        to_check_value = placed_volume + (self.bin.volume - VI)
        if to_check_value <= self.F:
            return False
        else:
            return True

    def getBoxesBelow(self, box, placed_boxes):
        to_place_box_y = box.get_pos_y()

        if to_place_box_y == 0:
            return []

        to_place_box_end_x = box.get_end_x()
        to_place_box_start_x = box.get_pos_x()
        to_place_box_end_z = box.get_end_z()
        to_place_box_start_z = box.get_pos_z()

        below_boxes = []
        for tmp_box in placed_boxes:
            if to_place_box_y == tmp_box.get_end_y():
                condition_x = not (
                        tmp_box.get_pos_x() >= to_place_box_end_x or tmp_box.get_end_x() <= to_place_box_start_x)
                condition_z = not (
                        tmp_box.get_pos_z() >= to_place_box_end_z or tmp_box.get_end_z() <= to_place_box_start_z)

                if condition_x and condition_z:
                    below_boxes.append(tmp_box)

        return below_boxes

    def check_full_weight_condition(self, box, current_weight, placed_boxes):
        below_boxes = self.getBoxesBelow(box, placed_boxes)
        if len(below_boxes) > 0:
            for below_box in below_boxes:
                if below_box.get_maximumWeight() < current_weight:
                    return False
                if not self.check_full_weight_condition(below_box, current_weight + below_box.get_weight(), placed_boxes):
                    return False
        return True

    def below_boxes_constraints(self, box, placed_boxes):
        below_boxes = self.getBoxesBelow(box, placed_boxes)
        if box.position.y != 0:
            if len(below_boxes) == 0:
                return False
            area = 0
            for box2 in below_boxes:
                area += box.get_overlapping_area(box2)
            tot_area = box.width * box.depth
            perc_area = area/tot_area
            if perc_area > 0.5:
                return True
            else:
                if DEBUG:
                    print "violato un vincolo sull'area"
                return False
        return True

    def add_boxes(self, to_add):
        if isinstance(to_add, list):
            for box in to_add:
                self.boxList.append(box)
        else:
            self.boxList.append(to_add)

    def update_best_filling(self, placed_boxes):
        new_F = sum([b.volume for b in placed_boxes])
        if self.F < new_F:
            self.F = new_F
            self.placement_best_solution = [box.copy() for box in placed_boxes]
            self.not_placed_boxes = [box for box in self.boxList if box not in placed_boxes]

    def pos_condition(self, box):
        return (box.get_end_x() <= self.bin.width and box.get_end_y() <= self.bin.height
                and box.get_end_z() <= self.bin.depth)

    def fillBin(self, optimized=False):
        self.node_count = 0
        self.reset_problem()
        if optimized:
            result = self.branch_and_bound_filling_optimized([], self.boxList)
        else:
            result = self.branch_and_bound_filling([], self.boxList)
        if result == True:
            return []
        placed_boxes = [box for box in self.placement_best_solution]
        not_placed_boxes = self.not_placed_boxes
        self.boxList = placed_boxes
        self.not_placed_boxes = []
        return not_placed_boxes


    def branch_and_bound_filling(self, placed_boxes, not_placed_boxes):
        if len(not_placed_boxes) == 0:
            self.update_best_filling(placed_boxes)
            return True

        points, VI = self.three_dimensional_corners(placed_boxes, not_placed_boxes)

        if not self.check_backtrack_condition(placed_boxes, VI):
            return False
        self.node_count += 1

        # TODO euristica nell'ordinamento delle scatole
        possible_configuration = self.get_possible_configurations(not_placed_boxes, points)

        if self.m_cut:
            if self.node_count > self.max_nodes:
                return False
            if len(possible_configuration) > self.m:
                possible_configuration = possible_configuration[:self.m]

        for config in possible_configuration:
            (p, box) = config
            box.position = p

            if self.pos_condition(box) and self.check_full_weight_condition(box, box.get_weight(), placed_boxes):

                new_placed_boxes = [b for b in placed_boxes] + [box]
                new_not_placed_boxes = [b for b in not_placed_boxes if not b == box]
                if self.m_cut and self.node_count > self.max_nodes:
                    return False
                elif self.branch_and_bound_filling(new_placed_boxes, new_not_placed_boxes):
                    return True

            # ripristino la scatola, in quanto ho fallito
            box.position = NOT_PLACED_POINT
        self.update_best_filling(placed_boxes)
        return False

    def get_possible_configurations(self, not_placed_boxes, points):
        return [(p, box) for box in not_placed_boxes for p in points]

    # Precondizione: box1 e box2 sono uguali e posizionate alla stessa altezza
    def next_to(self, box1, box2):
        return box1.get_end_z() == box2.get_pos_z() \
               or box1.get_end_x() == box2.get_pos_x() \
               or box2.get_end_z() == box1.get_pos_z() \
               or box2.get_end_x() == box1.get_pos_x()

    # non sarebbe meglio controllare i punti dove metto la scatola?
    def on_same_level_placing(self, box1, placed_boxes):
        on_same_level = [box for box in placed_boxes if box.get_pos_y() == box1.get_pos_y()
                         and box.itemName == box1.itemName]
        if on_same_level == []:
            return True
        for box in on_same_level:
            if self.next_to(box1, box):
                return True
        return False


    def get_possible_configurations_optimized(self, not_placed_boxes, points):
        box_dict = {}
        for box in not_placed_boxes:
            if box.itemName not in box_dict:
                box_dict[box.itemName] = box
        possible_boxes = [box_dict[key] for key in box_dict.keys()]
        possible_boxes = sorted(possible_boxes, key=lambda box: box.maximumWeight, reverse=True)
        return [(p, box) for box in possible_boxes for p in points]



    def branch_and_bound_filling_optimized(self, placed_boxes, not_placed_boxes):
        if len(not_placed_boxes) == 0:
            self.update_best_filling(placed_boxes)
            return True

        points, VI = self.three_dimensional_corners(placed_boxes, not_placed_boxes)
        sorted_points = sorted(points, key=lambda p: p.y, reverse=False)

        if not self.check_backtrack_condition(placed_boxes, VI):
            return False
        self.node_count += 1

        # TODO euristica nell'ordinamento delle scatole
        possible_configuration = self.get_possible_configurations_optimized(not_placed_boxes, sorted_points)

        if self.m_cut:
            if self.node_count > self.max_nodes:
                return False
            if len(possible_configuration) > self.m:
                possible_configuration = possible_configuration[:self.m]

        for config in possible_configuration:
            (p, box) = config
            box.position = p

            if self.pos_condition(box) \
                    and self.check_pallet_weight(placed_boxes+[box])\
                    and self.check_full_weight_condition(box, box.get_weight(), placed_boxes)\
                    and self.below_boxes_constraints(box, placed_boxes)\
                    and self.on_same_level_placing(box, placed_boxes):

                new_placed_boxes = [b for b in placed_boxes] + [box]
                new_not_placed_boxes = [b for b in not_placed_boxes if not b == box]
                if self.m_cut and self.node_count > self.max_nodes:
                    return False
                elif self.branch_and_bound_filling_optimized(new_placed_boxes, new_not_placed_boxes):
                    return True

            # ripristino la scatola, in quanto ho fallito
            box.position = NOT_PLACED_POINT
        self.update_best_filling(placed_boxes)
        return False

    def check_pallet_weight(self, boxes):
        weight_pallet = sum([b.weight for b in boxes])
        return weight_pallet <= self.bin.maxWeight



# Algoritmo per trovare la soluzione iniziale
def H2(box_set, bin, m_cut=True, m=4, max_nodes=5000, optimized=False):
    box_set = [box for box in box_set]
    box_set = sorted(box_set, key=lambda box: box.get_volume(), reverse=False)
    sb_list = []
    while box_set != []:
        sb = SingleBinProblem(bin)
        sb.add_boxes(box_set)
        sb.max_nodes = max_nodes
        sb.m_cut = m_cut
        sb.m = m
        not_placed_boxes = sb.fillBin(optimized=optimized)
        box_set = not_placed_boxes
        sb_list.append(sb)
    return sb_list



class Search:

    def __init__(self, first_problem):
        self.first_problem = first_problem
        self.first_problem.boxList = sorted(self.first_problem.boxList, key=lambda box: box.get_volume(), reverse=True)
        self.Z = len(H2(first_problem.boxList, first_problem.bin, optimized=True))

    def search(self):
        return self.backtracking_search(self.first_problem)

    def search_opt(self):
        return self.backtracking_search_optimized(self.first_problem)

    #Main Branching Tree dell'articolo
    def backtracking_search(self, current_problem):
        not_placed_boxes = current_problem.boxList
        if current_problem.get_l2_bound(current_problem.boxList) + current_problem.get_closed_bins() >= self.Z:
            if DEBUG:
                print "fail"
            return "fail"
        if not_placed_boxes == []:
            return current_problem
        box = not_placed_boxes[0]
        for i in range(len(current_problem.M)):
            if self.backtracking_condition(current_problem, i, box):
                new_p, single_bin_result = self.assign_box_to_bin(box, current_problem, i, not_placed_boxes)
                if single_bin_result == []:
                    new_p.try_to_close(i)
                    if DEBUG:
                        print "profondita: " + str(len(current_problem.M))
                        print "assegno la scatola numero " + str(len(not_placed_boxes) - 1) + " al bin " + str(i)
                    result = self.backtracking_search(new_p)
                    if result != "fail":
                        return result
        if len(current_problem.M) < self.Z - 1:
            new_p = self.assign_box_to_new_bin(box, current_problem, not_placed_boxes)
            if DEBUG:
                print "profondita: " + str(len(current_problem.M))
                print "assegno la scatola numero " + str(len(not_placed_boxes) - 1) + " ad un nuovo bin "
            result = self.backtracking_search(new_p)
            if result != "fail":
                return result
        return "fail"

    def assign_box_to_new_bin(self, box, current_problem, not_placed_boxes, optimized=False):
        new_sbp = SingleBinProblem(current_problem.bin)
        new_sbp.add_boxes(box)
        new_sbp.fillBin(optimized=optimized)
        new_not_placed_boxes = [b for b in not_placed_boxes[1:]]
        new_bins = [sbp.__copy__() for sbp in current_problem.M]
        new_p = PalletizationModel(current_problem.bin, new_not_placed_boxes, new_bins + [new_sbp])
        new_p.try_to_close(len(new_p.M) - 1, optimized=optimized)
        return new_p

    def assign_box_to_bin(self, box, current_problem, i, not_placed_boxes, optimized=False):
        new_p = current_problem.__copy__()
        new_not_placed_boxes = [b for b in not_placed_boxes[1:]]
        new_p.boxList = new_not_placed_boxes
        new_sbp = new_p.M[i]
        new_sbp.add_boxes(box)
        h2_result = H2(new_p.boxList, new_p.bin, optimized=optimized)
        if len(h2_result) == 1:
            new_p.M[i] = h2_result[0]
            return new_p, []
        single_bin_result = new_sbp.fillBin(optimized=optimized)
        return new_p, single_bin_result

    def backtracking_condition(self, current_problem, i, box):
        if current_problem.M[i].open:
            l2 = current_problem.get_l2_bound(current_problem.M[i].boxList + [box])
            if not l2 >= 2:
                return True
        return False

    #Main Branching Tree dell'articolo
    def backtracking_search_optimized(self, current_problem):
        not_placed_boxes = current_problem.boxList
        if current_problem.get_l2_bound(current_problem.boxList) + current_problem.get_closed_bins() >= self.Z:
            return "fail"
        if not_placed_boxes == []:
            return current_problem
        box = not_placed_boxes[0]
        for i in range(len(current_problem.M)):
            if self.backtracking_condition(current_problem, i, box):
                new_p, single_bin_result = self.assign_box_to_bin(box, current_problem, i, not_placed_boxes, optimized=True)
                if single_bin_result == []:
                    new_p.try_to_close(i, optimized=True)
                    result = self.backtracking_search_optimized(new_p)
                    if result != "fail":
                        return result
        if len(current_problem.M) < self.Z - 1:
            new_p = self.assign_box_to_new_bin(box, current_problem, not_placed_boxes, optimized=True)
            result = self.backtracking_search_optimized(new_p)
            if result != "fail":
                return result
        return "fail"
