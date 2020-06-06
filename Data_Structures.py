# coding=utf-8
import numpy as np

F_INITIAL_VALUE = -1

DEBUG = True
DEFAULT_MAX_WEIGHT = 10
DEFAULT_WEIGHT = 1

class Box:
    def __init__(self, width, height, depth):
        self.width = width
        self.height = height
        self.depth = depth
        self.volume = self.width * self.height * self.depth
        self.weight = DEFAULT_WEIGHT
        self.assigned = False
        self.position = NOT_PLACED_POINT
        self.belowBoxes = []
        self.maximumWeight = DEFAULT_MAX_WEIGHT

    def get_maximumWeight(self):
        return self.maximumWeight

    def get_weight(self):
        return self.weight

    def get_below_boxes(self):
        return self.belowBoxes

    def set_below_boxes(self, boxes):
        self.belowBoxes = boxes

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

    def __init__(self, bin, boxList, openBins=None):
        if openBins is None:
            openBins = []
        self.boxList = sorted(boxList, key=lambda box: box.get_volume(), reverse=False)
        self.bin = bin
        self.M = openBins

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
        width_values = []
        height_values = []
        depth_values = []
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

    def try_to_close(self, left_box_list, sp):
        sp = sp.__copy__()
        try_to_place_list = []
        for j in left_box_list:
            to_check = sp.boxList + [j]
            if not self.get_l2_bound(to_check) >= 2:
                try_to_place_list.append(j)
        if len(try_to_place_list) == 0:
            sp.open = False
            return sp
        else:
            sb_list = H2(sp.boxList + try_to_place_list, self.bin)
            if len(sb_list) == 1:
                sb_list[0].open = False
                for box in try_to_place_list:
                    left_box_list.remove(box)
                return sb_list[0]
            else:
                return sp

    def get_neighbor_new_open_bin(self, next_box, incumbent):
        left_box_list = [box for box in self.boxList if box != next_box]
        sp = SingleBinProblem(self.bin)
        sp.add_boxes(next_box)
        sp.fillBin()
        result, sp = self.try_to_close(left_box_list, sp)
        new_node = PalletizationModel(self.bin, left_box_list,
                                      [s.__copy__() for s in self.M] + [sp])
        if sp.open == False:
            l2 = self.get_l2_bound(left_box_list)
            if not l2 + len([spc for spc in new_node.M if spc.open == False]) >= incumbent:
                return new_node
            else:
                return None
        else:
            return new_node

    def get_closed_bins(self):
        dim = len([c for c in self.M if c.open == False])
        return dim

    # def get_neighbor(self, i, next_box):
    #     new_M = [s.__copy__() for s in self.M]
    #     sb = new_M[i]
    #     if not self.get_l2_bound(sb.boxList + [next_box]) >= 2:
    #         sb_H2 = H2(sb.boxList + [next_box], self.bin)
    #         # DA AGGIUNGERE H1
    #         if len(sb_H2) == 1:
    #             new_M[i] = sb_H2[0]
    #             new_M[i].try_to_close()
    #             return new_M
    #         else:
    #             result = sb.fillBin()
    #             if result == []:
    #                 new_M[i].try_to_close()
    #                 return new_M
    #
    # def get_neighborhood(self, node, incumbent):
    #     next_box = self.boxList[0]
    #     neighborhood = []
    #     if len(node.M) < self.Z - 1:
    #         new_node = self.get_neighbor_new_open_bin(next_box)
    #     for i in range(self.M):
    #         pass


class SingleBinProblem:

    def __init__(self, bin):
        self.bin = bin
        self.boxList = []
        self.open = True
        self.F = F_INITIAL_VALUE
        self.placement_best_solution = []
        self.withWeight = False

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
            if (corner.get_x() + minimum_w) <= self.bin.get_width() and (corner.get_y() + minimum_h) <= self.bin.get_height():
                final_corners.append(corner)

        return final_corners

    def compute_area(self, points2D):
        if len(points2D) == 1 and points2D[0] == Point2D(0, 0):
            return self.bin.get_height() * self.bin.get_width()

        area = points2D[0].get_x() * self.bin.get_height
        for index in range(1, len(points2D)):
            area = area + (points2D[index].get_x() - points2D[index - 1].get_x()) * points2D[index - 1].get_y()
        area = area + (self.bin.get_width() - points2D[-1].get_x()) * points2D[-1].get_y()

        return area

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
            if box.get_depth() > minimum_d:
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

        return total_corners, volume

    def check_weight_condition(self, box, current_weight):
        if len(box.get_below_boxes()) > 0:
            for below_box in box.get_below_boxes():
                if below_box.get_maximumWeight() < current_weight:
                    return False

                if not self.check_weight_condition(below_box, current_weight+below_box.get_weight()):
                    return False

        return True

    def branch_and_bound_filling(self, placed_boxes, not_placed_boxes, m_cut=False, m=4):
        if len(not_placed_boxes) == 0:
            return True

        points, VI = self.three_dimensional_corners(placed_boxes, not_placed_boxes)

        if not self.check_backtrack_condition(placed_boxes, VI):
            return False

        for p in points:
            # Dall'articolo é meglio ordinare le scatole per volume decrescente
            for box in not_placed_boxes:
                # piazzo una scatola
                box.position = p
                # qui si dovrá aggiungere il vincolo del peso.0

                if (box.get_end_x() <= self.bin.width and
                        box.get_end_y() <= self.bin.height and
                        box.get_end_z() <= self.bin.depth):

                    # start modifiche

                    below_boxes = self.getBoxesBelow(box, placed_boxes)
                    if self.withWeight:
                        box.set_below_boxes(below_boxes)

                        if self.check_weight_condition(box, box.weight):
                            # piazzo la scatola ed eventualmente distribuisco il peso
                            # per distribuire il peso servirà salvarsi un current_weight_supported o qualcosa del genere
                            # ma per adesso controlla se funziona così
                            new_placed_boxes = [b for b in placed_boxes] + [box]
                            new_not_placed_boxes = [b for b in not_placed_boxes if not b == box]

                            if self.branch_and_bound_filling(new_placed_boxes, new_not_placed_boxes):
                                return True
                    else:
                        new_placed_boxes = [b for b in placed_boxes] + [box]
                        new_not_placed_boxes = [b for b in not_placed_boxes if not b == box]

                        if self.branch_and_bound_filling(new_placed_boxes, new_not_placed_boxes):
                            return True
                # ripristino la scatola, in quanto ho fallito
                box.position = NOT_PLACED_POINT
                box.set_below_boxes([])
        self.update_best_filling(placed_boxes)
        return False

    def fillBin(self):
        self.reset_problem()
        result = self.branch_and_bound_filling([], self.boxList)
        if result == True:
            self.update_best_filling(placed_boxes=self.boxList)
            return []
        placed_boxes = [box for (box, pos) in self.placement_best_solution]
        not_placed_boxes = [box for box in self.boxList if box not in placed_boxes]
        self.boxList = placed_boxes
        return not_placed_boxes

    def reset_problem(self):
        for b in self.boxList:
            b.position = NOT_PLACED_POINT

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
            self.placement_best_solution = [(box, box.position) for box in placed_boxes]


# FOR TEST PURPOSES
def getBoxesBelow(box, placed_boxes):
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
            condition_x = not (tmp_box.get_pos_x() >= to_place_box_end_x or tmp_box.get_end_x() <= to_place_box_start_x)
            condition_z = not (tmp_box.get_pos_z() >= to_place_box_end_z or tmp_box.get_end_z() <= to_place_box_start_z)

            if condition_x and condition_z:
                below_boxes.append(tmp_box)

    return below_boxes


def H1(boxList, bin):
    return -1


def H2(boxList, bin):
    box_set = [box for box in boxList]
    sb_list = []
    while box_set != []:
        sb = SingleBinProblem(bin)
        sb.add_boxes(box_set)
        not_placed_boxes = sb.fillBin()
        box_set = not_placed_boxes
        sb_list.append(sb)
    return sb_list


class Search:

    def __init__(self,bin, allBoxes):
        self.Z = len(H2(allBoxes, bin))
        self.allBoxes = allBoxes
        self.bin = bin

    # def search(self):
    #     open_list = []
    #     current_node = PalletizationModel(self.allBoxes, self.bin)
    #     while not current_node.final_state():
    #         neighborhood = current_node.get_neighborhood(current_node)
    #         for n in neighborhood:
    #             open_list.append(n)
    #         current_node = open_list[len(open_list) - 1]
    #     return current_node

    def backtracking_search(self, current_problem):
        not_placed_boxes = current_problem.boxList
        if current_problem.get_l2_bound(current_problem.boxList) + current_problem.get_closed_bins() >= self.Z:
            return "fail"
        if not_placed_boxes == []:
            return current_problem
        box = not_placed_boxes[0]
        for i in range(len(current_problem.M)):
            new_p = current_problem.__copy__()
            new_not_placed_boxes = [b for b in not_placed_boxes[1:]]
            new_p.boxList = new_not_placed_boxes
            new_sbp = new_p.M[i]
            new_sbp.add_boxes(box)
            single_bin_result = new_sbp.fillBin()
            if single_bin_result == []:
                result = self.backtracking_search(new_p)
                if result != "fail":
                    return result
        if len(current_problem.M) < self.Z - 1:
            new_sbp = SingleBinProblem(self.bin)
            new_sbp.add_boxes(box)
            new_sbp.fillBin()
            new_not_placed_boxes = [b for b in not_placed_boxes[1:]]
            new_bins = [sbp.__copy__() for sbp in current_problem.M]
            new_p = PalletizationModel(self.bin, new_not_placed_boxes, new_bins + [new_sbp])
            result = self.backtracking_search(new_p)
            if result != "fail":
                return result
        return "fail"


