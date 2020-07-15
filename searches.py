import time
import random

import Data_Structures as ds

csv_format = "{},{},{},{},{},{},{},{},{}\n"


def assign_box_to_new_bin(box, current_problem, not_placed_boxes, optimized=False):
    new_sbp = ds.SingleBinProblem(current_problem.bin)
    new_sbp.add_boxes(box)
    new_sbp.fillBin(optimized=optimized)
    new_not_placed_boxes = [b for b in not_placed_boxes[1:]]
    new_bins = [sbp.__copy__() for sbp in current_problem.M]
    new_p = ds.PalletizationModel(current_problem.bin, new_not_placed_boxes, new_bins + [new_sbp])
    new_p.try_to_close(len(new_p.M) - 1, optimized=optimized)
    return new_p


def assign_box_to_bin(box, current_problem, i, not_placed_boxes, optimal=True, nodes=5000, optimized=False):
    new_p = current_problem.__copy__()
    new_not_placed_boxes = [b for b in not_placed_boxes[1:]]
    new_p.boxList = new_not_placed_boxes
    new_sbp = new_p.M[i]
    new_sbp.add_boxes(box)
    if optimal:
        h2_result = ds.H2(new_sbp.boxList, new_p.bin, optimized=optimized)
        if len(h2_result) == 1:
            new_p.M[i] = h2_result[0]
            return new_p, []
        single_bin_result = new_sbp.fillBin(optimized=optimized)
        return new_p, single_bin_result
    else:
        new_sbp.max_nodes = 5000
        new_sbp.m_cut = True
        new_sbp.m = 4
        single_bin_result = new_sbp.fillBin(optimized=optimized)
        return new_p, single_bin_result


def backtracking_condition(current_problem, i, box):
    if current_problem.M[i].open:
        l2 = current_problem.get_l2_bound(current_problem.M[i].boxList + [box])
        if not l2 >= 2:
            return True
    return False


def assign_box_to_bin_min_max(box, current_problem, i, not_placed_boxes):
    new_p = current_problem.__copy__()
    new_not_placed_boxes = [b for b in not_placed_boxes[1:]]
    new_p.boxList = new_not_placed_boxes
    new_sbp = new_p.M[i]
    new_sbp.add_boxes(box)
    single_bin_result = new_sbp.fillBin(optimized=True)
    return new_p, single_bin_result


def check_min_bound_feasibility(problem, not_placed_boxes):
    lower_violation = problem.get_lower_violations()
    items = problem.minDict.keys()
    for item in items:
        remaining_items = len([b for b in not_placed_boxes if b.itemName == item])
        for sb in lower_violation:
            items_placed = len([b for b in sb.placement_best_solution if b.itemName == item])
            if items_placed < problem.minDict[item]:
                to_add = problem.minDict[item] - items_placed
                remaining_items -= to_add
                if remaining_items < 0:
                    return False
    return True


def insert_lower_bound(lower_bounds, box_list):
    bs = ds.BoxSet(box_list)
    lower_bounds.append(bs)
    #print "dimensione lower bound:" + str(len(lower_bounds))


def check_lower_bound(lower_bounds, box_list):
    if ds.BoxSet(box_list) in lower_bounds:
        print "match lower bound"
        return False
    return True


def add_optimal_solution(optimal_solutions, placement_best_solution):
    bs = ds.BoxSet(placement_best_solution)
    bs.add_placement(placement_best_solution)
    optimal_solutions.append(bs)
    #print "dimensione delle soluzioni ottime:" + str(len(optimal_solutions))


def get_soluzione_ottima(bs, optimal_solutions):
    for opt in optimal_solutions:
        if bs.__eq__(opt):
             return opt
    return None


def assign_box_to_bin_v2(box, current_problem, i, not_placed_boxes, optimal_solutions, lower_bounds, optimal=True, nodes=5000, optimized=True):
    to_place = [box] + current_problem.M[i].boxList
    bs = ds.BoxSet(to_place)
    optimal_placement = get_soluzione_ottima(bs, optimal_solutions)
    if optimal_placement is None:
        new_p = current_problem.__copy__()
        new_not_placed_boxes = [b for b in not_placed_boxes[1:]]
        new_p.boxList = new_not_placed_boxes
        new_sbp = new_p.M[i]
        new_sbp.add_boxes(box)
        h2_result = ds.H2(new_sbp.boxList, new_p.bin, optimized=optimized)
        if len(h2_result) == 1:
            new_p.M[i] = h2_result[0]
            add_optimal_solution(optimal_solutions, h2_result[0].placement_best_solution)
            return new_p, []
        #print "sto usando fill bin"
        if not optimal:
            new_sbp.max_nodes = nodes
            new_sbp.m_cut = True
            new_sbp.m = 1
        single_bin_result = new_sbp.fillBin(optimized=optimized)
        if single_bin_result == []:
            add_optimal_solution(optimal_solutions, new_sbp.placement_best_solution)
        else:
            insert_lower_bound(lower_bounds, current_problem.M[i].boxList + single_bin_result)
        return new_p, single_bin_result
    else:
        new_p = current_problem.__copy__()
        new_not_placed_boxes = [b for b in not_placed_boxes[1:]]
        new_p.boxList = new_not_placed_boxes
        new_sbp = new_p.M[i]
        new_sbp.boxList = optimal_placement.placement
        new_sbp.placement_best_solution = optimal_placement.placement
        #print "soluzione ottima riutilizzata"
        return new_p, []


class IDSearchMinMaxConstraints:

    def __init__(self, first_problem, optimal=True):
        self.first_problem = first_problem
        self.optimal = optimal
        if not optimal:
            random.shuffle(first_problem.boxList)
        else:
            self.first_problem.boxList = sorted(self.first_problem.boxList, key=lambda box: box.get_volume(), reverse=True)
        self.max_depth = self.first_problem.get_l2_bound(self.first_problem.boxList)
        self.lower_bounds = []
        self.optimal_solutions = []
        self.max_nodes = int(len(self.first_problem.boxList) * 2)
        self.node_count = 0

    def inizialize_problem_depth(self):
        problem_copy = self.first_problem.__copy__()
        problem_copy.boxList = [box for box in self.first_problem.boxList]
        for i in range(int(self.max_depth)):
            problem_copy.M.append(ds.SingleBinProblem(problem_copy.bin))
        min_constr = problem_copy.fill_min_bin()
        self.node_count = 0
        return problem_copy, min_constr

    def initialize_min_constraints(self):
        problem, min_constr = self.inizialize_problem_depth()
        return min_constr, problem

    def search_id(self):
        f, problem = self.initialize_min_constraints()
        if not f:
            return 'fail'
        res = self.backtracking_search_optimized_id_min_max(problem)

        while res == 'fail':
            self.max_depth += 1
            self.max_nodes = self.max_nodes * 2
            f, problem = self.initialize_min_constraints()
            if not f:
                return 'fail'
            res = self.backtracking_search_optimized_id_min_max(problem)

            if self.max_depth >= len(self.first_problem.boxList):
                return 'fail'
        return res

    def search_id_multi(self, index, risultato):
        f, problem = self.initialize_min_constraints()
        if not f:
            risultato[index] = 'fail'
        res = self.backtracking_search_optimized_id_min_max(problem)
        while res == 'fail':
            print "aumento"
            self.max_depth += 1
            self.max_nodes = self.max_nodes * 2
            f, problem = self.initialize_min_constraints()
            if not f:
                risultato[index] = 'fail'
                return
            res = self.backtracking_search_optimized_id_min_max(problem)
            if self.max_depth >= len(self.first_problem.boxList):
                risultato[index] = 'fail'
                return
        risultato[index] = res

    def backtracking_search_optimized_id_min_max(self, current_problem):
        if self.node_count < self.max_nodes or self.optimal:
            not_placed_boxes = current_problem.boxList
            if not_placed_boxes == []:
                if current_problem.check_item_count():
                    return current_problem
                else:
                    return "fail"
            else:
                box = not_placed_boxes[0]
                for i in range(len(current_problem.M)):
                    if self.node_count < self.max_nodes or self.optimal \
                            and backtracking_condition(current_problem, i, box) \
                            and current_problem.check_item_upper():
                        new_p, single_bin_result = assign_box_to_bin(box,
                                                                     current_problem,
                                                                     i,
                                                                     not_placed_boxes,
                                                                     nodes=self.max_nodes,
                                                                     optimal=self.optimal,
                                                                     optimized=True)
                        if single_bin_result == []:
                            self.node_count += 1
                            print self.node_count
                            result = self.backtracking_search_optimized_id_min_max(new_p)
                            if result != "fail":
                                return result
                return "fail"
        else:
            return "fail"