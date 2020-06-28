import time

import Data_Structures as ds

csv_format = "{},{},{},{},{},{},{},{},{}\n"

class IDSearch:

    def __init__(self, first_problem):
        self.first_problem = first_problem
        self.first_problem.boxList = sorted(self.first_problem.boxList, key=lambda box: box.get_volume(), reverse=True)
        self.max_depth = self.first_problem.get_l2_bound(self.first_problem.boxList)
        #self.max_depth = 1

    def search_id(self):
        res = self.backtracking_search_optimized_id(self.first_problem)
        while res == 'fail':
            self.max_depth += 1
            print "aumento"
            res = self.backtracking_search_optimized_id(self.first_problem)
        return res


    def assign_box_to_new_bin(self, box, current_problem, not_placed_boxes, optimized=False):
        new_sbp = ds.SingleBinProblem(current_problem.bin)
        new_sbp.add_boxes(box)
        new_sbp.fillBin(optimized=optimized)
        new_not_placed_boxes = [b for b in not_placed_boxes[1:]]
        new_bins = [sbp.__copy__() for sbp in current_problem.M]
        new_p = ds.PalletizationModel(current_problem.bin, new_not_placed_boxes, new_bins + [new_sbp])
        new_p.try_to_close(len(new_p.M) - 1, optimized=optimized)
        return new_p

    def assign_box_to_bin(self, box, current_problem, i, not_placed_boxes, optimized=False):
        new_p = current_problem.__copy__()
        new_not_placed_boxes = [b for b in not_placed_boxes[1:]]
        new_p.boxList = new_not_placed_boxes
        new_sbp = new_p.M[i]
        new_sbp.add_boxes(box)
        h2_result = ds.H2(new_p.boxList, new_p.bin, optimized=optimized)
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

    def backtracking_search_optimized_id(self, current_problem):
        not_placed_boxes = current_problem.boxList
        if not_placed_boxes == []:
            return current_problem
        box = not_placed_boxes[0]
        for i in range(len(current_problem.M)):
            if self.backtracking_condition(current_problem, i, box):
                new_p, single_bin_result = self.assign_box_to_bin(box, current_problem, i, not_placed_boxes, optimized=True)
                if single_bin_result == []:
                    new_p.try_to_close(i, optimized=True)
                    result = self.backtracking_search_optimized_id(new_p)
                    if result != "fail":
                        return result
        if len(current_problem.M) < self.max_depth:
            new_p = self.assign_box_to_new_bin(box, current_problem, not_placed_boxes, optimized=True)
            result = self.backtracking_search_optimized_id(new_p)
            if result != "fail":
                return result
        return "fail"


class SearchAnyTime:

    def __init__(self, first_problem):
        self.first_problem = first_problem
        self.first_problem.boxList = sorted(self.first_problem.boxList, key=lambda box: box.get_volume(), reverse=True)
        self.init_solution = ds.H2(first_problem.boxList, first_problem.bin, optimized=True)
        self.Z = len(self.init_solution)

    def search(self):
        res = self.backtracking_search_optimized(self.first_problem)
        print "soluzione trovata"
        if res == "fail":
            p = self.first_problem.__copy__()
            p.M = self.init_solution
            return p
        solutions = []
        while res != "fail":
            solutions.append(res)
            self.Z = len(res.M)
            res = self.backtracking_search_optimized(self.first_problem)
            print "soluzione trovata"
        return solutions[len(solutions)-1]

    def assign_box_to_new_bin(self, box, current_problem, not_placed_boxes, optimized=False):
        new_sbp = ds.SingleBinProblem(current_problem.bin)
        new_sbp.add_boxes(box)
        new_sbp.fillBin(optimized=optimized)
        new_not_placed_boxes = [b for b in not_placed_boxes[1:]]
        new_bins = [sbp.__copy__() for sbp in current_problem.M]
        new_p = ds.PalletizationModel(current_problem.bin, new_not_placed_boxes, new_bins + [new_sbp])
        new_p.try_to_close(len(new_p.M) - 1, optimized=optimized)
        return new_p

    def assign_box_to_bin(self, box, current_problem, i, not_placed_boxes, optimized=False):
        new_p = current_problem.__copy__()
        new_not_placed_boxes = [b for b in not_placed_boxes[1:]]
        new_p.boxList = new_not_placed_boxes
        new_sbp = new_p.M[i]
        new_sbp.add_boxes(box)
        h2_result = ds.H2(new_p.boxList, new_p.bin, optimized=optimized)
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

    def search_info(self, results, INSTANCE, TOT_BOXES, NUM_CATEGORIES, SPLIT, STRATEGY, FIRST_SOLUTION,
                    TIME_FIRST_SOLUTION):
        start_time = time.time()
        res = self.backtracking_search_optimized(self.first_problem)
        if res == "fail":
            p = self.first_problem.__copy__()
            p.M = self.init_solution
            results.write(csv_format.format(INSTANCE, TOT_BOXES, NUM_CATEGORIES, SPLIT, STRATEGY, FIRST_SOLUTION,
                                            TIME_FIRST_SOLUTION, "/", "/"))
            return p
        solutions = []
        while res != "fail":
            results.write(csv_format.format(INSTANCE, TOT_BOXES, NUM_CATEGORIES, SPLIT, STRATEGY, FIRST_SOLUTION,
                                            TIME_FIRST_SOLUTION, len(res.M), time.time()-start_time))
            solutions.append(res)
            self.Z = len(res.M)
            start_time = time.time()
            res = self.backtracking_search_optimized(self.first_problem)
