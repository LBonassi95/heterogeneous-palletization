import getopt
import os
import sys
import datetime
import Data_Structures as ds
import time
import searches
import multiprocessing


csv_format = "{},{},{},{},{},{},{},{},{}\n"


def execute_test(box_list, bin, i):
    INSTANCE = i
    box_list = box_list
    categories = set()
    TOT_BOXES = len(box_list)
    for box in box_list:
        categories.add(box.itemName)
    NUM_CATEGORIES = len(categories)
    SPLIT = ""
    for c in categories:
        card_cat = len([box for box in box_list if box.itemName == c])
        SPLIT = SPLIT + str(float(card_cat)/float(len(box_list))) + ":"

    min_item_dict = {'item1': 2, 'item2': 1, 'item3': 1, 'item4': 1, 'item5': 2}
    #max_item_dict = {'item3': int(4+i/2)}

    manager = multiprocessing.Manager()
    return_values = manager.dict()
    jobs = []
    NUM_PROCESSES = 1
    start_time_id = time.time()
    for index in range(NUM_PROCESSES):
        model = ds.PalletizationModel(bin, box_list, minDict=min_item_dict, maxDict={})
        s = searches.IDSearchMinMaxConstraints(model, optimal=False)
        p = multiprocessing.Process(target=s.search_id_multi, args=(index, return_values))
        jobs.append(p)
        p.start()
    for process in jobs:
        process.join()

    TIME_OPTIMAL_SOLUTION = time.time() - start_time_id
    best_res = None
    best_val = 1e10
    for result in return_values.keys():
        if return_values.values()[result] != 'fail' and len(return_values.values()[result].M) < best_val:
            best_res = return_values.values()[result].M

    start_time = time.time()

    FIRST_SOLUTION = len(ds.H2(box_list, bin, optimized=True))

    TIME_FIRST_SOLUTION = time.time() - start_time

    if best_res is not None:
        SOLUTION = len(best_res)
    else:
        SOLUTION = '-1'
    results = open("./Test/results.csv", 'a', 0)
    results.write(csv_format.format(INSTANCE, TOT_BOXES, NUM_CATEGORIES, SPLIT, "ID", FIRST_SOLUTION,
                                    TIME_FIRST_SOLUTION, SOLUTION, TIME_OPTIMAL_SOLUTION))
    results.close()


def getBoxes(j):
    box_list1 = [ds.Box(1, 4, 1) for i in range(5 + j*2)]
    box_list2 = [ds.Box(2, 3, 2) for i in range(5 + j*2)]
    box_list3 = [ds.Box(5, 1, 4) for i in range(5 + j*2)]
    box_list4 = [ds.Box(1, 4, 4) for i in range(5 + j*2)]
    box_list5 = [ds.Box(1, 1, 1) for i in range(5*(2 + j*2))]
    for box in box_list1:
        box.itemName = 'item1'
        box.weight = 10
        box.maximumWeight = box.weight*3
    for box in box_list2:
        box.itemName = 'item2'
        box.weight = 4
        box.maximumWeight = box.weight*3
    for box in box_list3:
        box.itemName = 'item3'
        box.weight = 5
        box.maximumWeight = box.weight*3
    for box in box_list4:
        box.itemName = 'item4'
        box.weight = 7
        box.maximumWeight = box.weight*3
    for box in box_list5:
        box.itemName = 'item5'
        box.weight = 1
        box.maximumWeight = box.weight*3
    box_list = box_list1 + box_list2 + box_list3 + box_list4 + box_list5
    for i in range(len(box_list)):
        box_list[i].id = i
    return box_list


def main():
    results = open("./Test/results.csv", 'a', 0)
    results.write("INSTANCE,TOT_BOXES,NUM_CATEGORIES,SPLIT,STRATEGY,H2_SOLUTION,TIME_H2_SOLUTION,SOLUTION,TIME_SOLUTION\n")
    results.close()
    bin = ds.Bin(7, 9, 7)
    bin.set_maxWeight(50)
    for i in range(50):
        box_list = getBoxes(i)
        execute_test(box_list, bin, i)
        print "test fatto"
    # execute_test_multi(getBoxes(50))


if __name__ == '__main__':
    main()









