import getopt
import os
import sys
import XmlParser
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

    #min_item_dict = {'item1': int(1+i/3), 'item2': int(1+i/3), 'item3': int(1+i/3)}
    #max_item_dict = {'item1': int(4+i/2), 'item2': int(4+i/2), 'item3': int(4+i/2)}
    model = ds.PalletizationModel(bin, box_list, minDict={}, maxDict={})

    #iterative deepening
    start_time = time.time()
    s = searches.IDSearchMinMaxConstraints(model)
    TIME_FIRST_SOLUTION = time.time() - start_time
    FIRST_SOLUTION = len(ds.H2(box_list, bin, optimized=True))
    start_time_id = time.time()
    res = s.search_id()
    TIME_OPTIMAL_SOLUTION = time.time() - start_time_id
    SOLUTION = len(res.M)
    results = open("./Test/results.csv", 'a', 0)
    results.write(csv_format.format(INSTANCE, TOT_BOXES, NUM_CATEGORIES, SPLIT, "ID", FIRST_SOLUTION,
                                    TIME_FIRST_SOLUTION, SOLUTION, TIME_OPTIMAL_SOLUTION))
    results.close()

    # # AnyTime
    # model = ds.PalletizationModel(bin, box_list)
    #
    # s = searches.SearchAnyTime(model)
    # start_time_any = time.time()
    # res = s.search()
    # TIME_OPTIMAL_SOLUTION = time.time() - start_time_any
    # SOLUTION = len(res.M)
    # results = open("./Test/results.csv", 'a', 0)
    # results.write(csv_format.format(INSTANCE, TOT_BOXES, NUM_CATEGORIES, SPLIT, "ANY", "/",
    #                                 "/", SOLUTION, TIME_OPTIMAL_SOLUTION))
    # results.close()

def getBoxes(j):
    box_list1 = [ds.Box(3, 5, 2) for i in range(1 + j)]
    box_list2 = [ds.Box(2, 2, 2) for i in range(1 + j)]
    box_list3 = [ds.Box(2, 2, 4) for i in range(1 + j)]
    for box in box_list1:
        box.itemName = 'item1'
        box.weight = 10
        box.maximumWeight = 1e10
    for box in box_list2:
        box.itemName = 'item2'
        box.weight = 5
        box.maximumWeight = 1e10
    for box in box_list3:
        box.itemName = 'item3'
        box.weight = 4
        box.maximumWeight = 1e10
    box_list = box_list1 + box_list2 + box_list3
    for i in range(len(box_list)):
        box_list[i].id = i
    return box_list


def execute_test_multi(box_list):
    manager = multiprocessing.Manager()
    bin = ds.Bin(5, 7, 5)
    return_values = manager.dict()
    jobs = []
    NUM_PROCESSES = 3
    for index in range(NUM_PROCESSES):
        model = ds.PalletizationModel(bin, box_list, minDict={}, maxDict={})
        s = searches.IDSearchMinMaxConstraints(model, optimal=False)
        p = multiprocessing.Process(target=s.search_id_multi, args=(index, return_values))
        jobs.append(p)
        p.start()
    for process in jobs:
        process.join()

    print('Analisi dei risultatiiiii: \n')
    for result in return_values.keys():
        print(len(return_values.values()[result].M))
    print('finitoooo')
    print len(ds.H2(box_list, bin, m_cut=True, m=4, max_nodes=5000, optimized=True))


def main():
    # results = open("./Test/results.csv", 'a', 0)
    # results.write("INSTANCE,TOT_BOXES,NUM_CATEGORIES,SPLIT,STRATEGY,H2_SOLUTION,TIME_H2_SOLUTION,SOLUTION,TIME_SOLUTION\n")
    # results.close()
    # bin = ds.Bin(10, 10, 10)
    # for i in range(50):
    #     box_list = getBoxes(i)
    #     execute_test(box_list, bin, i)
    #     print "test fatto"
    execute_test_multi(getBoxes(50))


if __name__ == '__main__':
    main()









