import getopt
import os
import sys
import XmlParser
import datetime
import Data_Structures as ds
import time
import searches

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
    model = ds.PalletizationModel(bin, box_list)

    #iterative deepening
    start_time = time.time()
    s = searches.SearchAnyTime(model)
    TIME_FIRST_SOLUTION = time.time() - start_time
    FIRST_SOLUTION = s.Z
    s = searches.IDSearch(model)
    start_time_id = time.time()
    res = s.search_id()
    TIME_OPTIMAL_SOLUTION = time.time() - start_time_id
    SOLUTION = len(res.M)
    results = open("./Test/results.csv", 'a', 0)
    results.write(csv_format.format(INSTANCE, TOT_BOXES, NUM_CATEGORIES, SPLIT, "ID", FIRST_SOLUTION,
                                    TIME_FIRST_SOLUTION, SOLUTION, TIME_OPTIMAL_SOLUTION))
    results.close()

    # AnyTime
    model = ds.PalletizationModel(bin, box_list)

    s = searches.SearchAnyTime(model)
    start_time_any = time.time()
    res = s.search()
    TIME_OPTIMAL_SOLUTION = time.time() - start_time_any
    SOLUTION = len(res.M)
    results = open("./Test/results.csv", 'a', 0)
    results.write(csv_format.format(INSTANCE, TOT_BOXES, NUM_CATEGORIES, SPLIT, "ANY", "/",
                                    "/", SOLUTION, TIME_OPTIMAL_SOLUTION))
    results.close()



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


def main():
    results = open("./Test/results.csv", 'a', 0)
    results.write("INSTANCE,TOT_BOXES,NUM_CATEGORIES,SPLIT,STRATEGY,H2_SOLUTION,TIME_H2_SOLUTION,SOLUTION,TIME_SOLUTION\n")
    results.close()
    bin = ds.Bin(20, 20, 20)
    for i in range(1):
        box_list = getBoxes(i)
        execute_test(box_list, bin, i)
        print "test fatto"


if __name__ == '__main__':
    main()









