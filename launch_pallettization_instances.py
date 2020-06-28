import getopt
import os
import sys
import XmlParser
import datetime
import Data_Structures as ds
import time
import searches

csv_format = "{},{},{},{},{},{},{},{},{}\n"


def execute_test(box_list, bin, results, i):
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
        SPLIT = SPLIT + str(card_cat/len(box_list)) + ":"
    model = ds.PalletizationModel(bin, box_list)

    # iterative deepening
    start_time = time.time()
    s = searches.SearchAnyTime(model)
    TIME_FIRST_SOLUTION = time.time() - start_time
    FIRST_SOLUTION = s.Z
    s = searches.IDSearch(model)
    start_time_id = time.time()
    res = s.search_id()
    TIME_OPTIMAL_SOLUTION = time.time() - start_time_id
    SOLUTION = len(res.M)
    results.write(csv_format.format(INSTANCE, TOT_BOXES, NUM_CATEGORIES, SPLIT, "ID", FIRST_SOLUTION,
                                    TIME_FIRST_SOLUTION, SOLUTION, TIME_OPTIMAL_SOLUTION))

    # AnyTime
    model = ds.PalletizationModel(bin, box_list)
    s = searches.SearchAnyTime(model)
    s.search_info(results, INSTANCE, TOT_BOXES, NUM_CATEGORIES, SPLIT, "ANY", FIRST_SOLUTION,
                                    TIME_FIRST_SOLUTION)


def getBoxes(i):
    box_list1 = [ds.Box(3, 5, 2) for i in range(5 + i)]
    box_list2 = [ds.Box(2, 2, 2) for i in range(5 + i)]
    box_list3 = [ds.Box(2, 2, 4) for i in range(5 + i)]
    for box in box_list1:
        box.itemName = 'item1'
        box.weight = 10
        box.maximumWeight = 10
    for box in box_list2:
        box.itemName = 'item2'
        box.weight = 5
        box.maximumWeight = 5
    for box in box_list3:
        box.itemName = 'item3'
        box.weight = 4
        box.maximumWeight = 4
    box_list = box_list1 + box_list2 + box_list3
    for i in range(len(box_list)):
        box_list[i].id = i
    return box_list


def main():
    results = open("./Test/results.csv", 'w')
    results.write("INSTANCE,TOT_BOXES,NUM_CATEGORIES,SPLIT,STRATEGY,H2_SOLUTION,TIME_H2_SOLUTION,SOLUTION,TIME_SOLUTION \n")
    bin = ds.Bin(5, 5, 5)
    for i in range(5):
        box_list = getBoxes(i)
        execute_test(box_list, bin, results, i)
    results.close()



if __name__ == '__main__':
    main()









