import Data_Structures as ds
import searches
import XmlParser
import multiprocessing
import sys
import getopt
import json


def save_result(res):
    dict = {}
    for i in range(len(res)):
        sb = res[i]
        bin_name = "bin-"+str(i)
        dict[bin_name] = {}
        dict[bin_name]["box_list"] = []
        for box in sb.placement_best_solution:
            dict[bin_name]["box_list"].append((box.itemName, box.position.x, box.position.y, box.position.z))
        dict[bin_name]["left_volume"] = sb.get_left_volume()
        dict[bin_name]["bin_weight"] = sb.get_achieved_weight()
        x, y, z = sb.get_left_space()
        dict[bin_name]["effective_width_height_depth"] = (x, y, z)

    with open('result.json', 'w') as fp:
        json.dump(dict, fp, indent=1)


def main(argv):
    opts, args = getopt.getopt(argv, "o:x:")

    optimal = False
    path = None

    for opt, arg in opts:
        if opt == "-x":
            path = arg
        elif opt == "-o":
            optimal = True

    problem = XmlParser.xml2problem(path)

    res = execute_test_multi(problem, optimal)

    save_result(res)


def execute_test_multi(problem, optimal):
    manager = multiprocessing.Manager()
    return_values = manager.dict()
    jobs = []
    NUM_PROCESSES = 3
    print "pallettization started"
    for index in range(NUM_PROCESSES):
        s = searches.IDSearchMinMaxConstraints(problem, optimal=optimal)
        p = multiprocessing.Process(target=s.search_id_multi, args=(index, return_values))
        jobs.append(p)
        p.start()
    for process in jobs:
        process.join()

    best_res = None
    best_val = 1e10
    for result in return_values.keys():
        if len(return_values.values()[result].M) < best_val:
            best_res = return_values.values()[result].M
    return best_res



if __name__ == "__main__":
    main(sys.argv[1:])