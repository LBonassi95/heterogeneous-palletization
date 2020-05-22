import Data_Structures as ds
import random


def get_random_model(num_of_boxes):
    random.seed(47)
    boxList = [ds.Box(float(random.randint(1, 10)), float(random.randint(1, 10)), float(random.randint(1, 10))) for i in range(num_of_boxes)]
    return ds.PalletizationModel(ds.Bin(10, 10, 10), boxList)


def get_fixed_model(num_of_boxes):
    random.seed(47)
    boxList = [ds.Box(70, 70, 70) for i in range(num_of_boxes)]
    return ds.PalletizationModel(ds.Bin(100.0, 100.0, 100.0), boxList)


if __name__ == '__main__':
    # model = get_random_model(100)
    # print(model.get_l1_bound())
    # l2 = model.get_l2_bound()
    # print(l2)
    boxlist = [ds.Box(7, 6, 6), ds.Box(5, 6, 6), ds.Box(3, 6, 6)]
    bin = ds.Bin(6, 7, 8)
    p = 3
    model = ds.PalletizationModel(bin, boxlist)
    value_list = [[box.height, box.depth, box.width] for box in boxlist]
    model.calculate_l1_bound()
    print(model.get_l2_h_d(2, 3))
