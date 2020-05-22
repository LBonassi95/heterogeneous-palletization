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
    model = get_random_model(100)
    l1 = model.get_l1_bound()
    print(l1)
    # l2 = model.get_l2_bound()
    # print(l2)

