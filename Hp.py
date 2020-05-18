import Data_Structures as ds
import random


def get_random_model(num_of_boxes):
    random.seed(47)
    boxList = [ds.Box(float(random.randint(10, 70)), float(random.randint(10, 70)), float(random.randint(10, 70))) for i in range(num_of_boxes)]
    return ds.PalletizationModel(ds.Bin(100.0, 100.0, 100.0), boxList)


def get_fixed_model(num_of_boxes):
    random.seed(47)
    boxList = [ds.Box(70, 70, 70) for i in range(num_of_boxes)]
    return ds.PalletizationModel(ds.Bin(100.0, 100.0, 100.0), boxList)


if __name__ == '__main__':
    model = get_fixed_model(1723)
    l1 = model.get_l1_w_h()
    print(l1)