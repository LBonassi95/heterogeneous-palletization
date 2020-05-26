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

    boxList = [ds.Box(70, 70, 70) for i in range(5)]  # scatole già posizionate
    boxList.append(ds.Box(100, 70, 23))
    J = [ds.Box(1, 2, 1) for i in range(3)]  # scatole che vorrei mettere

    palletModel = ds.PalletizationModel(ds.Bin(1000.0, 1000.0, 1000.0), boxList)

    # posiziono lo scatele
    boxList[0].set_pos(0, 0, 0)
    boxList[1].set_pos(70, 0, 0)
    boxList[2].set_pos(140, 0, 0)
    boxList[3].set_pos(0, 70, 0)
    boxList[4].set_pos(70, 70, 0)
    boxList[5].set_pos(0, 70, 70)

    boxList = palletModel.order_box_set(boxList)
    # result = palletModel.two_dimensional_corners(boxList, J, palletModel.get_bin())
    result = palletModel.three_dimensional_corners(boxList, J, palletModel.get_bin())

    print('lala')
