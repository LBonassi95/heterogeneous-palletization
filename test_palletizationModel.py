from unittest import TestCase
import Data_Structures as ds
import random


def get_random_model(num_of_boxes):
    random.seed(47)
    boxList = [ds.Box(float(random.randint(1, 10)), float(random.randint(1, 10)), float(random.randint(1, 10))) for i in range(num_of_boxes)]
    return ds.PalletizationModel(ds.Bin(10, 10, 10), boxList)


class TestPalletizationModel(TestCase):

    def test_get_l1_bound(self):
        boxlist = [ds.Box(6, 6, 6), ds.Box(6, 6, 6), ds.Box(6, 6, 6)]
        bin = ds.Bin(10, 10, 10)
        model = ds.PalletizationModel(bin, boxlist)
        self.assertEqual(3, model.get_l1_bound())
        model = get_random_model(100)
        self.assertEqual(24, model.get_l1_bound())

    def test_get_l1_w_h(self):
        ############################################################
        boxlist = [ds.Box(6, 6, 7), ds.Box(6, 6, 5), ds.Box(6, 6, 3)]
        bin = ds.Bin(10, 10, 10)
        p = 3
        model = ds.PalletizationModel(bin, boxlist)
        value_list = [[box.width, box.height, box.depth] for box in boxlist]
        self.assertEqual(2, model.get_l1_p(p, value_list, bin.width, bin.height, bin.depth))
        ############################################################
        boxlist = [ds.Box(6, 6, 7), ds.Box(6, 6, 5), ds.Box(6, 6, 2)]
        bin = ds.Bin(10, 10, 10)
        p = 3
        model = ds.PalletizationModel(bin, boxlist)
        value_list = [[box.width, box.height, box.depth] for box in boxlist]
        self.assertEqual(2, model.get_l1_p(p, value_list, bin.width, bin.height, bin.depth))
        ############################################################
        boxlist = [ds.Box(6, 6, 5), ds.Box(6, 6, 5), ds.Box(6, 6, 2)]
        bin = ds.Bin(10, 10, 10)
        p = 3
        model = ds.PalletizationModel(bin, boxlist)
        value_list = [[box.width, box.height, box.depth] for box in boxlist]
        self.assertEqual(1, model.get_l1_p(p, value_list, bin.width, bin.height, bin.depth))
        ############################################################
        boxlist = []
        bin = ds.Bin(10, 10, 10)
        p = 3
        model = ds.PalletizationModel(bin, boxlist)
        value_list = [[box.width, box.height, box.depth] for box in boxlist]
        self.assertEqual(0, model.get_l1_p(p, value_list, bin.width, bin.height, bin.depth))
        ############################################################
        boxlist = [ds.Box(6, 6, 5), ds.Box(6, 6, 5), ds.Box(6, 6, 2)]
        bin = ds.Bin(10, 10, 10)
        p = 5
        model = ds.PalletizationModel(bin, boxlist)
        value_list = [[box.width, box.height, box.depth] for box in boxlist]
        self.assertEqual(1, model.get_l1_p(p, value_list, bin.width, bin.height, bin.depth))

    def test_get_l1_w_d(self):
        ############################################################
        boxlist = [ds.Box(6, 7, 6), ds.Box(6, 5, 6), ds.Box(6, 3, 6)]
        bin = ds.Bin(10, 10, 10)
        p = 3
        model = ds.PalletizationModel(bin, boxlist)
        value_list = [[box.width, box.depth, box.height] for box in boxlist]
        self.assertEqual(2, model.get_l1_p(p, value_list, bin.width, bin.depth, bin.height))

    def test_get_l1_h_d(self):
        ############################################################
        boxlist = [ds.Box(7, 6, 6), ds.Box(5, 6, 6), ds.Box(3, 6, 6)]
        bin = ds.Bin(10, 10, 10)
        p = 3
        model = ds.PalletizationModel(bin, boxlist)
        value_list = [[box.height, box.depth, box.width] for box in boxlist]
        self.assertEqual(2, model.get_l1_p(p, value_list, bin.height, bin.depth, bin.width))

    def test_get_l2_bound(self):
        model = get_random_model(100)
        self.assertEqual(26, model.get_l2_bound())

    def test_get_l2_w_h(self):
        boxlist = [ds.Box(7, 6, 6), ds.Box(5, 6, 6), ds.Box(3, 6, 6)]
        bin = ds.Bin(6, 7, 8)
        p = 2
        q = 3
        model = ds.PalletizationModel(bin, boxlist)
        self.assertEqual(2, model.get_l2_p_q(p,
                                             q,
                                             model.list_w_h,
                                             model.bin.width,
                                             model.bin.height,
                                             model.bin.depth,
                                             model.l1_w_h))

    def test_get_l2_w_d(self):
        boxlist = [ds.Box(7, 6, 6), ds.Box(5, 6, 6), ds.Box(3, 6, 6)]
        bin = ds.Bin(6, 7, 8)
        p = 2
        q = 3
        model = ds.PalletizationModel(bin, boxlist)
        self.assertEqual(3, model.get_l2_p_q(p,
                                             q,
                                             model.list_w_h,
                                             model.bin.width,
                                             model.bin.depth,
                                             model.bin.height,
                                             model.l1_w_d))

    def test_get_l2_h_d(self):
        boxlist = [ds.Box(7, 6, 6), ds.Box(5, 6, 6), ds.Box(3, 6, 6)]
        bin = ds.Bin(6, 7, 8)
        p = 2
        q = 3
        model = ds.PalletizationModel(bin, boxlist)
        self.assertEqual(3, model.get_l2_p_q(p,
                                             q,
                                             model.list_w_h,
                                             model.bin.height,
                                             model.bin.depth,
                                             model.bin.width,
                                             model.l1_h_d))

    # def test_two_dimensional_corners(self):
    #     self.fail()
    #
    # def test_three_dimensional_corners(self):
    #     self.fail()
