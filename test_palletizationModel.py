# coding=utf-8
from unittest import TestCase
import Data_Structures as ds
import random


def get_random_model(num_of_boxes):
    random.seed(47)
    boxList = [ds.Box(float(random.randint(1, 10)), float(random.randint(1, 10)), float(random.randint(1, 10))) for i in range(num_of_boxes)]
    return ds.PalletizationModel(ds.Bin(10.0, 10.0, 10.0), boxList)


class TestPalletizationModel(TestCase):

    def test_2d_ordering(self):
        boxList = [ds.Box(70, 70, 70) for i in range(5)]  # scatole già posizionate
        boxList.append(ds.Box(100, 70, 23))
        J = [ds.Box(1, 2, 1) for i in range(3)]  # scatole che vorrei mettere

        palletModel = ds.SingleBinProblem(ds.Bin(1000.0, 1000.0, 1000.0))
        palletModel.boxList = boxList
        # posiziono lo scatele
        boxList[0].set_pos(0, 0, 0)
        boxList[1].set_pos(70, 0, 0)
        boxList[2].set_pos(140, 0, 0)
        boxList[3].set_pos(0, 70, 0)
        boxList[4].set_pos(70, 70, 0)
        boxList[5].set_pos(0, 70, 70)

        boxList = palletModel.order_box_set(boxList)
        result = palletModel.three_dimensional_corners(boxList, J)

        # da mettere il controllo sui punti

    def test_3d_ordering(self):
        boxList = [ds.Box(70, 70, 70) for i in range(5)]  # scatole già posizionate
        boxList.append(ds.Box(100, 70, 23))
        J = [ds.Box(1, 2, 1) for i in range(3)]  # scatole che vorrei mettere

        palletModel = ds.SingleBinProblem(ds.Bin(1000.0, 1000.0, 1000.0))
        palletModel.boxList = boxList
        # posiziono lo scatele
        boxList[0].set_pos(0, 0, 0)
        boxList[1].set_pos(70, 0, 0)
        boxList[2].set_pos(140, 0, 0)
        boxList[3].set_pos(0, 70, 0)
        boxList[4].set_pos(70, 70, 0)
        boxList[5].set_pos(0, 70, 70)

        boxList = palletModel.order_box_set(boxList)
        result = palletModel.three_dimensional_corners(boxList, J)
        print('ciao')
        # da metttere il controllo sui punti


    def test_get_l1_bound(self):
        boxlist = [ds.Box(6.0, 6.0, 6.0), ds.Box(6.0, 6.0, 6.0), ds.Box(6.0, 6.0, 6.0)]
        bin = ds.Bin(10.0, 10.0, 10.0)
        model = ds.PalletizationModel(bin, boxlist)
        self.assertEqual(3, model.get_l1_bound())
        model = get_random_model(100)
        self.assertEqual(19, model.get_l1_bound())

    def test_get_l1_w_h(self):
        ############################################################
        boxlist = [ds.Box(6, 6, 7), ds.Box(6, 6, 5), ds.Box(6, 6, 3)]
        bin = ds.Bin(10.0, 10.0, 10.0)
        p = 3
        model = ds.PalletizationModel(bin, boxlist)
        value_list = [[box.width, box.height, box.depth] for box in boxlist]
        self.assertEqual(2, model.get_l1_p(p, value_list, bin.width, bin.height, bin.depth))
        ############################################################
        boxlist = [ds.Box(6, 6, 7), ds.Box(6, 6, 5), ds.Box(6, 6, 2)]
        bin = ds.Bin(10.0, 10.0, 10.0)
        p = 3
        model = ds.PalletizationModel(bin, boxlist)
        value_list = [[box.width, box.height, box.depth] for box in boxlist]
        self.assertEqual(2, model.get_l1_p(p, value_list, bin.width, bin.height, bin.depth))
        ############################################################
        boxlist = [ds.Box(6, 6, 5), ds.Box(6, 6, 5), ds.Box(6, 6, 2)]
        bin = ds.Bin(10.0, 10.0, 10.0)
        p = 3
        model = ds.PalletizationModel(bin, boxlist)
        value_list = [[box.width, box.height, box.depth] for box in boxlist]
        self.assertEqual(1, model.get_l1_p(p, value_list, bin.width, bin.height, bin.depth))
        ############################################################
        boxlist = []
        bin = ds.Bin(10.0, 10.0, 10.0)
        p = 3
        model = ds.PalletizationModel(bin, boxlist)
        value_list = [[box.width, box.height, box.depth] for box in boxlist]
        self.assertEqual(0, model.get_l1_p(p, value_list, bin.width, bin.height, bin.depth))
        ############################################################
        boxlist = [ds.Box(6, 6, 5), ds.Box(6, 6, 5), ds.Box(6, 6, 2)]
        bin = ds.Bin(10.0, 10.0, 10.0)
        p = 5
        model = ds.PalletizationModel(bin, boxlist)
        value_list = [[box.width, box.height, box.depth] for box in boxlist]
        self.assertEqual(1, model.get_l1_p(p, value_list, bin.width, bin.height, bin.depth))

    def test_get_l1_w_d(self):
        ############################################################
        boxlist = [ds.Box(6, 7, 6), ds.Box(6, 5, 6), ds.Box(6, 3, 6)]
        bin = ds.Bin(10.0, 10.0, 10.0)
        p = 3
        model = ds.PalletizationModel(bin, boxlist)
        value_list = [[box.width, box.depth, box.height] for box in boxlist]
        self.assertEqual(2, model.get_l1_p(p, value_list, bin.width, bin.depth, bin.height))

    def test_get_l1_h_d(self):
        ############################################################
        boxlist = [ds.Box(7, 6, 6), ds.Box(5, 6, 6), ds.Box(3, 6, 6)]
        bin = ds.Bin(10.0, 10.0, 10.0)
        p = 3
        model = ds.PalletizationModel(bin, boxlist)
        value_list = [[box.height, box.depth, box.width] for box in boxlist]
        self.assertEqual(2, model.get_l1_p(p, value_list, bin.height, bin.depth, bin.width))

    def test_get_l2_bound(self):
        model = get_random_model(100)
        self.assertEqual(21, model.get_l2_bound())

    def test_get_l2_w_h(self):
        boxlist = [ds.Box(7, 6, 6), ds.Box(5, 6, 6), ds.Box(3, 6, 6)]
        bin = ds.Bin(6.0, 7.0, 8.0)
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
        boxlist = [ds.Box(7.0, 6.0, 6.0), ds.Box(5.0, 6.0, 6.0), ds.Box(3.0, 6.0, 6.0)]
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
        boxlist = [ds.Box(7.0, 6.0, 6.0), ds.Box(5.0, 6.0, 6.0), ds.Box(3.0, 6.0, 6.0)]
        bin = ds.Bin(6.0, 7.0, 8.0)
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

    def test_single_bin_filling(self):
        single_bin = ds.SingleBinProblem(ds.Bin(1000.0, 1000.0, 1000.0))
        boxList = [ds.Box(500.0, 500.0, 500.0) for i in range(7)]
        single_bin.boxList = boxList
        res, _ = single_bin.fillBin()
        self.assertEqual(res, True)

        boxList.append(ds.Box(500.0, 500.0, 500.0))
        res, _ = single_bin.fillBin()
        self.assertEqual(res, True)

        boxList.append(ds.Box(500.0, 500.0, 500.0))
        res, _ = single_bin.fillBin()
        self.assertEqual(res, False)

        boxList = [ds.Box(100.0, 200.0, 300.0) for i in range(150)]
        single_bin.boxList = boxList
        res, boxList = single_bin.fillBin()
        self.assertEqual(res, True)

    def test_below_boxes(self):
        box1 = ds.Box(4.0,5.0,3.0)
        box1.set_pos(0, 0, 0)

        box2 = ds.Box(4.0, 5.0, 2.0)
        box2.set_pos(2.0, 5.0, 2.0)
        self.assertEqual(len(ds.getBoxesBelow(box2, [box1])), 1)

        box3 = ds.Box(4.0, 5.0, 2.0)
        box3.set_pos(4.0, 5.0, 3.0)
        self.assertEqual(len(ds.getBoxesBelow(box3, [box1])), 0)

        box4 = ds.Box(4.0, 5.0, 2.0)
        box4.set_pos(3.99, 5.0, 2.99)
        self.assertEqual(len(ds.getBoxesBelow(box4, [box1])), 1)

        box5 = ds.Box(3.0, 5.0, 2.0)
        box5.set_pos(1.0, 5.0, 1.0)
        self.assertEqual(len(ds.getBoxesBelow(box5, [box1])), 1)

        box6 = ds.Box(1.0, 5.0, 1.0)
        box6.set_pos(0.0, 0, 0.0)

        box7 = ds.Box(1.0, 5.0, 1.0)
        box7.set_pos(1.0, 0, 1.0)

        box8 = ds.Box(1.0, 5.0, 1.0)
        box8.set_pos(2.0, 0, 2.0)

        box9 = ds.Box(1.0, 5.0, 1.0)
        box9.set_pos(3.0, 0, 3.0)

        box0 = ds.Box(1.0, 5.0, 1.0)
        box0.set_pos(4.0, 4, 4.0)

        box_sopra = ds.Box(6.0, 5.0, 6.0)
        box_sopra.set_pos(0, 5, 0)

        self.assertEqual(len(ds.getBoxesBelow(box_sopra, [box6, box7, box8, box9, box0])), 4)

    def test_weighted_single_bin_filling(self):
        box6 = ds.Box(1.0, 5.0, 1.0)
        box6.set_pos(0.0, 0, 0.0)

        box7 = ds.Box(1.0, 5.0, 1.0)
        box7.set_pos(1.0, 0, 1.0)

        box8 = ds.Box(1.0, 5.0, 1.0)
        box8.set_pos(2.0, 0, 2.0)

        box9 = ds.Box(1.0, 5.0, 1.0)
        box9.set_pos(3.0, 0, 3.0)

        box0 = ds.Box(1.0, 5.0, 1.0)
        box0.set_pos(4.0, 0, 4.0)

        box_sopra = ds.Box(6.0, 5.0, 6.0)
        box_sopra.set_pos(0, 5, 0)
        box_sopra.set_weight(ds.DEFAULT_MAX_WEIGHT-5)

        box_sopra_sopra = ds.Box(6.0, 5.0, 6.0)
        box_sopra_sopra.set_pos(0, 10, 0)
        box_sopra_sopra.set_weight(ds.DEFAULT_MAX_WEIGHT )

        single_bin = ds.SingleBinProblem(ds.Bin(6, 15, 6))
        boxList = [box6, box7, box8, box9, box0]
        single_bin.withWeight = True

        self.assertEqual(True, single_bin.branch_and_bound_filling(boxList, [box_sopra]))

        boxList.append(box_sopra)
        self.assertEqual(False, single_bin.branch_and_bound_filling(boxList, [box_sopra_sopra]))
