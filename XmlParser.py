import xml.etree.ElementTree as ET
from docutils.utils.math.latex2mathml import mi

import Data_Structures as ds


def xml2boxlist(xml_path):
    root = ET.parse(xml_path).getroot()
    boxlist = []

    for box_tag in root.findall("Box"):
        box_type = box_tag.find("Type").text

        if box_type == 'Box':
            width = float(box_tag.find("Width").text)
            height = float(box_tag.find("Height").text)
            depth = float(box_tag.find("Depth").text)
            item = box_tag.find("ItemName").text
            qty = int(box_tag.find("Quantity").text)
            maxWeigth = float(box_tag.find("MaxWeight").text)
            weight = float(box_tag.find("Weight").text)

            for i in range(qty):
                box = ds.Box(width, height, depth)
                box.itemName = item
                box.maximumWeight = maxWeigth
                box.weight = weight
                boxlist.append(box)

        elif box_type == 'Cylinder':
            radius = float(box_tag.find("Radius").text)
            height = float(box_tag.find("Height").text)
            item = box_tag.find("ItemName").text
            qty = int(box_tag.find("Quantity").text)
            maxWeigth = float(box_tag.find("MaxWeight").text)
            weight = float(box_tag.find("Weight").text)

            for i in range(qty):
                box = ds.Box(2*radius, height, 2*radius)  # constructor return the smallest box that contains the cylinder
                box.itemName = item
                box.maximumWeight = maxWeigth
                box.weight = weight
                boxlist.append(box)

    return boxlist



# AGGIUNTO ORA
def xml2problem(xml_path):
    root = ET.parse(xml_path).getroot()
    boxlist = []

    minDict = {}
    maxDict = {}

    for box_tag in root.findall("Box"):
        box_type = box_tag.find("Type").text

        if box_type == 'Box':
            width = float(box_tag.find("Width").text)
            height = float(box_tag.find("Height").text)
            depth = float(box_tag.find("Depth").text)
            item = box_tag.find("ItemName").text
            qty = int(box_tag.find("Quantity").text)
            maxWeigth = float(box_tag.find("MaxWeight").text)
            weight = float(box_tag.find("Weight").text)

            minQuantity = box_tag.find("MinQuantity")
            if minQuantity is not None:
                minDict[item] = int(minQuantity.text)

            maxQuantity = box_tag.find("MaxQuantity")
            if minQuantity is not None:
                maxDict[item] = int(maxQuantity.text)

            for i in range(qty):
                box = ds.Box(width, height, depth)
                box.itemName = item
                box.maximumWeight = maxWeigth
                box.weight = weight
                boxlist.append(box)

        elif box_type == 'Cylinder':
            radius = float(box_tag.find("Radius").text)
            height = float(box_tag.find("Height").text)
            item = box_tag.find("ItemName").text
            qty = int(box_tag.find("Quantity").text)
            maxWeigth = float(box_tag.find("MaxWeight").text)
            weight = float(box_tag.find("Weight").text)

            minQuantity = box_tag.find("MinQuantity")
            if minQuantity is not None:
                minDict[item] = int(minQuantity.text)

            maxQuantity = box_tag.find("MaxQuantity")
            if minQuantity is not None:
                maxDict[item] = int(maxQuantity.text)

            for i in range(qty):
                box = ds.Box(2*radius, height, 2*radius)  # constructor return the smallest box that contains the cylinder
                box.itemName = item
                box.maximumWeight = maxWeigth
                box.weight = weight
                boxlist.append(box)

    bin = root.findall("Bin")
    if len(bin) > 0:
        bin_width = int(bin[0].find("Width").text)
        bin_height = int(bin[0].find("Height").text)
        bin_depth = int(bin[0].find("Depth").text)
        bin_maxWeight = float(bin[0].find("MaximumWeight").text)
    else:
        print('Error, the bin structure is not defined!!!')
        return None

    return ds.PalletizationModel(ds.Bin(bin_width, bin_height, bin_depth, bin_maxWeight),
                                 boxlist,
                                 maxDict=maxDict,
                                 minDict=minDict)
