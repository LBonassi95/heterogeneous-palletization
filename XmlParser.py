import xml.etree.ElementTree as ET
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
