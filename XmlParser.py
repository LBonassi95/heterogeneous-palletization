import xml.etree.ElementTree as ET
import Data_Structures as ds


def xml2boxlist(xml_path):
    root = ET.parse(xml_path).getroot()
    boxlist = []
    for box_tag in root.findall("Box"):
        type = box_tag.find("Type").text
        width = float(box_tag.find("Width").text)
        height = float(box_tag.find("Height").text)
        depth = float(box_tag.find("Depth").text)
        item = box_tag.find("ItemName").text
        qty = int(box_tag.find("Quantity").text)
        maxweigth = float(box_tag.find("MaxWeight").text)
        weight = float(box_tag.find("Weight").text)
        for i in range(qty):
            box = ds.Box(width, height, depth)
            box.itemName = item
            box.maximumWeight = maxweigth
            box.weight = weight
            boxlist.append(box)
    return boxlist

