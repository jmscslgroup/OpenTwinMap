import xml.etree.ElementTree as ET
from OpenDrive import osm_to_opendrive
from tdot_subset import TDOTSubset

def indent(elem, level=0, indent_size="\t"):
    i = "\n" + level * indent_size
    if len(elem):
        # If element has children:
        if not (elem.text and elem.text.strip()):
            elem.text = i + indent_size  # child block starts on next line

        for child in elem:
            indent(child, level + 1, indent_size)
            # Each child (except possibly the last) ends with a newline + one extra indent
            if not (child.tail and child.tail.strip()):
                child.tail = i + indent_size

        # Fix the last child's tail so the parent's closing tag aligns with the parent
        if not (elem[-1].tail and elem[-1].tail.strip()):
            elem[-1].tail = i
    else:
        # No children: place this element's closing tag at the current level
        if level and not (elem.tail and elem.tail.strip()):
            elem.tail = i

map_data = TDOTSubset("./SubsetSelection/")
converter = osm_to_opendrive.OSMToOpenDrive(map_data)
converter.readOSM()
xml_data = converter.convertToOpenDrive().toXML()
indent(xml_data)
xml_data_root = ET.ElementTree(xml_data)
xml_data_root.write(map_data.getXODRPath(), encoding="utf-8", xml_declaration=True)
