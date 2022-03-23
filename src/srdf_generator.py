# Source:
# https://github.com/servetb/robowflex/blob/master/robowflex_dart/include/io/create_srdf.py
# Created by serboba at 20.03.22

from lxml import etree as ET
import sys

def parse_file(root_name):
    groups = []

    root = ET.parse(root_name).getroot()
    i = 0
    temp = 0
    group = []
    robot_name = root.get("name")
    print(robot_name)
    for joint in root.findall('joint'):
        if(joint.get("type") != "fixed"):
            gr_index = int(joint.get("name").split('_')[1])
            if(gr_index> temp):
                groups.append(group)
                group = []
                group.append(joint.get("name"))
                temp = gr_index
            else:
                group.append(joint.get("name"))
    if(group != []):
        groups.append(group)

    root = ET.Element("robot")
    root.set("name",robot_name)

    for i in range (len(groups)):
        gr_name = "link_"+str(i)+"_gr"
        group = ET.SubElement(root,"group")
        group.set("name",gr_name)
        for elem in groups[i]:
            ET.SubElement(group,"joint",name =elem).text = ""

    tree = ET.ElementTree(root)

    #srdf_name = root_name.split('/')[0]+'/'+robot_name+".srdf"
    srdf_name = "puzzles/" + robot_name + "/srdf/" + robot_name +".srdf" # todo change output file path, je nach dem wo file geschrieben werden soll
    tree.write(srdf_name,pretty_print=True)


# os.chdir("../../..")
# os.chdir(os.getcwd()+"/src/robowflex/robowflex_dart/include/io/")
#
if len(sys.argv) == 1:
     sys.exit("NOT ENOUGH ARGS")

parse_file(str(sys.argv[1]))

#
# def main():
#     parse_file("/home/serboba/puzzle-generator/puzzles/simple_sliders/urdf/simple_sliders.urdf")
#
# if __name__ == "__main__":
#     main()
