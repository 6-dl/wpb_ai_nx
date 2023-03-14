"""yolo_classes.py

NOTE: Number of classes for WPB models.
"""
import os

class_list = []

def get_cls_dict():
    """Get the class name translation dictionary."""
    class_num =0
    home_path = os.path.expanduser('~')
    obj_path = home_path + "/model/objects.names"
    #obj_file = open("/home/robot/model/objects.names",mode='r')
    obj_file = open(obj_path,mode='r')
    for line in obj_file.readlines():
        line=line.strip('\n')
        class_list.append(line)
        class_num+=1
    print(class_list)
    return class_list,class_num
