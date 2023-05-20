#!/usr/bin/env python
import csv

class Obstacle():
    def __init__(self, name, id, size, pos, quat, euler) -> None:
        self.name = name
        self.id = id
        self.size = size
        self.pos = pos
        self.quat = quat
        self.euler = euler

object_instances = []
object_ids = []

# read csv file line by line by ;
with open('object_list.txt','r') as csvfile:
    csvreader = csv.reader(csvfile, delimiter=';')
    for row in csvreader:
        alias = row[0].split('.')
        id = 0
        name = alias[0]
        if len(alias) > 1:
            id = int(alias[1])
    
        size = [float(row[1]), float(row[2]), float(row[3])]
        pos = [float(row[4]), float(row[5]), float(row[6])]
        quat = [float(row[7]), float(row[8]), float(row[9]), float(row[10])]
        euler = [float(row[11]), float(row[12]), float(row[13])]
        object_instances.append(Obstacle(name, id, size, pos, quat, euler))
        object_ids.append(name)

# remove duplicate name in list
object_ids = list(dict.fromkeys(object_ids))
print(len(object_ids))
