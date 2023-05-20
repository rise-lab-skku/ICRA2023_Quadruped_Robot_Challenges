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
['1200x2400_OSB', '1200x600_OSB', '12_00x1200_OSB_1', '12_00x1200_OSB', '1_1m_5x10cm_beam', '1_2m_5x10cm_beam_1', '1_2m_5x10cm_beam', '1', '2_4m_5x10cm_beam', '2', '300x600_OSB', '3', "42'_2x2_baluster", '450mm_hurdle_retainer', '4_1', '5x5x30_cm_wall_corner', '600x1200_OSB', '600x2400_OSB', '', 'Component_246', 'Component_247', 'Component_248', 'Component_249', 'Component_251', 'Component_272', 'Component_2', 'Component_363', 'Component_7', 'Crate_4', 'Diagonal_beam_5x10cm_1', 'fin_2', 'foam', 'Group_1595', 'Group_1596', 'Group_2258', 'Group_2261', 'Group_362', 'Group_368', 'Group_369', 'Group_36', 'Group_371', 'Group_38', 'Group_41', 'Group_73', 'Group_74', 'Group_75', 'Group_9', 'Half_Crate', 'Hingeplate', 'Hurdle_pipe', 'Hurdle_spacer', 'K_rail_rail', 'Obstructed1', 'Obstructed1A', 'Obstructed2', 'Obstructed2A', 'Obstructed3', 'Obstructed3A', 'Obstructed4', 'Obstructed4A', 'Pallet_leg', 'Ramp_side', 'Ramp_top', 'RampBrace', 'Slope_back', 'Slope_backplate', 'Slope_leg', 'Slope_post_1', 'Slope_post_2', 'Slope_sidewall', 'Sloper_brace', 'WtBkt_GrnRng_8in_4', 'WtBkt_GrnRng_8in_5']  #73

# read csv file line by line by ;
with open('object_list.txt','r') as csvfile:
    csvreader = csv.reader(csvfile, delimiter=';')
    for i, row in enumerate(csvreader):
        alias = row[0].split('.')
        id = 0
        name = alias[0].split('__')[0]
        #print(i,name)
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
print((object_ids, len(object_ids)))
