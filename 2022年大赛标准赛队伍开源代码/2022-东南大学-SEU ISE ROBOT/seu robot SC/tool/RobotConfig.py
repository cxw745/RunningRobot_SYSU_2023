# ################1：起始门关参数#################：
color_range1 = {
    'yellow_door': [(24, 150, 95), (37, 255, 255)],  # ok
    'black_door': [(0, 0, 11), (188, 174, 31)]
}

# ################2：坑洞关关参数#################：
color_range2 = {
    #'head_green': [(52, 59, 62), (76, 250, 255)],  # ok
    'head_green': [(52, 104, 62), (76, 250, 228)],  # ok
}

# ################3：地雷关参数#################：
color_range3 = {
    'blue_obstacle_chest': [(107, 123, 96), (130, 255, 255)],
    #'blue_obstacle_head': [(95, 101, 129), (135, 255, 255)],
    'blue_obstacle_head': [(106, 111, 52), (131, 255, 255)],
    'chest_black_louxia': [(14, 10, 55), (255, 51, 92)],  # ok
    'white_chest': [(0 , 0 , 166), (150 , 15 , 255)] ,
}

# ################4：翻墙关参数#################：
color_range4 = {
    'blue_door_chest': [(108, 123, 96), (130, 255, 255)],
    #'blue_door_fanguang': [(56, 107, 109), (87, 251, 255)],
    'head_hongzhuan':  [(0, 47, 47), (14, 232, 232)]
}

# ################5：门框关参数#################：
color_range5 = {
    #'blue_door_chest': [(95, 123, 96), (130, 255, 255)],
    #'blue_door_chest': [(95, 123, 67), (130, 255, 255)],
    'blue_door_chest': [(95, 77, 53), (130, 255, 255)],
    #'blue_door_fanguang': [(56, 107, 109), (87, 251, 255)],
    'chest_red_hongzhuan': [(0 , 47 , 47), (14 , 232 , 232)],
    'blue_door_chest_dafanwei': [(98, 111, 52), (131, 255, 255)]
}

# ################6：独木桥关参数#################：
color_range6 = {
    'head_green_bridge': [(52, 104, 62), (76, 250, 228)],
    'chest_green_bridge': [(56, 91, 76), (89, 236, 255)],
}

# ################8：踢球｜上楼梯关参数#################：
color_range8 = {
    'head_blue_stair': [(95, 123, 96), (110, 255, 255)],
    'head_green_stair': [(52, 104, 62), (76, 250, 228)],
    # 'head_red_stair': [(0, 127, 170), (10, 228, 197)],
    # 'chest_green_stair': [(56, 91, 76), (89, 236, 255)],
}

# ################9：下楼梯关参数#################：
color_range9 = {
    # 'head_red': [(50, 70, 38), (79, 200, 147)],
    #'chest_red': [(0, 127, 57), (10, 228, 197)]
    'chest_red': [(0, 115, 71), (16, 251, 255)]
}

# ##################总颜色卡####################：
color_rangen = {}
color_rangen.update(color_range1)
color_rangen.update(color_range2)
color_rangen.update(color_range3)
color_rangen.update(color_range4)
color_rangen.update(color_range5)
color_rangen.update(color_range6)
color_rangen.update(color_range8)
color_rangen.update(color_range9)

color_range = color_rangen

