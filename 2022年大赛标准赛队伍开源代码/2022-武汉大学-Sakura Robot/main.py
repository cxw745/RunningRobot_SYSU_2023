#!/usr/bin/env python3
# coding:utf-8
import rospy
import time
import startdoor
import threading
import CMDcontrol
import square_hole
import obstacle
import overcome_obstacle
import door
import cross_the_bridge
import kickBallOnly
import Take_the_stairs
import end_door


###################################动作监听线程################################
def thread_move_action():
    CMDcontrol.CMD_transfer()

th2 = threading.Thread(target=thread_move_action)
th2.setDaemon(True)
th2.start()

##################################动作执行####################################
def action_append(act_name):
    print(f'执行动作: {act_name}')
    # time.sleep(1)
    CMDcontrol.action_append(act_name) 



def main():
    if startdoor.start_door(action_append):                 # 第一关，道闸
        print("道闸结束,进入下一关")

    if square_hole.main('green_hole_chest',action_append):  # 第二关，过坑，green_hole_chest为过坑地板颜色
        print("过坑结束，进入下一关")

    if obstacle.obstacle(action_append):                    # 第三关，地雷阵
        print("地雷阵结束，进入下一关")

    if overcome_obstacle.baffle(action_append):             # 第四关，挡板障碍
        print("翻越障碍结束，进入下一关")

    if door.into_the_door(action_append):                   # 第五关，过窄门框
        print("门框结束，进入下一关")

    if cross_the_bridge.Greenbridge(action_append):         # 第六关，过独木桥
        print("独木桥结束，进入下一关")

    if kickBallOnly.kick_ball(action_append):               # 第七关，踢球
        print("踢球结束，进入下一关")

    if Take_the_stairs.floor(action_append):                # 第八关，楼梯
        print("楼梯结束，进入下一关")

    if end_door.end_door(action_append):                    # 第九关，双开道闸
        print("结束")
    




if __name__ == '__main__':
    rospy.init_node('runningrobot')
    time.sleep(3)
    main()