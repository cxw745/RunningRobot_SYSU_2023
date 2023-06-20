#!/usr/bin/python3
# -*- coding: utf-8 -*-
import enum

# 关卡状态枚举
class State(enum.Enum):
    idle = enum.auto()
    start = enum.auto()
    debug = enum.auto()
    hole = enum.auto()
    mine = enum.auto()
    obstacle = enum.auto()
    door = enum.auto()
    bridge = enum.auto()
    ball = enum.auto()
    stair = enum.auto()
    end = enum.auto()


StateTrans_List = [
    # 0 --- 单关
    {
        State.start: State.idle,
        State.hole: State.idle,
        State.mine: State.idle,
        State.obstacle: State.idle,
        State.door: State.idle,
        State.bridge: State.idle,
        State.ball: State.idle,
        State.stair: State.idle,
        State.end: State.idle,
    },
    # 1 --- 洞-桥-梯
    {
        State.start: State.hole,
        State.hole: State.mine,
        State.mine: State.obstacle,
        State.obstacle: State.door,
        State.door: State.bridge,
        State.bridge: State.ball,
        State.ball: State.stair,
        State.stair: State.end,
        State.end: State.idle,
    },
    # 2 --- 桥-洞-梯
    {
        State.start: State.bridge,
        State.bridge: State.mine,
        State.mine: State.obstacle,
        State.obstacle: State.door,
        State.door: State.hole,
        State.hole: State.ball,
        State.ball: State.stair,
        State.stair: State.end,
        State.end: State.idle,
    },
    # 3 --- 梯-桥-洞
    {
        State.start: State.stair,
        State.stair: State.mine,
        State.mine: State.obstacle,
        State.obstacle: State.door,
        State.door: State.bridge,
        State.bridge: State.ball,
        State.ball: State.hole,
        State.hole: State.end,
        State.end: State.idle,
    },
    # 4 --- 桥-梯-洞
    {
        State.start: State.bridge,
        State.bridge: State.mine,
        State.mine: State.obstacle,
        State.obstacle: State.door,
        State.door: State.stair,
        State.stair: State.ball,
        State.ball: State.hole,
        State.hole: State.end,
        State.end: State.idle,
    },
    # 5 --- 梯-洞-桥
    {
        State.start: State.stair,
        State.stair: State.mine,
        State.mine: State.obstacle,
        State.obstacle: State.door,
        State.door: State.hole,
        State.hole: State.ball,
        State.ball: State.bridge,
        State.bridge: State.end,
        State.end: State.idle,
    },
    # 6 --- 洞-梯-桥
    {
        State.start: State.hole,
        State.hole: State.mine,
        State.mine: State.obstacle,
        State.obstacle: State.door,
        State.door: State.stair,
        State.stair: State.ball,
        State.ball: State.bridge,
        State.bridge: State.end,
        State.end: State.idle,
    },
]

