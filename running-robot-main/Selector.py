def serial_select(state_cnt):
    states = ['START', 'RULER', 'HOLE', 'MINE',
            'DOOR', 'BRIDGE', 'BALL', 'STAIR', 'GATE', 'END']
    return states[state_cnt]