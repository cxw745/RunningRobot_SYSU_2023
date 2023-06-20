import hiwonder.ActionGroupControl as AGC
# import hiwonder.Board as Board

class Runner:
    stop = False
    state_cnt = -1

    def __init__(self, states, select):
        self.state = 'START'
        self.operations = {'START': self.idle}
        self.states = states
        self.select = select
        for s in self.states:
            self.operations[s] = self.idle
        self.operations['END'] = self.end

    def idle(self):
        print(f'{self.state}: Do Nothing')

    def end(self):
        print('END')
        self.stop = True

    def run(self):
        state_cnt = 1
        while not self.stop:
            self.state = self.select(state_cnt)
            if self.state is None:
                AGC.runActionGroup('sysu_slow_move')
                continue
            
            print(f'============== {self.state} ==============')
            if self.state == 'BRIDGE':
                self.operations[self.state](state_cnt)
            else:
                self.operations[self.state]()
            # a = input()
            state_cnt += 1


if __name__ == '__main__':
    states = ['START', 'RULER', 'HOLE', 'MINE', 'BAFFLE',
              'DOOR', 'BRIDGE', 'BALL', 'STAIR', 'GATE', 'END']
    from Selector import serial_select
    runner = Runner(states, serial_select)
    runner.operations['RULER'] = runner.idle
    runner.operations['HOLE'] = runner.idle
    runner.operations['MINE'] = runner.idle
    runner.operations['BAFFLE'] = runner.idle
    runner.operations['DOOR'] = runner.idle
    runner.operations['BRIDGE'] = runner.idle
    runner.operations['BALL'] = runner.idle
    runner.operations['STAIR'] = runner.idle
    runner.operations['GATE'] = runner.idle
    runner.run()
