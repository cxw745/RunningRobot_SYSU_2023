from Runner import Runner
from Selector import serial_select

from lyk_part.door import door_main
from lyk_part.bridge import bridge_main
import lwx.lwx_part as lwx
from golfBall import kickBall
import hwy_part.owo.hwy as hwy

states = ['RULER', 'HOLE', 'MINE',
          'DOOR', 'BRIDGE', 'BALL', 'STAIR', 'GATE', 'END']

runner = Runner(states, lwx.checkpoint_recognize)
# runner = Runner(states, serial_select)

runner.operations['RULER'] = hwy.ruler
runner.operations['HOLE'] = hwy.hole
runner.operations['MINE'] = hwy.mine
# runner.operations['BAFFLE'] = runner.idle # BAFFLE 集成到MINE的关卡里了
runner.operations['DOOR'] = door_main
runner.operations['BRIDGE'] = bridge_main
runner.operations['BALL'] = kickBall.kick_golf
runner.operations['STAIR'] = lwx.cross_stair
runner.operations['GATE'] = lwx.final_gate

runner.run()
