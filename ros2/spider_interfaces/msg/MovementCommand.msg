# 状态机输出：状态与方向使用固定枚举，避免下游重新解释原始按键。
uint8 STANDING=0
uint8 MOVING=1

uint8 STOPPED=0
uint8 FORWARD=1
uint8 BACKWARD=2
uint8 LEFT=3
uint8 RIGHT=4
uint8 FORWARD_LEFT=5
uint8 FORWARD_RIGHT=6
uint8 BACKWARD_LEFT=7
uint8 BACKWARD_RIGHT=8

uint8 state
uint8 direction
