from dynamixel_sdk import *

DEVICE   = '/dev/gripper_u2d2'
BAUD     = 1000000
ID       = 12
PROTOCOL = 2.0

ADDR_TORQUE_ENABLE    = 64
ADDR_GOAL_POSITION    = 116
ADDR_PRESENT_POSITION = 132

OPEN_POS  = 1754
CLOSE_POS = 594

port = PortHandler(DEVICE)
ph   = PacketHandler(PROTOCOL)
port.openPort()
port.setBaudRate(BAUD)

ph.write1ByteTxRx(port, ID, ADDR_TORQUE_ENABLE, 1)
print(f"Safe range: {CLOSE_POS} (closed) to {OPEN_POS} (open)")
print("Type a position, 'o' for open, 'c' for close, 'q' to quit\n")

while True:
    cmd = input("Position: ").strip()
    if cmd == 'q':
        break
    elif cmd == 'o':
        goal = OPEN_POS
    elif cmd == 'c':
        goal = CLOSE_POS
    else:
        try:
            goal = int(cmd)
        except:
            print("Invalid input")
            continue

    if not (CLOSE_POS <= goal <= OPEN_POS):
        print(f"Out of safe range! Stay between {CLOSE_POS} and {OPEN_POS}")
        continue

    ph.write4ByteTxRx(port, ID, ADDR_GOAL_POSITION, goal)
    pos, _, _ = ph.read4ByteTxRx(port, ID, ADDR_PRESENT_POSITION)
    print(f"  goal={goal}, actual={pos}")

ph.write1ByteTxRx(port, ID, ADDR_TORQUE_ENABLE, 0)
port.closePort()
print("Torque disabled, bye!")