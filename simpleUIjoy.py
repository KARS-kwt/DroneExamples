import time
import curses
from collections import deque
from itertools import cycle
import serial.tools.list_ports
import os
import pygame
from yamspy import MSPy
from pygame.locals import MOUSEMOTION, MOUSEBUTTONUP, MOUSEBUTTONDOWN

CTRL_LOOP_TIME = 1 / 100
SLOW_MSGS_LOOP_TIME = 1 / 5
NO_OF_CYCLES_AVERAGE_GUI_TIME = 10

# --- Initialize pygame and joystick
pygame.init()
pygame.event.set_blocked((MOUSEMOTION, MOUSEBUTTONUP, MOUSEBUTTONDOWN))
pygame.joystick.init()
if pygame.joystick.get_count() == 0:
    raise RuntimeError("No joystick detected.")
joy = pygame.joystick.Joystick(0)
joy.init()

axis = []
button = []

def update():
    pygame.event.pump()
    global axis, button
    axis = [joy.get_axis(i) for i in range(joy.get_numaxes())]
    button = [joy.get_button(i) for i in range(joy.get_numbuttons())]

# --- Serial port detection
def detect_serial_port():
    ports = serial.tools.list_ports.comports()
    for port in ports:
        if "ACM" in port.device or "USB" in port.device:
            return port.device
    return "/dev/ttyACM0"  # fallback

SERIAL_PORT = detect_serial_port()


# --- Curses wrapper
def run_curses(external_function):
    result = 1
    try:
        screen = curses.initscr()
        curses.noecho()
        curses.cbreak()
        screen.timeout(0)
        screen.keypad(True)

        screen.addstr(1, 0, "Joystick mode: 'A' to arm, 'B' to disarm, 'X' to switch mode, hold R2 to reboot, 'Y' for failsafe", curses.A_BOLD)
        result = external_function(screen)

    finally:
        curses.nocbreak(); screen.keypad(0); curses.echo()
        curses.endwin()
        if result == 1:
            print("An error occurred... probably the serial port is not available ;)")

# --- Main joystick control loop
def joy_controller(screen):
    CMDS = {
        'roll': 1500,
        'pitch': 1500,
        'throttle': 900,
        'yaw': 1500,
        'aux1': 1000,  # arm
        'aux2': 1000   # mode
    }
    CMDS_ORDER = ['roll', 'pitch', 'throttle', 'yaw', 'aux1', 'aux2']

    try:
        screen.addstr(15, 0, f"Connecting to the FC on {SERIAL_PORT}...")
        with MSPy(device=SERIAL_PORT, loglevel='WARNING', baudrate=115200) as board:
            if board == 1:
                return 1

            if not hasattr(board, 'INAV'):
                board.INAV = False

            screen.addstr(15, 0, f"Connected to FC on {SERIAL_PORT}")
            screen.clrtoeol()
            screen.move(1, 0)

            average_cycle = deque([0]*NO_OF_CYCLES_AVERAGE_GUI_TIME)

            command_list = [
                'MSP_API_VERSION', 'MSP_FC_VARIANT', 'MSP_FC_VERSION', 'MSP_BUILD_INFO',
                'MSP_BOARD_INFO', 'MSP_UID', 'MSP_ACC_TRIM', 'MSP_NAME',
                'MSP_STATUS', 'MSP_STATUS_EX', 'MSP_BATTERY_CONFIG',
                'MSP_BATTERY_STATE', 'MSP_BOXNAMES'
            ]
            if board.INAV:
                command_list += ['MSPV2_INAV_ANALOG', 'MSP_VOLTAGE_METER_CONFIG']

            for msg in command_list:
                if board.send_RAW_msg(MSPy.MSPCodes[msg], data=[]):
                    dataHandler = board.receive_msg()
                    board.process_recv_data(dataHandler)

            cellCount = board.BATTERY_STATE.get('cellCount', 3)
            min_voltage = board.BATTERY_CONFIG.get('vbatmincellvoltage', 3.5) * cellCount
            warn_voltage = board.BATTERY_CONFIG.get('vbatwarningcellvoltage', 3.6) * cellCount
            max_voltage = board.BATTERY_CONFIG.get('vbatmaxcellvoltage', 4.4) * cellCount

            slow_msgs = cycle(['MSP_ANALOG', 'MSP_STATUS_EX', 'MSP_MOTOR', 'MSP_RC'])
            cursor_msg = ""
            last_loop_time = last_slow_msg_time = last_cycleTime = time.time()

            screen.addstr(2, 0, f"Joystick: {joy.get_name()}")

            # Delay for joystick to stabilize
            time.sleep(0.2)
            update()

            while True:
                start_time = time.time()
                char = screen.getch()
                curses.flushinp()

                try:
                    update()
                except Exception as e:
                    screen.addstr(3, 0, f"Joystick lost: {e}. Sending failsafe...")
                    CMDS['aux2'] = 1800
                    continue

                def scale_axis(val): return int(1500 + max(-100, min(100, int(val * 100))))
                def scale_throttle(val): return int(900 + max(-800, min(800, int(val * 800))))

                # Joystick axes
                raw_throttle = scale_throttle(-axis[1])
                raw_yaw = 1500
                raw_roll = scale_axis(axis[2])
                raw_pitch = scale_axis(-axis[3])

                # Apply ramping
                CMDS['throttle'] = (raw_throttle)
                CMDS['yaw'] = (raw_yaw)
                CMDS['roll'] = (raw_roll)
                CMDS['pitch'] = (raw_pitch)

                # Button mappings
                if len(axis) > 4 and axis[4] > 0.9:
                    board.reboot()
                    time.sleep(0.2)
                    break
                elif len(button) > 0 and button[0]:
                    cursor_msg = 'Sending Arm command...'
                    CMDS['aux1'] = 1800
                elif len(button) > 1 and button[1]:
                    cursor_msg = 'Sending Disarm command...'
                    CMDS['aux1'] = 1000
                elif len(button) > 2 and button[2]:
                    if CMDS['aux2'] <= 1300:
                        CMDS['aux2'] = 1500; cursor_msg = "Horizon"
                    elif 1300 < CMDS['aux2'] < 1700:
                        CMDS['aux2'] = 2000; cursor_msg = "Flip"
                    elif CMDS['aux2'] >= 1700:
                        CMDS['aux2'] = 1000; cursor_msg = "Angle"
                elif len(button) > 3 and button[3]:
                    CMDS['aux2'] = 1800
                    cursor_msg = "FAILSAFE"

                # Send RC commands
                if (time.time() - last_loop_time) >= CTRL_LOOP_TIME:
                    last_loop_time = time.time()
                    if board.send_RAW_RC([CMDS[ki] for ki in CMDS_ORDER]):
                        dataHandler = board.receive_msg()
                        board.process_recv_data(dataHandler)

                # Send slow messages
                if (time.time() - last_slow_msg_time) >= SLOW_MSGS_LOOP_TIME:
                    last_slow_msg_time = time.time()
                    next_msg = next(slow_msgs)
                    if board.send_RAW_msg(MSPy.MSPCodes[next_msg], data=[]):
                        dataHandler = board.receive_msg()
                        board.process_recv_data(dataHandler)

                    if next_msg == 'MSP_ANALOG':
                        voltage = board.ANALOG.get('voltage', 0.0)
                        voltage_msg = ""
                        if min_voltage < voltage <= warn_voltage:
                            voltage_msg = "LOW BATT WARNING"
                        elif voltage <= min_voltage:
                            voltage_msg = "ULTRA LOW BATT!!!"
                        elif voltage >= max_voltage:
                            voltage_msg = "VOLTAGE TOO HIGH"
                        screen.addstr(8, 0, f"Battery Voltage: {voltage:.2f}V")
                        screen.addstr(8, 24, voltage_msg, curses.A_BOLD + curses.A_BLINK)
                        screen.clrtoeol()

                    elif next_msg == 'MSP_STATUS_EX':
                        mode_flags = board.CONFIG.get('mode', 0)
                        ARMED = board.bit_check(mode_flags, 0)
                        RXLOSS = board.bit_check(mode_flags, 1)

                        screen.addstr(5, 0, f"ARMED: {ARMED}")
                        screen.addstr(5, 20, f"RXLOSS: {'YES' if RXLOSS else 'NO'}", curses.A_BOLD | (curses.A_BLINK if RXLOSS else 0))
                        screen.addstr(5, 50, f"Arming Flags: {board.process_armingDisableFlags(board.CONFIG.get('armingDisableFlags', 0))}")
                        screen.addstr(6, 0, f"CPU Load: {board.CONFIG.get('cpuload', 0)}")
                        screen.addstr(6, 50, f"Cycle Time: {board.CONFIG.get('cycleTime', 0)}")
                        screen.addstr(7, 0, f"Mode: {board.CONFIG.get('mode', 0)}")
                        screen.addstr(7, 50, f"Flight Mode: {board.process_mode(mode_flags)}")

                    elif next_msg == 'MSP_MOTOR':
                        screen.addstr(9, 0, f"Motor Values: {board.MOTOR_DATA}")
                        screen.clrtoeol()
                    elif next_msg == 'MSP_RC':
                        screen.addstr(10, 0, f"RC Channels: {board.RC.get('channels', [])}")
                        screen.clrtoeol()

                    # screen.addstr(11, 0, f"GUI Cycle Time: {last_cycleTime * 1000:.2f}ms (avg {1 / (sum(average_cycle) / len(average_cycle)):.2f}Hz)")
                    avg_cycle_time = sum(average_cycle) / len(average_cycle)
                    avg_hz = 1 / avg_cycle_time if avg_cycle_time > 0 else 0
                    screen.addstr(11, 0, f"GUI Cycle Time: {last_cycleTime * 1000:.2f}ms (avg {avg_hz:.2f}Hz)")

                    screen.addstr(3, 0, cursor_msg)
                    screen.clrtoeol()

                end_time = time.time()
                last_cycleTime = end_time - start_time
                if last_cycleTime < CTRL_LOOP_TIME:
                    time.sleep(CTRL_LOOP_TIME - last_cycleTime)
                average_cycle.append(last_cycleTime)
                average_cycle.popleft()

    finally:
        screen.addstr(5, 0, "Disconnected from FC. Failsafe triggered.")
        screen.clrtoeol()

# --- Run
if __name__ == "__main__":
    run_curses(joy_controller)

