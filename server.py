import signal
import numpy as np
import socket
import bmx055

bmx = bmx055.Bmx055()

def set_timer_interrupt(func, start, cycle):
    signal.signal(signal.SIGALRM, func)
    signal.setitimer(signal.ITIMER_REAL, start, cycle)

def start_timer_interrupt_to_update_quaternion(start, cycle):
    def update_quaternion(arg1, arg2):
        bmx.update_quaternion_with_Madgwick_filter(cycle)
    
    set_timer_interrupt(update_quaternion, start, cycle)

start_timer_interrupt_to_update_quaternion(0.1, 0.1)

np.set_printoptions(precision=6, suppress=True)

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.bind(('0.0.0.0', 10000))
    s.listen(1)
    while True:
        conn, addr = s.accept()
        with conn:
            while True:
                data = conn.recv(1024)
                if data == b's\n':
                    q = bmx.q
                    msg = ' '.join(list(map(str, q)))
                    print(msg)
                    conn.sendall(msg.encode())