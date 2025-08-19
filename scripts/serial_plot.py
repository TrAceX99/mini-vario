import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import time
import threading

# --- CONFIG ---
SERIAL_PORT = "COM3"
BAUDRATE = 115200   # adjust if needed
HISTORY_SECONDS = 10

# --- Data storage ---
times = deque(maxlen=1000)
pressures = deque(maxlen=1000)

# --- Serial reader thread ---
def serial_reader():
    ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1)
    while True:
        try:
            line = ser.readline().decode("ascii", errors="ignore").strip()
            pressure = parse_lk8ex1(line)
            if pressure is not None:
                now = time.time()
                times.append(now)
                pressures.append(pressure)
        except Exception:
            continue

# --- Parse LK8EX1 string ---
def parse_lk8ex1(line):
    """
    Example LK8EX1 string:
    $LK8EX1,pressure,altitude,vario,temperature,battery*checksum
    """
    try:
        if not line.startswith("$LK8EX1"):
            return None
        parts = line.split(",")
        return float(parts[1])  # Pressure is field 2
    except:
        return None

# --- Plot update ---
def update(frame):
    cutoff = time.time() - HISTORY_SECONDS
    while times and times[0] < cutoff:
        times.popleft()
        pressures.popleft()

    ax.clear()
    if times:
        rel_times = [t - times[0] for t in times]
        ax.plot(rel_times, pressures, label="Pressure (Pa)")
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Pressure (Pa)")
        ax.set_title("LK8EX1 Pressure Readings (last 10s)")
        ax.legend()
        ax.grid(True)

# --- Start serial thread ---
thread = threading.Thread(target=serial_reader, daemon=True)
thread.start()

# --- Setup plot ---
fig, ax = plt.subplots()
ani = animation.FuncAnimation(fig, update, interval=200)
plt.show()
