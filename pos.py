import tkinter as tk
from tkinter import ttk
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import threading
import time
import smbus
import math

# MPU6050 Registers and their Address
MPU6050_ADDR = 0x68
PWR_MGMT_1 = 0x6B
ACCEL_XOUT_H = 0x3B
GYRO_XOUT_H = 0x43

# Sensitivity scalars
ACCEL_SCALE = 16384.0  # Accelerometer scale factor for ±2g (LSB/g)
GYRO_SCALE = 131.0     # Gyroscope scale factor for ±250°/s (LSB/°/s)
GRAVITY = 9.80665      # Acceleration due to gravity (m/s^2)

# Global variables for roll, pitch, yaw
roll, pitch, yaw = 0, 0, 0

# Function to read raw data from MPU6050
def read_raw_data(bus, addr, reg):
    high = bus.read_byte_data(addr, reg)
    low = bus.read_byte_data(addr, reg + 1)
    value = (high << 8) | low
    if value > 32768:
        value = value - 65536
    return value

# Function to calculate roll, pitch, yaw
def calculate_roll_pitch_yaw(acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, dt, prev_angles):
    roll_acc = math.degrees(math.atan2(acc_y, acc_z))
    pitch_acc = math.degrees(math.atan2(-acc_x, math.sqrt(acc_y**2 + acc_z**2)))

    roll_gyro = prev_angles["roll"] + gyro_x * dt
    pitch_gyro = prev_angles["pitch"] + gyro_y * dt
    yaw_gyro = prev_angles["yaw"] + gyro_z * dt

    alpha = 0.96
    roll = alpha * roll_gyro + (1 - alpha) * roll_acc
    pitch = alpha * pitch_gyro + (1 - alpha) * pitch_acc
    yaw = yaw_gyro

    return {"roll": roll, "pitch": pitch, "yaw": yaw}

# Function to update roll, pitch, yaw from MPU6050
def update_sensor_data():
    global roll, pitch, yaw

    bus = smbus.SMBus(1)
    bus.write_byte_data(MPU6050_ADDR, PWR_MGMT_1, 0)

    prev_angles = {"roll": 0, "pitch": 0, "yaw": 0}
    prev_time = time.time()

    while True:
        acc_x_raw = read_raw_data(bus, MPU6050_ADDR, ACCEL_XOUT_H)
        acc_y_raw = read_raw_data(bus, MPU6050_ADDR, ACCEL_XOUT_H + 2)
        acc_z_raw = read_raw_data(bus, MPU6050_ADDR, ACCEL_XOUT_H + 4)

        gyro_x_raw = read_raw_data(bus, MPU6050_ADDR, GYRO_XOUT_H)
        gyro_y_raw = read_raw_data(bus, MPU6050_ADDR, GYRO_XOUT_H + 2)
        gyro_z_raw = read_raw_data(bus, MPU6050_ADDR, GYRO_XOUT_H + 4)

        acc_x = (acc_x_raw / ACCEL_SCALE) * GRAVITY
        acc_y = (acc_y_raw / ACCEL_SCALE) * GRAVITY
        acc_z = (acc_z_raw / ACCEL_SCALE) * GRAVITY

        gyro_x = gyro_x_raw / GYRO_SCALE
        gyro_y = gyro_y_raw / GYRO_SCALE
        gyro_z = gyro_z_raw / GYRO_SCALE

        curr_time = time.time()
        dt = curr_time - prev_time
        prev_time = curr_time

        angles = calculate_roll_pitch_yaw(acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, dt, prev_angles)
        prev_angles = angles

        roll, pitch, yaw = angles["roll"], angles["pitch"], angles["yaw"]

        time.sleep(0.1)

# Function to update 3D visualization
def update_plot():
    global roll, pitch, yaw

    while True:
        time.sleep(0.1)
        ax.cla()

        cube = np.array([
            [-1, -1, -1],
            [1, -1, -1],
            [1, 1, -1],
            [-1, 1, -1],
            [-1, -1, 1],
            [1, -1, 1],
            [1, 1, 1],
            [-1, 1, 1]
        ])

        def rotation_matrix(axis, theta):
            axis = axis / np.sqrt(np.dot(axis, axis))
            a = np.cos(theta / 2.0)
            b, c, d = -axis * np.sin(theta / 2.0)
            return np.array([
                [a * a + b * b - c * c - d * d, 2 * (b * c - a * d), 2 * (b * d + a * c)],
                [2 * (b * c + a * d), a * a + c * c - b * b - d * d, 2 * (c * d - a * b)],
                [2 * (b * d - a * c), 2 * (c * d + a * b), a * a + d * d - b * b - c * c]
            ])

        cube = cube @ rotation_matrix([1, 0, 0], np.radians(roll)).T
        cube = cube @ rotation_matrix([0, 1, 0], np.radians(pitch)).T
        cube = cube @ rotation_matrix([0, 0, 1], np.radians(yaw)).T

        edges = [
            (0, 1), (1, 2), (2, 3), (3, 0),
            (4, 5), (5, 6), (6, 7), (7, 4),
            (0, 4), (1, 5), (2, 6), (3, 7)
        ]
        for edge in edges:
            start, end = edge
            ax.plot3D(
                [cube[start, 0], cube[end, 0]],  # X-coordinates
                [cube[start, 1], cube[end, 1]],  # Y-coordinates
                [cube[start, 2], cube[end, 2]],  # Z-coordinates
                color="blue"
            )

        ax.set_xlim([-2, 2])
        ax.set_ylim([-2, 2])
        ax.set_zlim([-2, 2])
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_title("3D Orientation Visualization")
        canvas.draw()

# Create GUI window
root = tk.Tk()
root.title("3D Orientation Visualization")

fig = plt.figure(figsize=(6, 6))
ax = fig.add_subplot(111, projection='3d')
canvas = FigureCanvasTkAgg(fig, master=root)
canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=True)

sensor_thread = threading.Thread(target=update_sensor_data, daemon=True)
plot_thread = threading.Thread(target=update_plot, daemon=True)
sensor_thread.start()
plot_thread.start()

root.mainloop()
