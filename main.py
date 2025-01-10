import serial
from pymavlink import mavutil
from datetime import datetime

# Arrays to store data
ACCData = [0.0] * 8
GYROData = [0.0] * 8
AngleData = [0.0] * 8
MagData = [0.0] * 8

# Variables for frame parsing
FrameState = 0  # Frame type (e.g., 0x51: acc, 0x52: gyro, 0x53: angle)
Bytenum = 0     # Position within the frame
CheckSum = 0    # Checksum for validation

# Variables to store processed data
a = [0.0] * 3  # Accelerometer data (x, y, z)
w = [0.0] * 3  # Gyroscope data (x, y, z)
Angle = [0.0] * 3  # Angle data (pitch, roll, yaw)
Mag = [0.0] * 3

# Initialize MAVLink connection on another UART port
mavlink_connection = mavutil.mavlink_connection('/dev/ttyAMA3', baud=115200)


def DueData(inputdata):
    """
    Core function to parse input data from the sensor and extract meaningful values.
    """
    global FrameState, Bytenum, CheckSum, a, w, Angle, Mag

    for data in inputdata:
        data = ord(data) if isinstance(data, str) else data

        if FrameState == 0:  # Determine frame type
            if data == 0x55 and Bytenum == 0:  # Start of frame
                CheckSum = data
                Bytenum = 1
                continue
            elif data == 0x51 and Bytenum == 1:  # Accelerometer frame
                CheckSum += data
                FrameState = 1
                Bytenum = 2
            elif data == 0x52 and Bytenum == 1:  # Gyroscope frame
                CheckSum += data
                FrameState = 2
                Bytenum = 2
            elif data == 0x53 and Bytenum == 1:  # Angle frame
                CheckSum += data
                FrameState = 3
                Bytenum = 2
            elif data == 0x54 and Bytenum == 1:  # Magnetometer frame
                CheckSum += data
                FrameState = 4
                Bytenum = 2

        elif FrameState == 1:  # Accelerometer data
            if Bytenum < 10:
                ACCData[Bytenum - 2] = data
                CheckSum += data
                Bytenum += 1
            else:
                if data == (CheckSum & 0xff):  # Validate checksum
                    a = get_acc(ACCData)
                CheckSum = 0
                Bytenum = 0
                FrameState = 0

        elif FrameState == 2:  # Gyroscope data
            if Bytenum < 10:
                GYROData[Bytenum - 2] = data
                CheckSum += data
                Bytenum += 1
            else:
                if data == (CheckSum & 0xff):
                    w = get_gyro(GYROData)
                CheckSum = 0
                Bytenum = 0
                FrameState = 0

        elif FrameState == 3:  # Angle data
            if Bytenum < 10:
                AngleData[Bytenum - 2] = data
                CheckSum += data
                Bytenum += 1
            else:
                if data == (CheckSum & 0xff):
                    Angle = get_angle(AngleData)
                CheckSum = 0
                Bytenum = 0
                FrameState = 0

        elif FrameState == 4:  # Magnetometer data
            if Bytenum < 10:
                MagData[Bytenum - 2] = data
                CheckSum += data
                Bytenum += 1
            else:
                if data == (CheckSum & 0xff):
                    Mag = get_mag(MagData)
                CheckSum = 0
                Bytenum = 0
                FrameState = 0

                # Get pressure via MAVLink
        pressure = get_mavlink_pressure()
        d = a + w + Angle + Mag + [pressure]
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        log_entry = (
            f"[{timestamp}] "
            "a(g):%10.3f %10.3f %10.3f w(deg/s):%10.3f %10.3f %10.3f "
            "Angle(deg):%10.3f %10.3f %10.3f mag:%10.3f %10.3f %10.3f pr: %10.3f\n"
            % tuple(d)
        )
        print(log_entry, end="")

        # Save to file
        with open("log.txt", "a") as log_file:
            log_file.write(log_entry)

        #print(
        #    "a(g):%10.3f %10.3f %10.3f w(deg/s):%10.3f %10.3f %10.3f Angle(deg):%10.3f %10.3f %10.3f mag:%10.3f %10.3f %10.3f pr: %10.3f"
        #    % tuple(d)
        #)





def get_acc(datahex):
    """Convert raw accelerometer data to g units."""
    k_acc = 16.0
    acc_x = (datahex[1] << 8 | datahex[0]) / 32768.0 * k_acc
    acc_y = (datahex[3] << 8 | datahex[2]) / 32768.0 * k_acc
    acc_z = (datahex[5] << 8 | datahex[4]) / 32768.0 * k_acc
    return _adjust_values([acc_x, acc_y, acc_z], k_acc)


def get_gyro(datahex):
    """Convert raw gyroscope data to degrees per second."""
    k_gyro = 2000.0
    gyro_x = (datahex[1] << 8 | datahex[0]) / 32768.0 * k_gyro
    gyro_y = (datahex[3] << 8 | datahex[2]) / 32768.0 * k_gyro
    gyro_z = (datahex[5] << 8 | datahex[4]) / 32768.0 * k_gyro
    return _adjust_values([gyro_x, gyro_y, gyro_z], k_gyro)


def get_angle(datahex):
    """Convert raw angle data to degrees."""
    k_angle = 180.0
    angle_x = (datahex[1] << 8 | datahex[0]) / 32768.0 * k_angle
    angle_y = (datahex[3] << 8 | datahex[2]) / 32768.0 * k_angle
    angle_z = (datahex[5] << 8 | datahex[4]) / 32768.0 * k_angle
    return _adjust_values([angle_x, angle_y, angle_z], k_angle)


def get_mag(datahex):
    """Convert raw magnetometer data."""
    k_mag = 3276.80
    mag_x = (datahex[1] << 8 | datahex[0]) / 32768.0 * k_mag
    mag_y = (datahex[3] << 8 | datahex[2]) / 32768.0 * k_mag
    mag_z = (datahex[5] << 8 | datahex[4]) / 32768.0 * k_mag
    return _adjust_values([mag_x, mag_y, mag_z], k_mag)


def _adjust_values(values, max_value):
    """Adjust values for overflow."""
    return [v - 2 * max_value if v >= max_value else v for v in values]

def set_mavlink_pressure():
    
    message = mavlink_connection.mav.command_long_encode(
        mavlink_connection.target_system,  # Target system ID
        mavlink_connection.target_component,  # Target component ID
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,  # ID of command to send
        0,  # Confirmation
        29,  # param1: Message ID to be streamed, 26 - Scaled_IMU
        10000, # param2: Interval in microseconds
        0,       # param3 (unused)
        0,       # param4 (unused)
        0,       # param5 (unused)
        0,       # param5 (unused)
        0        # param6 (unused)
        )
    mavlink_connection.mav.send(message)




def get_mavlink_pressure():
    try:
        message = mavlink_connection.recv_match(type='SCALED_PRESSURE', blocking=True, timeout=0.1)
        if message:
            return message.press_abs  # Absolute pressure in hPa
        else:
            return float('0')  # Return NaN if no data
    except Exception as e1:
        print(f"Error reading MAVLink pressure: {e1}")
        return float('nan')

set_mavlink_pressure()
if __name__ == '__main__':
    try:
        ser = serial.Serial('/dev/ttyAMA0', baudrate=115200, timeout=0.5)
        print(f"Serial port opened: {ser.is_open}")

        while True:
            datahex = ser.read(33)
            DueData(datahex)

    except serial.SerialException as e:
        print(f"Serial error: {e}")
    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("Serial port closed.")
