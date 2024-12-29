# coding: UTF-8
import serial
 
# Arrays to store data
ACCData = [0.0] * 8
GYROData = [0.0] * 8
AngleData = [0.0] * 8

# Variables for frame parsing
FrameState = 0  # Frame type (e.g., 0x51: acc, 0x52: gyro, 0x53: angle)
Bytenum = 0     # Position within the frame
CheckSum = 0    # Checksum for validation

# Variables to store processed data
a = [0.0] * 3  # Accelerometer data (x, y, z)
w = [0.0] * 3  # Gyroscope data (x, y, z)
Angle = [0.0] * 3  # Angle data (pitch, roll, yaw)

def DueData(inputdata):
    """
    Core function to parse input data from the sensor and extract meaningful values.
    """
    global FrameState, Bytenum, CheckSum, a, w, Angle

    for data in inputdata:
        # Convert byte to integer
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
                    d = a + w + Angle
                    print("a(g):%10.3f %10.3f %10.3f w(deg/s):%10.3f %10.3f %10.3f Angle(deg):%10.3f %10.3f %10.3f" % tuple(d))
                CheckSum = 0
                Bytenum = 0
                FrameState = 0

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

def _adjust_values(values, max_value):
    """Adjust values for overflow."""
    return [v - 2 * max_value if v >= max_value else v for v in values]

if __name__ == '__main__':
    try:
        # Initialize serial connection
        ser = serial.Serial('/dev/serial0', baudrate=115200, timeout=0.5)
        print(f"Serial port opened: {ser.is_open}")

        while True:
            datahex = ser.read(33)  # Read 33 bytes at a time
            DueData(datahex)

    except serial.SerialException as e:
        print(f"Serial error: {e}")
    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("Serial port closed.")
