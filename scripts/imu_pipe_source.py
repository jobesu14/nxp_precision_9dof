
import os
import time
import struct


import time
import board
import busio
import adafruit_fxos8700
import adafruit_fxas21002c


_r_fd = int(os.getenv("PY_READ_FD"))
_w_fd = int(os.getenv("PY_WRITE_FD"))

_r_pipe = os.fdopen(_r_fd, 'rb', 0)
_w_pipe = os.fdopen(_w_fd, 'wb', 0)

def _read_n(f, n):
    buf = ''
    while n > 0:
        data = f.read(n)
        if data == '':
            raise RuntimeError('unexpected EOF')
        buf += data
        n -= len(data)
    return buf

def _api_get(apiName, apiArg):
    # Python sends format
    # [apiNameSize][apiName][apiArgSize][apiArg]
    # on the pipe
    msg_size = struct.pack('<I', len(apiName))
    _w_pipe.write(msg_size)
    _w_pipe.write(apiName)

    apiArg = str(apiArg)  # Just in case
    msg_size = struct.pack('<I', len(apiArg))
    _w_pipe.write(msg_size)
    _w_pipe.write(apiArg)

    # Response comes as [resultSize][resultString]
    buf = _read_n(_r_pipe, 4)
    msg_size = struct.unpack('<I', buf)[0]

    data = _read_n(_r_pipe, msg_size)

    if data == "__BAD API__":
        raise Exception(data)

    return data

def imu_data(gyro_sensor, acc_magn_sensor):
    gyro_x, gyro_y, gyro_z = gyro_sensor.gyroscope
    accel_x, accel_y, accel_z = acc_magn_sensor.accelerometer
    #mag_x, mag_y, mag_z = acc_magn_sensor.magnetometer

    msg_str = '{0:0.6f},{1:0.6f},{2:0.6f})'.format(gyro_x, gyro_y, gyro_z)
    msg_str += '{0:0.6f},{1:0.6f},{2:0.6f})'.format(accel_x, accel_y, accel_z)

    print(msg_str)

    msg_size = struct.pack('<I', len(msg_str))
    _w_pipe.write(msg_size)
    _w_pipe.write(msg_str)

def main():
    i2c = busio.I2C(board.SCL_1, board.SDA_1)
    print("I2C 1 ok!")

    acc_magn_sensor = adafruit_fxos8700.FXOS8700(i2c)
    print("acc magn ok!")

    gyro_sensor = adafruit_fxas21002c.FXAS21002C(i2c)
    print("gyro ok!")
    
    while True:
        imu_data(gyro_sensor, acc_magn_sensor)
        time.sleep(1.0)


if __name__ == "__main__":
    main()
