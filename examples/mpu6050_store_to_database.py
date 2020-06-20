import time
import board
import busio
import adafruit_mpu6050

import mysql.connector
from mysql.connector import Error


class DatabaseConnect:
    def __init__(self, x_out, y_out, z_out):
        self.x_out = x_out
        self.y_out = y_out
        self.z_out = z_out

    def connect(x_out, y_out, z_out):
        """ Connect to MySQL database """
        conn = None
        try:
            conn = mysql.connector.connect(
                host="192.168.100.7",
                database="sensor_data",
                user="demo",
                password="demo",
            )
            if conn.is_connected():
                print("Connected to MySQL database")
                print(x_out, y_out, z_out)
                insert_query = (
                    """INSERT INTO collector(acc_x, acc_y, acc_z) VALUES(%s, %s, %s)"""
                )
                cursor = conn.cursor()
                cursor.execute(insert_query, (x_out, y_out, z_out))
                # cursor.execute('INSERT INTO collector(acc_x,acc_y,acc_z) VALUES(22,33,44)')

                conn.commit()
                print(
                    cursor.rowcount, "Record inserted successfully in collector_table"
                )
                # cursor.close()
        except Error as e:
            print(e)


if __name__ == "__main__":
    i2c = busio.I2C(board.SCL, board.SDA)
    mpu = adafruit_mpu6050.MPU6050(i2c)

    while True:
        # print("Acceleration: X:%.2f, Y: %.2f, Z: %.2f m/s^2" % (mpu.acceleration))
        # print("Gyro X:%.2f, Y: %.2f, Z: %.2f degrees/s" % (mpu.gyro))
        # print("Temperature: %.2f C" % mpu.temperature)
        x_out = "%.4f" % mpu.acceleration[0]
        y_out = "%.4f" % mpu.acceleration[1]
        z_out = "%.4f" % mpu.acceleration[2]
        # print(x_out, y_out, z_out)
        DatabaseConnect.connect(x_out, y_out, z_out)
        time.sleep(3)
