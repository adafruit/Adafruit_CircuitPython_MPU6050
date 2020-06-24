"""Author: Sanjog Sigdel
--------------------------------------------------
This example works only on Blinka / RasPi devices.
--------------------------------------------------

Code commented out in the file is just to show different way
of playing around the sensor to connect and execute sql queries.

Donot forget to make changes in the database configuration before running the code."""
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
#         Connect to MySQL database
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
#                     INSERT INTO collector(acc_x, acc_y, acc_z) VALUES(%s, %s, %s)
                )
                cursor = conn.cursor()
                cursor.execute(insert_query, (x_out, y_out, z_out))
                conn.commit()
                print(
                    cursor.rowcount, "Record inserted successfully in collector_table"
                )
        except Error as e:
            print(e)


if __name__ == "__main__":
    i2c = busio.I2C(board.SCL, board.SDA)
    mpu = adafruit_mpu6050.MPU6050(i2c)

    while True:        
        x_out = "%.4f" % mpu.acceleration[0]
        y_out = "%.4f" % mpu.acceleration[1]
        z_out = "%.4f" % mpu.acceleration[2]
        
        DatabaseConnect.connect(x_out, y_out, z_out)
        time.sleep(1)
