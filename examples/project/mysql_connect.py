import mysql.connector
from mysql.connector import Error

class StoreToDatabase():
#    def __init__(self,acc_x,acc_y,acc_z):
#        self.acc_x = acc_x
#        self.acc_y = acc_y
#        self.acc_z = acc_z
        
    def check_if_connected():
        conn = None
        try:
            conn = mysql.connector.connect(host='192.168.100.7', database='sensor_data', user='demo', password='demo')
            if conn.is_connected():
                return conn
        except Error as e:
            print(e)

    def accelerometer_store(acc_x, acc_y, acc_z,):
        try:
            print(acc_x, acc_y, acc_z,)
            conn = StoreToDatabase.check_if_connected()
            insert_query = """INSERT INTO accelerometer(acc_x, acc_y, acc_z) VALUES(%s, %s, %s)"""
            cursor = conn.cursor()
            cursor.execute(insert_query, (acc_x, acc_y, acc_z,))
            conn.commit()
        except Error as e:
            print(e)
    
    def gyroscope_store(gyro_x, gyro_y, gyro_z):
        try:
            conn = StoreToDatabase.check_if_connected()
            print(gyro_x, gyro_y, gyro_z)
            insert_query = """INSERT INTO gyroscope(gyro_x, gyro_y, gyro_z) VALUES(%s, %s, %s)"""
            cursor = conn.cursor()
            cursor.execute(insert_query, (gyro_x, gyro_y, gyro_z))
            conn.commit()
        except Error as e:
            print(e)
        finally:
            cursor.close()

