# from servobus import ServoSerial
from servobus import ServoSerial
import time

port = "/dev/ttyAMA0"
addr = 0x04

bus1 = ServoSerial(port=port , addr=addr, gpio_tx_ena=18 , gpio_rx_ena=27)

# bus1.Write_ServoID(0x04)
bus1.Write_MoveTime(0,1000)
while(1):
    # bus1.Write_MoveTime(200,1000)
    # bus1.Read_MoveTime()
    # bus1.Read_Pos()
    # bus1.Read_ServoID()
    # bus1.Read_Temp()
    time.sleep(1)