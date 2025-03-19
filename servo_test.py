import servobus as svb

port = "/dev/ttyAMA1"
addr = 0x01

bus1 = svb.ServoSerial(port, addr, 18, 27)

bus1.Write_MoveTime(100,1000)