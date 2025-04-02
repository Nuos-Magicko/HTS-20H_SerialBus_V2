import serial
import RPi.GPIO as GPIO
from time import sleep
import warnings

SERVO_MOVE_TIME_WRITE               =       1
SERVO_MOVE_TIME_READ                =       2
SERVO_MOVE_TIME_WAIT_WRITE          =       7
SERVO_MOVE_TIME_WAIT_READ           =       8
SERVO_MOVE_START                    =       11
SERVO_MOVE_STOP                     =       12
SERVO_ID_WRITE                      =       13
SERVO_ID_READ                       =       14
SERVO_ANGLE_OFFSET_ADJUST           =       17
SERVO_ANGLE_OFFSET_WRITE            =       18
SERVO_ANGLE_OFFSET_READ             =       19
SERVO_ANGLE_LIMIT_WRITE             =       20
SERVO_ANGLE_LIMIT_READ              =       21
SERVO_VIN_LIMIT_WRITE               =       22
SERVO_VIN_LIMIT_READ                =       23
SERVO_TEMP_MAX_LIMIT_WRITE          =       24
SERVO_TEMP_MAX_LIMIT_READ           =       25
SERVO_TEMP_READ                     =       26
SERVO_VIN_READ                      =       27
SERVO_POS_READ                      =       28
SERVO_OR_MOTOR_MODE_WRITE           =       29
SERVO_OR_MOTOR_MODE_READ            =       30
SERVO_LOAD_OR_UNLOAD_WRITE          =       31
SERVO_LOAD_OR_UNLOAD_READ           =       32
SERVO_LED_CTRL_WRITE                =       33
SERVO_LED_CTRL_READ                 =       34
SERVO_LED_ERROR_WRITE               =       35
SERVO_LED_ERROR_READ                =       36

SERVO_PACKET_LEN_3                  =       3
SERVO_PACKET_LEN_4                  =       4
SERVO_PACKET_LEN_5                  =       5
SERVO_PACKET_LEN_7                  =       7

SERVO_PACKET_OK                     =       0
SERVO_PACKET_SEND_ERROR             =       -1
SERVO_PACKET_HEADER_ERROR           =       -2
SERVO_PACKET_ID_ERROR               =       -3
SERVO_PACKET_LEN_ERROR              =       -4
SERVO_PACKET_CMD_ERROR              =       -5
SERVO_PACKET_CHKSUM_ERROR           =       -6
SERVO_PORT_BUSY                     =       -7

SERVO_GPIO_OUT_HIGH                 =       1
SERVO_GPIO_OUT_LOW                  =       0

class ServoSerial:
    header1                         =       0x55
    header2                         =       0x55
    def __init__(self, port, addr, gpio_tx_ena, gpio_rx_ena):
        self.addr                   =       addr
        self.port                   =       port
        self.tx_ena                 =       gpio_tx_ena
        self.rx_ena                 =       gpio_rx_ena
        self.Serial                 =       serial.Serial(port=self.port, baudrate=115200, timeout= 1)  # open serial port
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(self.tx_ena,GPIO.OUT)
        GPIO.setup(self.rx_ena,GPIO.OUT)
        GPIO.output(self.tx_ena, GPIO.HIGH)
        GPIO.output(self.rx_ena, GPIO.HIGH)


    def transmit_data(self, data):
        # start to send
        if self.Serial.is_open == False:
            self.Serial.open()
        if self.Serial.is_open == True:
            # print("Transmitted data(RAW): ",bytearray(data))
            print("Transmitted data(DEC): ",data)
            num = self.Serial.write(bytearray(data))
            self.Serial.flush()
            # eliminated the echo
            self.Serial.read(num)
            sleep(0.08)
            # check that data sent completely
            if num == len(data):
                print("Servo data has been transmited successfully.\n")
                return SERVO_PACKET_OK
            else:
                print("Servo data was occured an error.\n")
                return SERVO_PACKET_SEND_ERROR
        else:
            print("The port has not open yet, please make sure the port is ready to use\n")
            return SERVO_PACKET_SEND_ERROR
    
    def receive_data(self, read_len, cmd):
        rx = self.Serial.read(read_len)
        sleep(0.01)
        # print("Received data(RAW): ",rx)
        print("Received data(DEC): ",list(rx))
        if self.rx_check(rx, read_len, cmd) == SERVO_PACKET_OK:
            return rx
        else:
            return []


    def checksum(self,arr):
        ''' 
        This function use to calculate the checksum method :
        Checksum = ~(ID + Len + CMD + Param1 + Param2 + ... + ParamN)
        '''
        return (~sum(arr[2:])) & 0xFF
    
    def rx_check(self, rx, length, cmd):
        '''
        This function use to check the data packet.
        '''
        if len(rx) != length:
            print("The length of packet doesn't match to the actual length.\n")
            return SERVO_PACKET_LEN_ERROR
        elif rx[0] != 0x55 and rx[1] != 0x55:
            print("The header of packet isn't correct.\n")
            return SERVO_PACKET_HEADER_ERROR
        elif rx[2] != self.addr:
            print("The Servo ID doesn't match.\n")
            return SERVO_PACKET_ID_ERROR
        elif rx[4] != cmd:
            print("The command doesn't match.\n")
            return SERVO_PACKET_CMD_ERROR
        elif rx[-1] != self.checksum(rx[0:len(rx)-1]):
            print("The checksum isn't correct.\n")
            return SERVO_PACKET_CHKSUM_ERROR
        else:
            return SERVO_PACKET_OK
    
    def twos_comp(self,val, bits):
        """compute the 2's complement of int value val"""
        return (~abs(val) + 1) & (2**bits - 1)
    
    def inv_twos_comp(self, val, bits):
        """compute int value val of 2's complement"""
        return -(~(val - 1) & (2**bits - 1))

    '''
    1. Command name: SERVO_MOVE_TIME_WRITE Command value:1
    Length: 7
    Parameter 1: lower 8 bits of angle value
    Parameter 2: higher 8 bits of angle value.range 0~1000. corresponding to the
    servo angle of 0 ~ 240 °, that means the minimum angle of the servo can be
    varied is 0.24 degree. Parameter 3: lower 8 bits of time value
    Parameter 4: higher 8 bits of time value. the range of time is 0~30000ms. When the command is sent to servo, the servo will be rotated from current
    angle to parameter angle at uniform speed within parameter time. After the
    command reaches servo, servo will rotate immediately.
    '''
    def Write_MoveTime(self, angle, time):
        '''
        This Function use to command servo motor to go to a desired angle with specific time.
        angle(0 - 240 degree), time(0 - 30000 ms)
        return 0 (success), 1 (Issue)
        '''
        # absolute the value of data
        angle = abs(angle)
        time = abs(time)

        # check the exceed value
        if angle > 240:
            angle = 240
        
        if time > 30000:
            time = 30000
        
        #divide into 2-Bytes
        angle_l =   int(angle * 1000 / 240) & 0x00FF
        angle_h =   (int(angle * 1000 / 240) & 0xFF00) >> 8

        tim_l   =   time & 0x00FF
        tim_h  =   (time & 0xFF00) >> 8
        
        # arrange the packet
        packet = [self.header1, self.header2, self.addr, SERVO_PACKET_LEN_7, SERVO_MOVE_TIME_WRITE, angle_l, angle_h, tim_l, tim_h]
        # add checksum
        packet.append(self.checksum(packet))

        # start to send
        num = self.transmit_data(packet)
        if num == SERVO_PACKET_OK:
            print("The servo has moved to : " + str(angle) + " degree within time : " + str(time) + " ms.\n")
    
    '''
    2. Command name: SERVO_MOVE_TIME_READ Command value: 2Length: 3
    Read the angle and time value which sent by SERVO_MOVE_TIME_WRITE to the servo
    For the details of the command packet that the servo returns to host computer.
    Command name: SERVO_MOVE_TIME_READ Command value: 2Length: 7
    Parameter 1: lower 8 bits of angle value
    Parameter 2: higher 8 bits of angle, range 0~1000
    Parameter 3: lower 8 bits of time value
    Parameter 4: higher 8 bits of time value, range 0~30000ms
    '''
    def Read_MoveTime(self):
        '''
        This Function use to read the angle and time value which sent by SERVO_MOVE_TIME_WRITE to the servo.
        return angle,time
        '''
        # arrange the packet
        packet = [self.header1, self.header2, self.addr, SERVO_PACKET_LEN_3, SERVO_MOVE_TIME_READ]
        # add checksum
        packet.append(self.checksum(packet))
        # start to send
        num = self.transmit_data(packet)
        if num == SERVO_PACKET_OK:
            rx = self.receive_data(SERVO_PACKET_LEN_7+3, SERVO_MOVE_TIME_READ)
                    
        if len(rx) == SERVO_PACKET_LEN_7 + 3:
            move = int(round((rx[5] + (rx[6] << 8)) * 0.24))
            time = int(round((rx[7] + (rx[8] << 8))))
            print("The servo has moved to : " + str(move) + " degree within time : " + str(time) + " ms.\n")
            return move, time
        else:
            print("An error was occured.\n")
            return -1,-1
        
    '''
    Command name: SERVO_MOVE_TIME_WAIT_WRITE
    Command value: 7 Length : 7
    Parameter1: lower 8 bits of preset angle
    Parameter2: higher 8 bits of preset angle. range 0~1000. corresponding to the servo angle of 0 ~ 240 °
    Parameter3: lower 8 bits of preset time
    Parameter3: higher 8 bits of preset time. the range of time is 0~30000ms. The function of this command is similar to this
    “SERVO_MOVE_TIME_WRITE” command in the first point. But the difference is that the servo will not immediately turn when the command arrives at the servo,
    the servo will be rotated from current angle to parameter angle at uniform speed within parameter time until 
    the command name SERVO_MOVE_START sent to servo(command value of 11), then the servo will be rotated from current angle to setting angle at uniformspeed within setting time.
    '''
    def Write_MoveTimeWait(self, angle, time):
        '''
        This Function use to command servo motor to go to a desired angle with specific time when the status in SERVO_MOVE_START is set.
        angle(0 - 240 degree), time(0 - 30000 ms)
        return 0 (success), 1 (Issue)
        '''
        # absolute the value of data
        angle = abs(angle)
        time = abs(time)

        # check the exceed value
        if angle > 240:
            angle = 240
        
        if time > 30000:
            time = 30000
        
        #divide into 2-Bytes
        angle_l =   int(round(angle * 1000 / 240)) & 0x00FF
        angle_h =   (int(round(angle * 1000 / 240)) & 0xFF00) >> 8

        tim_l   =   time & 0x00FF
        tim_h  =   (time & 0xFF00) >> 8
        
        # arrange the packet
        packet = [self.header1, self.header2, self.addr, SERVO_PACKET_LEN_7, SERVO_MOVE_TIME_WAIT_WRITE, angle_l, angle_h, tim_l, tim_h]
        # add checksum
        packet.append(self.checksum(packet))

        # start to send
        num = self.transmit_data(packet)
        if num == SERVO_PACKET_OK:
            print("The servo has been waiting to move to desired postion at : " + str(angle) + " degree within time : " + str(time) + " ms.\n")
    
    '''
    Read the preset angle and preset time value which sent by SERVO_MOVE_TIME_WAIT_WRITE to the servo
    For the details of the command packet that the servo returns to host computer.
    '''
    def Read_MoveTimeWait(self):
        '''
        This Function use to read the angle and time value which sent by SERVO_MOVE_TIME_WRITE to the servo.
        return angle,time
        
        This Function is not available
        '''
        # arrange the packet
        packet = [self.header1, self.header2, self.addr, SERVO_PACKET_LEN_3, SERVO_MOVE_TIME_WAIT_READ]
        # add checksum
        packet.append(self.checksum(packet))
        
        # start to send
        num = self.transmit_data(packet)
        if num == SERVO_PACKET_OK:
            rx = self.receive_data(SERVO_PACKET_LEN_7+3, SERVO_MOVE_TIME_WAIT_READ)

        if len(rx) == SERVO_PACKET_LEN_7 + 3:
            move = rx[5] + rx[6] * 256
            time = rx[7] + rx[8] * 256
            print("The servo has moved to : " + str(move) + " degree within time : " + str(time) + " ms.\n")
            return move, time
        else:
            print("An error was occured.\n")
            return -1, -1
        
    def Write_MoveStart(self):
        '''This function uses of command SERVO_MOVE_TIME_WAIT_WRITE '''
        # arrange the packet
        packet = [self.header1, self.header2, self.addr, SERVO_PACKET_LEN_3, SERVO_MOVE_START]
        # add checksum
        packet.append(self.checksum(packet))

        # start to send
        num = self.transmit_data(packet)
        if num == SERVO_PACKET_OK:
            print("The servo has started to rotate.\n")
    
    def Write_MoveStop(self):
        '''
        When the command arrives at the servo, it will stop running immediately 
        if theservo is rotating, and stop at the current angle position.
        '''
        # arrange the packet
        packet = [self.header1, self.header2, self.addr, SERVO_PACKET_LEN_3, SERVO_MOVE_STOP]
        # add checksum
        packet.append(self.checksum(packet))

        # start to send
        num = self.transmit_data(packet)
        if num == SERVO_PACKET_OK:
            print("The servo has stopped to rotate.\n")
        
    def Write_ServoID(self, ID = 1):
        '''
        The servo ID, range 0 ~ 253, defaults to 1. The command will
        re-write the ID value to the servo and save it even when power-down.
        '''
        # arrange the packet
        packet = [self.header1, self.header2, self.addr, SERVO_PACKET_LEN_4, SERVO_ID_WRITE, ID]
        # add checksum
        packet.append(self.checksum(packet))

        # start to send
        num = self.transmit_data(packet)
        if num == SERVO_PACKET_OK:
            print("The servo ID has been changed to " + str(packet[5]))
    
    def Read_ServoID(self):
        '''
        Read servo ID, For the details of the command package that the servo returnsto host computer
        '''
        # arrange the packet
        packet = [self.header1, self.header2, 0xFE, SERVO_PACKET_LEN_3, SERVO_ID_READ]
        # add checksum
        packet.append(self.checksum(packet))
        # start to send
        num = self.transmit_data(packet)
        if num == SERVO_PACKET_OK:
            rx = self.receive_data(SERVO_PACKET_LEN_4 + 3, SERVO_ID_READ)

        if len(rx) == SERVO_PACKET_LEN_4 + 3:
            print("The servo ID is " + str(rx[5]) + "\n")
            return rx[5]
        else:
            print("An error was occured.\n")
            return -1

    '''
    Command name: SERVO_ANGLE_OFFSET_ADJUST
    Command value: 17 Length: 4
    Parameter 1: servo deviation, range -125~ 125, The corresponding angle of -30 ° ~ 30 °, when this command reach to the servo, the servo will immediatelyrotate to adjust the deviation. 
    Note 1:The adjusted deviation value is not saved when power-down by thiscommand, if you want to save please refer to point 10. 
    Note 2: Because the parameter is “signed char” type of data, and the command packets 
    to be sent are “unsigned char” type of data, so beforesending, parameters are forcibly converted to “unsigned char” data and thenput them in command packet.
    '''   
    def Write_AngleOffsetAdjust(self, adjust):
        '''
        this function use to servo deviation angle of -30 ° ~ 30 ° with range -125 ~ 125 in signed char.
        '''
        if adjust < -30:
            adjust = -30
        if adjust > 30:
            adjust = 30
        
        val = int(round(adjust * 125 / 30))
        if val >= 0:
            packet = [self.header1, self.header2, self.addr, SERVO_PACKET_LEN_4, SERVO_ANGLE_OFFSET_ADJUST, val]
        else:
            packet = [self.header1, self.header2, self.addr, SERVO_PACKET_LEN_4, SERVO_ANGLE_OFFSET_ADJUST, self.twos_comp(val, 8)]
        # add checksum
        packet.append(self.checksum(packet))

        # start to send
        num = self.transmit_data(packet)
        if num == SERVO_PACKET_OK:
            print("The angle of servo has been offseted by " + str(adjust) + ".\n")
    

    def Write_AngleOffsetSave(self):
        '''
        Save the deviation value, and support “power-down save”. The adjustment of
        the deviation is stated in point 9.
        '''
        # arrange the packet
        packet = [self.header1, self.header2, self.addr, SERVO_PACKET_LEN_3, SERVO_ANGLE_OFFSET_WRITE]
        # add checksum
        packet.append(self.checksum(packet))

        # start to send
        num = self.transmit_data(packet)
        if num == SERVO_PACKET_OK:
            print("The angle of servo has been saved.\n")
    
    def Read_AngleOffset(self):
        '''
        Read the deviation value set by the servo
        '''
        # arrange the packet
        packet = [self.header1, self.header2, self.addr, SERVO_PACKET_LEN_7, SERVO_ANGLE_OFFSET_READ, 0, 0, 0, 0]
        # add checksum
        packet.append(self.checksum(packet))
        # start to send
        num = self.transmit_data(packet)
        if num == SERVO_PACKET_OK:
            rx = self.receive_data(SERVO_PACKET_LEN_4 + 3, SERVO_ANGLE_OFFSET_READ)

        if len(rx) == SERVO_PACKET_LEN_4 + 3:
            offset = rx[5]
            if offset > 127:
                offset = self.inv_twos_comp(offset, 8)
            print("The angle of servo has been offseted by " + str(offset) + ".\n")
            return int(round(offset * 30 / 125))
        else:
            print("An error was occured.\n")
            return -1
    
    def Write_AngleLimit(self, low_angle_limit, high_angle_limit):
        '''
        The minimum angle value should always be less than the maximumangle value. 
        The command is sent to the servo, and the rotation angle of the servo will be limited between the minimum and maximum angle. And the anglelimit
        value supports 'power-down save'.
        Parameter 1: lower 8 bits of minimum angle
        Parameter 2: higher 8 bits of minimum angle, range 0~1000
        Parameter 3: lower 8 bits of maximum angle
        Parameter 4: higher 8 bits of maximum angle, range 0~1000
        '''
        low_angle_limit     = int(round(abs(low_angle_limit) * 1000 / 240))
        high_angle_limit    = int(round(abs(high_angle_limit) * 1000 / 240))

        low_angle_limit_l   =   low_angle_limit & 0x00FF
        low_angle_limit_h   =   (low_angle_limit & 0xFF00) >> 8

        high_angle_limit_l  =   high_angle_limit & 0x00FF
        high_angle_limit_h  =   (high_angle_limit & 0xFF00) >> 8
        # arrange the packet
        packet = [self.header1, self.header2, self.addr, SERVO_PACKET_LEN_7, SERVO_ANGLE_LIMIT_WRITE, low_angle_limit_l, low_angle_limit_h, high_angle_limit_l, high_angle_limit_h]
        # add checksum
        packet.append(self.checksum(packet))
        if low_angle_limit >= high_angle_limit:
            print("WARNNING!! minimun angle higher than maximun angle")

        # start to send
        num = self.transmit_data(packet)
        if num == SERVO_PACKET_OK:
            print("The maximum limitation of servo angle as " + str(high_angle_limit) + "has been set.\n")
            print("The minimum limitation of servo angle as " + str(low_angle_limit) + "has been set.\n")
        
    def Read_AngleLimit(self):
        '''
        Read the angle limit value of the servo, for the details of the instruction packet
        that the servo returns to host computer.
        Parameter 1: lower 8 bits of minimum angle value
        Parameter 2: higher 8 bits of minimum angle, range 0~1000
        Parameter 3: lower 8 bits of maximum angle value
        Parameter 4: higher 8 bits of maximum angle value, range 0~1000,
        The default value is 0, the maximum angle is 1000
        '''
        # arrange the packet
        packet = [self.header1, self.header2, self.addr, SERVO_PACKET_LEN_3, SERVO_ANGLE_LIMIT_READ]
        # add checksum
        packet.append(self.checksum(packet))
        # start to send
        num = self.transmit_data(packet)
        if num == SERVO_PACKET_OK:
            rx = self.receive_data(SERVO_PACKET_LEN_7 + 3, SERVO_ANGLE_LIMIT_READ)
        if len(rx) == SERVO_PACKET_LEN_7 + 3:
            low     =   int(round((rx[5] + (rx[6] << 8)) * 240 / 1000))
            high    =   int(round((rx[7] + (rx[8] << 8)) * 240 / 1000))
            print("The maximum limitation of servo angle as " + str(high) + "has been set.\n")
            print("The minimum limitation of servo angle as " + str(low) + "has been set.\n")
            return low,high
        else:
            print("An error was occured.")
            return -1, -1
    
    def Write_VinLimit(self, low_vin_limit, high_vin_limit):
        '''
        The minimum input voltage should always be less than the maximuminput
        voltage. The command is sent to the servo, and the input voltage of the servowill be limited between the minimum and the maximum. If the servo is out of
        range, the led will flash and alarm (if an LED alarm is set). In order to protect
        the servo, the motor will be in the unloaded power situation, and the servowill
        not output torque and the input limited voltage value supports for power-down save.
        Parameter 1: lower 8 bits of minimum input voltage
        Parameter 2: higher 8 bits of minimum input voltage, range 4500~12000mv
        Parameter 3: lower 8 bits of maximum input voltage
        Parameter 4: higher 8 bits of maximum input voltage, range 4500~12000mv
        '''
        if low_vin_limit < 4500:
            low_vin_limit = 4500
        elif low_vin_limit > 12000:
            low_vin_limit = 12000

        if high_vin_limit < 4500:
            high_vin_limit = 4500
        elif high_vin_limit > 12000:
            high_vin_limit = 12000

        low_vin_limit_l   =   low_vin_limit & 0x00FF
        low_vin_limit_h   =   (low_vin_limit & 0xFF00) >> 8

        high_vin_limit_l  =   high_vin_limit & 0x00FF
        high_vin_limit_h  =   (high_vin_limit & 0xFF00) >> 8
        # arrange the packet
        packet = [self.header1, self.header2, self.addr, SERVO_PACKET_LEN_7, SERVO_VIN_LIMIT_WRITE, low_vin_limit_l, low_vin_limit_h, high_vin_limit_l, high_vin_limit_h]
        # add checksum
        packet.append(self.checksum(packet))
        
        if low_vin_limit >= high_vin_limit:
            warnings.warn("WARNNING!! minimun Vin higher than maximun Vin")
        # start to send
        num = self.transmit_data(packet)
        if num == SERVO_PACKET_OK:
            print("The maximum limitation of servo input voltage as " + str(high_vin_limit) + "has been set.\n")
            print("The maximum limitation of servo input voltage as " + str(low_vin_limit) + "has been set.\n")
    
    def Read_VinLimit(self):
        '''
        Parameter 1: lower 8 bits of minimum input voltage
        Parameter 2: higher 8 bits of minimum input voltage, range 4500~12000mv
        Parameter 3: lower 8 bits of maximum input voltage
        Parameter 4: higher 8 bits of maximum input voltage, range 4500~12000mv
        '''
        # arrange the packet
        packet = [self.header1, self.header2, self.addr, SERVO_PACKET_LEN_3, SERVO_VIN_LIMIT_READ]
        # add checksum
        packet.append(self.checksum(packet))
        # start to send
        num = self.transmit_data(packet)
        if num == SERVO_PACKET_OK:
            rx = self.receive_data(SERVO_PACKET_LEN_7 + 3, SERVO_VIN_LIMIT_READ)

        if len(rx) == SERVO_PACKET_LEN_7 + 3:
            low     =   rx[5] + (rx[6] << 8)
            high    =   rx[7] + (rx[8] << 8)
            print("The maximum limitation of servo input voltage as " + str(high) + "has been set.\n")
            print("The minimum limitation of servo input voltage as " + str(low) + "has been set.\n")
            return low,high
        else:
            print("An error was occured.")
            return -1, -1
    
    def Write_TempMaxLimit(self, temp):
        '''
        In order to protect the servo, the motor will be in the unloaded power situation, 
        and the servo will not output torque until the temperature below this value of theservo,
        then it will once again enter the working state.and this value supports for power-down save. 
        Parameter 1: The maximum temperature limit inside the servo range
        0~100°C, the default value is 85°C, if the internal temperature of the servo exceeds 
        this value the led will flash and alarm (if an LED alarmis set).
        '''
        if temp < 50:
            temp = 50
        elif temp > 100:
            temp = 100
        # arrange the packet
        packet = [self.header1, self.header2, self.addr, SERVO_PACKET_LEN_4, SERVO_TEMP_MAX_LIMIT_WRITE, temp]
        # add checksum
        packet.append(self.checksum(packet))

        # start to send
        num = self.transmit_data(packet)
        if num == SERVO_PACKET_OK:
            print("The maximum limitation of servo temperature as " + str(temp) + "has been set.\n")
    
    def Read_TempMaxLimit(self):
        '''
        Parameter 1: The maximum temperature limit inside the servo range
        0~100°C, the default value is 85°C, if the internal temperature of the servo exceeds 
        this value the led will flash and alarm (if an LED alarmis set).
        '''
        # arrange the packet
        packet = [self.header1, self.header2, self.addr, SERVO_PACKET_LEN_3, SERVO_TEMP_MAX_LIMIT_READ]
        # add checksum
        packet.append(self.checksum(packet))
        # start to send
        num = self.transmit_data(packet)
        if num == SERVO_PACKET_OK:
            rx = self.receive_data(SERVO_PACKET_LEN_4 + 3, SERVO_TEMP_MAX_LIMIT_READ)

        if len(rx) == SERVO_PACKET_LEN_4 + 3:
            print("The maximum limitation of servo temperature as " + str(rx[5]) + "has been set.\n")
            return rx[5]
        else:
            print("An error was occured.\n")
            return -1
    
    def Read_Temp(self):
        '''
        This functin use to read the present temperature.
        Parameter 1: The current temperature inside the servo, no default value
        '''
        # arrange the packet
        packet = [self.header1, self.header2, self.addr, SERVO_PACKET_LEN_3, SERVO_TEMP_READ]
        # add checksum
        packet.append(self.checksum(packet))
        # start to send
        num = self.transmit_data(packet)
        if num == SERVO_PACKET_OK:
            rx = self.receive_data(SERVO_PACKET_LEN_4 + 3, SERVO_TEMP_READ)

        if len(rx) == SERVO_PACKET_LEN_4 + 3:
            print("The current temperature is " + str(rx[5]) + ".\n")
            return rx[5]
        else:
            print("An error was occured.\n")
            return -1
        
    def Read_Vin(self):
        '''
        This functin use to read the present voltage.
        Parameter 1: Parameter 1: lower 8 bits of current input voltage value
        Parameter 2: higher 8 bits of current input voltage value, no default
        '''
        # arrange the packet
        packet = [self.header1, self.header2, self.addr, SERVO_PACKET_LEN_3, SERVO_VIN_READ]
        # add checksum
        packet.append(self.checksum(packet))
        # start to send
        num = self.transmit_data(packet)
        if num == SERVO_PACKET_OK:
            rx = self.receive_data(SERVO_PACKET_LEN_5 + 3, SERVO_VIN_READ)

        if len(rx) == SERVO_PACKET_LEN_5 + 3:
            print("The current voltage is " + str(rx[5] + (rx[6] << 8)) + ".\n")
            return rx[5] + (rx[6] << 8)
        else:
            print("An error was occured.\n")
            return -1
        
    def Read_Pos(self):
        '''
        This functin use to read the present position.
        Parameter 1: lower 8 bits of current servo position value
        Parameter 2: higher 8 bits of current servo position value, no default
        '''
        # arrange the packet
        packet = [self.header1, self.header2, self.addr, SERVO_PACKET_LEN_3, SERVO_POS_READ]
        # add checksum
        packet.append(self.checksum(packet))
        # start to send
        num = self.transmit_data(packet)
        if num == SERVO_PACKET_OK:
            rx = self.receive_data(SERVO_PACKET_LEN_5 + 3, SERVO_POS_READ)
        if len(rx) == SERVO_PACKET_LEN_5 + 3:
            print("The current position is " + str((rx[5] + (rx[6] << 8)) * 0.24) + ".\n")
            return (rx[5] + (rx[6] << 8)) * 0.24
        else:
            print("An error was occured.\n")
            return -1
        
    def Write_ServoOrMotorMode(self, mode, motor_speed):
        '''
        This function use to choose between motor mode(1) and servo mode(0).
        Additionally in motor can control the speed of motor.
        Parameter 1: Servo mode, range 0 or 1, 0 for position control mode, 1 for
        motor control mode, default 0. 
        Parameter 2: null value
        Parameter 3: lower 8 bits of rotation speed value.
        Parameter 4: higher 8 bits of rotation speed value. range -1000~1000.

        Only in the motor control mode is valid, control the motor speed, the value of
        the negative value represents the reverse, positive value represents the forward rotation. 
        Write mode and speed do not support power-down save. 
        Note: Since the rotation speed is the “signed short int” type of data, 
        it is forced to convert the data to “unsigned short int" type of data before sending thecommand packet.
        '''
        mode = abs(mode)
        match mode:
            case 0:
                print("Servo mode has been select")
            case 1:
                print("Motor mode has been select")
            case _:
                mode = 0
                warnings.warn("Select mode Warning!! The user must select either mode 0 or mode 1. Mode 0 is selected by default.")
                
        if motor_speed > 1000:
            motor_speed = 1000
        elif motor_speed < -1000:
            motor_speed = -1000
        
        if motor_speed < 0:
            motor_speed_l  =  self.twos_comp(motor_speed,16) & 0x00FF
            motor_speed_h  =  (self.twos_comp(motor_speed,16) & 0xFF00) >> 8
        else:
            motor_speed_l  =  motor_speed & 0x00FF
            motor_speed_h  =  (motor_speed & 0xFF00) >> 8
            
        # arrange the packet
        packet = [self.header1, self.header2, self.addr, SERVO_PACKET_LEN_7, SERVO_OR_MOTOR_MODE_WRITE, mode, 0 ,motor_speed_l, motor_speed_h]
        # add checksum
        packet.append(self.checksum(packet))
        # start to send
        num = self.transmit_data(packet)
        if num == SERVO_PACKET_OK:
            print("The servo speed has been set at" + str(motor_speed) + ".\n")
    
    def Read_ServoOrMotorMode(self):
        # arrange the packet
        packet = [self.header1, self.header2, self.addr, SERVO_PACKET_LEN_3, SERVO_OR_MOTOR_MODE_READ]
        # add checksum
        packet.append(self.checksum(packet))
        # start to send
        num = self.transmit_data(packet)
        if num == SERVO_PACKET_OK:
            rx = self.receive_data(SERVO_PACKET_LEN_7 + 3, SERVO_OR_MOTOR_MODE_READ)

        if len(rx) == SERVO_PACKET_LEN_7 + 3:
            mode         =   rx[5]
            motor_speed  =   rx[7] + (rx[8] << 8)
            if motor_speed > 32767:
                motor_speed = self.inv_twos_comp(motor_speed, 16)
            print("The servo is on mode" + str(mode) + ".\n")
            print("The servo speed has been set at" + str(motor_speed) + ".\n")
            return mode, motor_speed
        else:
            print("An error was occured.\n")
            return -1
        
    def Write_LoadOrUnload(self, load):
        '''
        Parameter 1: Whether the internal motor of the servo is unloaded power-down or not, the range 0 or 1, 
        0 represents the unloading power down, and the servo has no torque output. 
        1 represents the loaded motor, then the servo has a torque output, the default 
        '''
        if load not in(0,1):
            warnings.warn("Load must be 0 or 1")
            load = 1
        # arrange the packet
        packet = [self.header1, self.header2, self.addr, SERVO_PACKET_LEN_4, SERVO_LOAD_OR_UNLOAD_WRITE, load]
        # add checksum
        packet.append(self.checksum(packet))
        # start to send
        num = self.transmit_data(packet)
    
    def Read_LoadOrUnload(self):
        # arrange the packet
        packet = [self.header1, self.header2, self.addr, SERVO_PACKET_LEN_3, SERVO_LOAD_OR_UNLOAD_READ]
        # add checksum
        packet.append(self.checksum(packet))
        # start to send
        num = self.transmit_data(packet)
        if num == SERVO_PACKET_OK:
            rx = self.receive_data(SERVO_PACKET_LEN_3 + 3, SERVO_LOAD_OR_UNLOAD_READ)

        if len(rx) == SERVO_PACKET_LEN_4 + 3:
            return rx[5]
        else:
            print("An error was occured.\n")
            return -1
        
    def Write_LedCtrl(self, led):
        '''
        Parameter 1: LED light/off state, the range 0 or 1, 
        0 represents that the LEDisalways on. 
        1 represents the LED off, the default 0, and support power-downsave
        '''
        if led < 0:
            led = 0
        if led > 1:
            led = 1
        # arrange the packet
        packet = [self.header1, self.header2, self.addr, SERVO_PACKET_LEN_4, SERVO_LED_CTRL_WRITE, led]
        # add checksum
        packet.append(self.checksum(packet))

        # start to send
        num = self.transmit_data(packet)
    
    def Read_LedCtrl(self):
        # arrange the packet
        packet = [self.header1, self.header2, self.addr, SERVO_PACKET_LEN_3, SERVO_LED_CTRL_READ]
        # add checksum
        packet.append(self.checksum(packet))
        # start to send
        num = self.transmit_data(packet)
        if num == SERVO_PACKET_OK:
            rx = self.receive_data(SERVO_PACKET_LEN_4 + 3, SERVO_LED_CTRL_READ)

        if len(rx) == SERVO_PACKET_LEN_4 + 3:
            return rx[5]
        else:
            print("An error was occured.\n")
            return -1
    
    def Write_LedError(self, alarm):
        '''
        This function use to write the alarm value.
        Parameter 1: what faults will cause LED flashing alarm value, range 0~7There are three types of faults that cause the LED to flash and alarm, regardless of whether the LED is in or off. 
        The first fault is that internal temperature of the servo exceeds the maximum temperature limit (this valueisset at point 16). 
        The second fault is that the servo input voltage exceeds thelimit value (this value is set at 14 points). 
        The third one is when locked-rotor occurred.
        No. | Description
        0   | No Alarm
        1   | Over Temperature
        2   | Over Voltage
        3   | Over Temperature and Over voltage
        4   | Locked rotor
        5   | Over Temperature and stalled
        6   | Over Voltage and stalled
        7   | Over Temperature, over voltage and stalled
        '''
        if alarm < 0:
            alarm = 0
        if alarm > 7:
            alarm = 7
        # arrange the packet
        packet = [self.header1, self.header2, self.addr, SERVO_PACKET_LEN_4, SERVO_LED_ERROR_WRITE, alarm]
        # add checksum
        packet.append(self.checksum(packet))

        # start to send
        num = self.transmit_data(packet)
    
    def Read_LedError(self):
        '''
        This function use to read the alarm value.
        Parameter 1: what faults will cause LED flashing alarm value, range 0~7There are three types of faults that cause the LED to flash and alarm, regardless of whether the LED is in or off. 
        The first fault is that internal temperature of the servo exceeds the maximum temperature limit (this valueisset at point 16). 
        The second fault is that the servo input voltage exceeds thelimit value (this value is set at 14 points). 
        The third one is when locked-rotor occurred.
        No. | Description
        0   | No Alarm
        1   | Over Temperature
        2   | Over Voltage
        3   | Over Temperature and Over voltage
        4   | Locked rotor
        5   | Over Temperature and stalled
        6   | Over Voltage and stalled
        7   | Over Temperature, over voltage and stalled
        '''
        # arrange the packet
        packet = [self.header1, self.header2, self.addr, SERVO_PACKET_LEN_3, SERVO_LED_ERROR_READ]
        # add checksum
        packet.append(self.checksum(packet))
        # start to send
        num = self.transmit_data(packet)
        if num == SERVO_PACKET_OK:
            rx = self.receive_data(SERVO_PACKET_LEN_4 + 3, SERVO_LED_ERROR_READ)
        if len(rx) == SERVO_PACKET_LEN_4 + 3:
            return rx[5]
        else:
            print("An error was occured.\n")
            return -1