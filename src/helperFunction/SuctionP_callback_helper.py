#!/usr/bin/env python3
import numpy as np
import rospy
from cuhk_suction_cup.msg import SensorPacket
from cuhk_suction_cup.msg import cmdPacket
from scipy import signal


class P_CallbackHelp(object):
    def __init__(self):
        rospy.Subscriber("SensorPacket", SensorPacket, self.callback_P)

        self.START_CMD = 2
        self.IDLE_CMD = 3
        self.RECORD_CMD = 10
        self.msg2Sensor = cmdPacket()
        # self.P_vac = -15000.0
        self.P_vac = -1000.0

        self.sensorCMD_Pub = rospy.Publisher("cmdPacket", cmdPacket, queue_size=10)

        # callback delay test
        self.callback_Pub = rospy.Publisher('SensorCallback', SensorPacket, queue_size=10)
        self.callback_Pressure = SensorPacket()

        ## For pressure feedback
        self.Psensor_Num = 4
        self.BufferLen = 7

        self.PressureBuffer = [[0.0] * self.Psensor_Num] * self.BufferLen
        self.P_idx = 0
        self.startPresAvg = False
        self.four_pressure = [0.0] * self.Psensor_Num
        self.thisPres = 0

        # For FFT
        self.samplingF = 166
        self.FFTbuffer_size = int(self.samplingF / 2)  # 166 is 1 second
        self.PressurePWMBuffer = np.array(
            [[0] * self.Psensor_Num] * self.FFTbuffer_size
        )
        self.PressureOffsetBuffer = np.array([[0] * self.Psensor_Num] * 51)
        self.PWM_idx = 0
        self.offset_idx = 0
        self.startPresPWMAvg = False
        self.offsetMissing = True
        self.four_pressurePWM = np.array([0.0] * 4)
        self.power = 0
        self.PressureOffset = np.array([0.0] * 4)

    def startSampling(self):
        self.msg2Sensor.cmdInput = self.START_CMD
        self.sensorCMD_Pub.publish(self.msg2Sensor)

    def stopSampling(self):
        self.msg2Sensor.cmdInput = self.IDLE_CMD
        self.sensorCMD_Pub.publish(self.msg2Sensor)

    def setNowAsOffset(self):
        self.PressureOffset *= 0
        rospy.sleep(0.5)
        # print("self.PressureBuffer: ", self.PressureBuffer)
        self.PressureOffset = np.mean(self.PressureBuffer, axis=0)

    def callback_P(self, data):
        fs = self.samplingF
        N = self.FFTbuffer_size
        fPWM = 30

        # print("self.four_pressurePWM:", np.floor(self.four_pressurePWM))
        # print("self.PressureOffset: ", self.PressureOffset)
        # print("self.PressureOffsetBuffer: ", self.PressureOffsetBuffer)

        # fill in the pressure data ring buffer
        self.thisPres = np.array(data.data)

        self.PressureBuffer[self.P_idx] = self.thisPres #- self.PressureOffset
        self.P_idx += 1

        # if buffer is filled, then set average flag to true and reset idx
        if self.P_idx == len(self.PressureBuffer):
            # averagin flag is always true now, i.e. ring buffer
            self.startPresAvg = True
            self.P_idx = 0

        # if averaging flag is True
        if self.startPresAvg:
            averagePres_dummy = [0] * 4

            # let each row contribute to its column average
            for pressure in self.PressureBuffer:
                first = averagePres_dummy
                second = [x / len(self.PressureBuffer) for x in pressure]
                final_list = [sum(value) for value in zip(first, second)]
                averagePres_dummy = final_list

            self.four_pressure = averagePres_dummy

            # callback delay check
            self.callback_Pressure.data = averagePres_dummy
            self.callback_Pub.publish(self.callback_Pressure)

    def get_P_WENS(self):
        # absolute pressures - P_atm for each sensor
        P0, P1, P2, P3 = self.four_pressure

        # absolute cardinal direction pressures
        PW = (P3 + P2) / 2
        PE = (P1 + P0) / 2
        PN = (P1 + P2) / 2
        PS = (P0 + P3) / 2

        return PW, PE, PN, PS
        # P0pwm = four_pressurePWM[0]
        # P1pwm = four_pressurePWM[1]
        # P2pwm = four_pressurePWM[2]
        # P3pwm = four_pressurePWM[3]
