from pymodbus.client.sync import ModbusTcpClient as ModbusClient
from pymodbus.constants import Endian
from pymodbus.payload import BinaryPayloadDecoder
import numpy as np
import time

# Change ip on pc to 192.168.1.k where k is different from 1 or anything else connected

class Modbus(object):
    def __init__(self, ip, unit, port):
        """

        :param ip: String
        :param unit: Int
        :param port: Int
        """
        self.unit = unit
        self.modbus = ModbusClient(host=ip, port=port)
        return_val = self.modbus.connect()  # Tries to connect to the computebox
        print("Connection status ", return_val)
        self.leftFTSensor = np.zeros(6)  # [fx, fy, fz, tx, ty, tz]
        self.rightFTSensor = np.zeros(6)
        self.currentGipperWidth = 0  # in mm
        self.gripperBusy = 0  # 1 if busy, can't accept new commands until completed previous.
        self.gripDetected = 0  # 1 if grip is detected
        self.proximityValueL = 0.0
        self.proximityStatusL = 0.0
        self.proximityValueR = 0.0
        self.proximityStatusR = 0.0
        self.statusLeft = 0  # 0 for no error
        self.statusRight = 0  # 0 for no error

    def read(self):
        for i in range(3):
            self.leftFTSensor[i] = self.validator_int16(self.modbus.read_holding_registers(259+i, 1,
                                                                                           unit=self.unit))/10
            self.leftFTSensor[i+3] = self.validator_int16(self.modbus.read_holding_registers(259+3+i, 1,
                                                                                             unit=self.unit))/100

            self.rightFTSensor[i] = self.validator_int16(self.modbus.read_holding_registers(268+i, 1,
                                                                                            unit=self.unit))/10
            self.rightFTSensor[i+3] = self.validator_int16(self.modbus.read_holding_registers(268+3+i, 1,
                                                                                              unit=self.unit))/100

        self.currentGipperWidth = self.validator_int16(self.modbus.read_holding_registers(280, 1, unit=self.unit)) / 10
        self.gripperBusy = self.modbus.read_holding_registers(281)
        self.gripDetected = self.modbus.read_holding_registers(282)
        self.proximityValueL = self.validator_int16(self.modbus.read_holding_registers(275, 1, unit=self.unit)) / 10
        self.proximityValueR = self.validator_int16(self.modbus.read_holding_registers(278, 1, unit=self.unit)) / 10
        self.proximityStatusL = self.modbus.read_holding_registers(274, 1, unit=self.unit)
        self.proximityStatusR = self.modbus.read_holding_registers(277, 1, unit=self.unit)
        self.statusLeft = self.modbus.read_holding_registers(257, 1, unit=self.unit)
        self.statusRight = self.modbus.read_holding_registers(266, 1, unit=self.unit)

    def command(self, targetWidth, targetForce, command):
        """

        :param targetWidth: grip width mm
        :param targetForce: grip force in N
        :param command: 0 for stop and 1 for activate motion
        :return: Bool
        """

        self.modbus.write_register(3, int(targetWidth * 10), unit=self.unit)
        self.modbus.write_register(2, int(targetForce * 10), unit=self.unit)
        if command == 0:
            self.modbus.write_register(4, 0, unit=self.unit)
            return True

        if self.gripperBusy:
            return False
        self.modbus.write_register(4, 1, unit=self.unit)
        return True

    def validator_int16(self, instance):
        # Function taken from other source
        if not instance.isError():
            '''.isError() implemented in pymodbus 1.4.0 and above.'''
            decoder = BinaryPayloadDecoder.fromRegisters(
                instance.registers,
                byteorder=Endian.Big, wordorder=Endian.Little
            )
            return float('{0:.2f}'.format(decoder.decode_16bit_int()))

        else:
            # Error handling.
            print("Error: Connection to the registers could not be established.")
            return None


def main():

    gripper = Modbus(ip="192.168.1.1", unit=65, port=502)
    time.sleep(1)
    gripper.command(targetWidth=850, targetForce=10, command=1)
    while True:
        gripper.read()
        time.sleep(0.1)


if __name__ == '__main__':
    main()
