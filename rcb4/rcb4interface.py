from enum import Enum
import select
from threading import Lock
import time

import numpy as np
import serial
import serial.tools.list_ports

from rcb4.asm import encode_servo_ids_to_5bytes_bin
from rcb4.asm import encode_servo_positions_to_bytes
from rcb4.asm import encode_servo_velocity_and_position_to_bytes
from rcb4.asm import rcb4_checksum
from rcb4.asm import rcb4_servo_svector
from rcb4.asm import rcb4_velocity


class CommandTypes(Enum):
    Move = 0x00
    Jump = 0x0B
    Call = 0x0C
    SingleServo = 0x0F
    MultiServoSingleVelocity = 0x10
    MultiServoMultiVelocities = 0x11
    ServoParam = 0x12
    Version = 0xFD
    AckCheck = 0xFE
    _None = 0xFF


class ServoParams(Enum):
    Stretch = 0x01
    Speed = 0x02
    CurrentLimit = 0x03
    TemperatureLimit = 0x04


class SubMoveCmd(Enum):
    RamToCom = 0x20
    ComToRam = 0x02
    DeviceToCom = 0x21
    ComToDevice = 0x12


rcb4_dof = 36  # servo 35 + 1


class RCB4Interface(object):

    def __init__(self):
        self.lock = Lock()
        self.serial = None
        self.servo_sorted_ids = None
        self.wheel_servo_sorted_ids = None
        self._joint_to_actuator_matrix = None
        self._actuator_to_joint_matrix = None

    def __del__(self):
        self.close()

    def open(self, port='/dev/ttyUSB0',
             baudrate=1250000, timeout=0.01):
        """Opens a serial connection to the RCB4 device.

        Parameters
        ----------
        port : str
            The port name to connect to.
        baudrate : int
            The baud rate for the serial connection.
        timeout : float, optional
            The timeout for the serial connection in seconds (default is 0.01).

        Returns
        -------
        None

        Raises
        ------
        serial.SerialException
            If there is an error opening the serial port.
        """
        try:
            self.serial = serial.Serial(port, baudrate,
                                        parity='E',
                                        stopbits=1,
                                        timeout=timeout)
            print(f"Opened {port} at {baudrate} baud")
        except serial.SerialException as e:
            print(f"Error opening serial port: {e}")
            raise serial.SerialException(e)
        ack = self.check_ack()
        if ack is not True:
            return False
        self.check_firmware_version()
        self.search_servo_ids()
        return True

    def auto_open(self):
        vendor_id = 0x165c
        product_id = 0x0008
        ports = serial.tools.list_ports.comports()
        for port in ports:
            if port.vid == vendor_id and port.pid == product_id:
                return self.open(port=port.device)

    def close(self):
        if self.serial:
            self.serial.close()
        self.serial = None

    def is_opened(self):
        return self.serial is not None

    def serial_write(self, byte_list, timeout=10):
        if self.serial is None:
            raise RuntimeError('Serial is not opened.')

        data_to_send = bytes(byte_list)
        with self.lock:
            try:
                self.serial.flushInput()
                self.serial.write(data_to_send)
            except serial.SerialException as e:
                self.close()
                print(f"Error sending data: {e}")
            ret = self.serial_read(timeout)
        return ret

    def serial_read(self, timeout=10):
        if self.serial is None:
            raise RuntimeError('Serial is not opened.')

        self.serial.flushInput()
        start_time = time.time()
        read_data = b''
        while True:
            ready, _, _ = select.select(
                [self.serial], [], [], timeout - (time.time() - start_time))
            if not ready:
                self.close()
                raise serial.SerialException("Timeout: No data received.")

            chunk = self.serial.read(self.serial.in_waiting or 1)
            if not chunk:
                self.close()
                raise serial.SerialException(
                    "Timeout: Incomplete data received.")
            read_data += chunk
            if len(read_data) > 0 and read_data[0] == len(read_data):
                return read_data[1:len(read_data) - 1]

    def get_version(self):
        byte_list = [0x03, CommandTypes.Version.value, 0x00]
        return self.serial_write(byte_list)

    def get_ack(self):
        byte_list = [0x04, CommandTypes.AckCheck.value, 0x06, 0x08]
        return self.serial_write(byte_list)

    def check_ack(self):
        ack_byte_list = self.get_ack()
        return ack_byte_list[1] == 0x06

    def check_firmware_version(self):
        version = self.get_version()
        if version != b'\xfdCB-4 V1.0      090715          \xc7':
            raise RuntimeError('The firmware version is inconsistent. '
                               'Perhaps are you using not RCB-4mini?'
                               ' https://kondo-robot.com/product/rcb4_mini')

    @staticmethod
    def move_ram_to_com_command(scr_addr, src_data_size):
        """Create a command to transfer data from RAM to COM (RAM ==> COM).

        Parameters
        ----------
        scr_addr : int
            The starting address of the data to retrieve.
        src_data_size : int
            The number of bytes of data to retrieve.

        Returns
        -------
        return_data_size : int
            The number of bytes of received data.
        byte_list : list
            The command data array for transmission.

        Notes
        -----
        This method constructs a command to transfer data from RAM to COM.
        The `byte_list` list is initialized and filled with specific values to
        form the command. These include the command type and sub-command
        for RAM to COM transfer, the source address, and the size of the data
        to transfer. The checksum of the command is calculated and appended
        to `byte_list` as the last element. The `returnDataSize` is calculated
        by adding 3 to the `src_data_size`, accounting for additional bytes
        in the response.

        """
        byte_list = []
        byte_list.append(0x0A)
        byte_list.append(CommandTypes.Move.value)
        byte_list.append(SubMoveCmd.RamToCom.value)
        byte_list.append(0x00)
        byte_list.append(0x00)
        byte_list.append(0x00)
        byte_list.append(scr_addr & 0xFF)  # Source address low byte
        byte_list.append((scr_addr >> 8) & 0xFF)  # Source address high byte
        byte_list.append(src_data_size)  # Data size
        byte_list.append(rcb4_checksum(byte_list))
        return_data_size = src_data_size + 3  # Total bytes of received data
        return return_data_size, byte_list

    def reference_angle_vector(self, servo_ids=None):
        if servo_ids is None:
            servo_ids = self.search_servo_ids()
        if len(servo_ids) == 0:
            return np.empty(shape=0)
        ref_angles = self._angle_vector('reference')[servo_ids]
        return ref_angles

    def servo_error(self, servo_ids=None):
        if servo_ids is None:
            servo_ids = self.search_servo_ids()
        if len(servo_ids) == 0:
            return np.empty(shape=0)
        error_angles = self._angle_vector('error')[servo_ids]
        return error_angles

    def servo_id_to_index(self, servo_id):
        if self.valid_servo_ids([servo_id]):
            return self.sequentialized_servo_ids([servo_id])[0]

    def sequentialized_servo_ids(self, servo_ids):
        if len(servo_ids) == 0:
            return np.empty(shape=0, dtype=np.uint8)
        return self._servo_id_to_sequentialized_servo_id[
            np.array(servo_ids)].astype(np.uint8)

    def _angle_vector(self, slot='current'):
        rcb4_dof = 35
        offset = 2
        avs = np.zeros((rcb4_dof, 3))
        vi = 0
        size = 126
        for j in range(rcb4_dof // 7):
            ram = 0x90 + (20 * j * 7) + offset
            while True:
                _, byte_list = self.move_ram_to_com_command(
                    ram, size)
                try:
                    byte_list = self.serial_write(byte_list, timeout=0.1)[1:]
                except serial.SerialException as _:  # NOQA
                    self.auto_open()
                    self.serial.flushInput()
                    continue
                if len(byte_list) == size:
                    break
            seg = np.frombuffer(byte_list, dtype='<u2')
            for i in range(7):
                avs[vi, 0] = seg[i * 10]
                avs[vi, 1] = seg[i * 10 + 1]
                avs[vi, 2] = seg[i * 10 + 2]
                vi += 1
        if slot == 'current':
            return avs[:, 1]
        elif slot == 'reference':
            return avs[:, 2]
        elif slot == 'error':
            return avs[:, 0]
        else:
            raise ValueError('slot should be ["current", "reference", "error"]'
                             ' not {}'.format(slot))

    def _send_angle_vector(self, av, servo_ids=None, velocity=127):
        if servo_ids is None:
            servo_ids = self.search_servo_ids()
        if len(av) != len(servo_ids):
            raise ValueError(
                'Length of servo_ids and angle_vector must be the same.')
        av = np.array(av)
        svs = self.angle_vector_to_servo_angle_vector(av, servo_ids)
        return self.servo_angle_vector(
            servo_ids, svs, velocity=velocity)

    def angle_vector(self, av=None, servo_ids=None, velocity=127):
        if av is not None:
            return self._send_angle_vector(av, servo_ids, velocity)
        all_servo_ids = self.search_servo_ids()
        if len(all_servo_ids) == 0:
            return np.empty(shape=0)
        av = np.append(self._angle_vector()[all_servo_ids], 1)
        av = np.matmul(av.T, self.actuator_to_joint_matrix.T)[:-1]
        if servo_ids is not None:
            if len(servo_ids) == 0:
                return np.empty(shape=0)
            av = av[self.sequentialized_servo_ids(servo_ids)]
        return av

    def angle_vector_to_servo_angle_vector(self, av, servo_ids=None):
        if servo_ids is None:
            servo_ids = self.search_servo_ids()
        if len(av) != len(servo_ids):
            raise ValueError(
                'Length of servo_ids and angle_vector must be the same.')
        if len(servo_ids) == 0:
            return np.empty(shape=0)
        seq_indices = self.sequentialized_servo_ids(servo_ids)
        tmp_av = np.append(np.zeros(len(self.servo_sorted_ids)), 1)
        tmp_av[seq_indices] = np.array(av)
        return np.matmul(self.joint_to_actuator_matrix, tmp_av)[seq_indices]

    def search_servo_ids(self):
        if self.servo_sorted_ids is not None:
            return self.servo_sorted_ids
        av = self._angle_vector()
        servo_indices = np.where(av > 0)
        if len(servo_indices) > 0:
            servo_indices = servo_indices[0]
        else:
            servo_indices = []

        self.servo_sorted_ids = servo_indices
        self._servo_id_to_sequentialized_servo_id = np.nan * np.ones(rcb4_dof)
        servo_indices = np.array(servo_indices)
        if len(servo_indices):
            self._servo_id_to_sequentialized_servo_id[servo_indices] = \
                np.arange(len(servo_indices))
        self.joint_to_actuator_matrix
        return servo_indices

    def valid_servo_ids(self, servo_ids):
        return np.isfinite(self._servo_id_to_sequentialized_servo_id[
            np.array(servo_ids)])

    def hold(self, servo_ids=None):
        if servo_ids is None:
            servo_ids = self.servo_sorted_ids
        servo_vector = [32767] * len(servo_ids)
        return self.servo_angle_vector(servo_ids,
                                       servo_vector,
                                       velocity=127)

    def free(self, servo_ids=None):
        if servo_ids is None:
            servo_ids = self.servo_sorted_ids
        servo_vector = [32768] * len(servo_ids)
        return self.servo_angle_vector(servo_ids,
                                       servo_vector,
                                       velocity=127)

    def neutral(self, servo_ids=None, velocity=127):
        if servo_ids is None:
            servo_ids = self.servo_sorted_ids
        av = [0] * len(servo_ids)
        return self.angle_vector(av, servo_ids,
                                 velocity=velocity)

    def servo_angle_vector(self, servo_ids, servo_vector, velocity=127):
        """Sends a command to control multiple servos.

        This function sorts the servo IDs and corresponding angles,
        constructs a command byte list, and sends it.
        The velocity parameter is clamped between 1 and 255.

        Parameters
        ----------
        servo_ids : array_like
            Array of servo IDs. Each ID corresponds to a specific servo.
        servo_vector : array_like
            Array of angles (in servo pulse) for the servos.
            Each angle corresponds to the servo ID at the same index
            in servo_ids.
        velocity : int, or array like optional
            Velocity for the servo movement, clamped between 1 and 255.
            Default value is 127.

        Raises
        ------
        ValueError
            If the length of `servo_ids` does not match the length
            of `servo_vector`.

        Notes
        -----
        The function internally sorts `servo_ids` and `servo_vector`
        based on the servo IDs to maintain the correspondence
        between each servo ID and its angle. This sorted order is
        used for constructing the command byte list.
        """
        if len(servo_ids) != len(servo_vector):
            raise ValueError(
                'Length of servo_ids and servo_vector must be the same.')

        # Sort the servo vectors based on servo IDs
        sorted_indices = np.argsort(servo_ids)
        sorted_servo_ids = np.array(servo_ids)[sorted_indices]
        sorted_servo_vector = np.array(servo_vector)[sorted_indices]

        # Prepare the command byte list
        if isinstance(velocity, list) or isinstance(velocity, tuple) \
           or isinstance(velocity, np.ndarray):
            sorted_servo_velocities = np.array(velocity)[sorted_indices]
            byte_list = [CommandTypes.MultiServoMultiVelocities.value] \
                + encode_servo_ids_to_5bytes_bin(sorted_servo_ids) \
                + encode_servo_velocity_and_position_to_bytes(
                    sorted_servo_velocities, sorted_servo_vector)
        else:
            byte_list = [CommandTypes.MultiServoSingleVelocity.value] \
                + encode_servo_ids_to_5bytes_bin(sorted_servo_ids) \
                + [rcb4_velocity(velocity)] \
                + encode_servo_positions_to_bytes(sorted_servo_vector)

        # Add header (length) and checksum to the byte list
        byte_list.insert(0, 2 + len(byte_list))
        byte_list.append(rcb4_checksum(byte_list))

        # send the command
        return self.serial_write(byte_list)

    def read_stretch(self, servo_ids=None):
        if servo_ids is None:
            servo_ids = self.servo_sorted_ids
        return [self.servo_param64(sid, ['stretch_gain'])['stretch_gain'] // 2
                for sid in servo_ids]

    def send_stretch(self, value=127, servo_ids=None):
        if servo_ids is None:
            servo_ids = self.servo_sorted_ids
        if not isinstance(value, list) or not isinstance(value, tuple):
            value = [value] * len(servo_ids)
        byte_list = [CommandTypes.ServoParam.value] \
            + encode_servo_ids_to_5bytes_bin(servo_ids) \
            + [ServoParams.Stretch.value] \
            + rcb4_servo_svector(servo_ids, value)
        byte_list.insert(0, 2 + len(byte_list))
        byte_list.append(rcb4_checksum(byte_list))
        return self.serial_write(byte_list)

    @property
    def joint_to_actuator_matrix(self):
        if self._joint_to_actuator_matrix is None:
            servo_ids = self.search_servo_ids()
            servo_length = len(servo_ids)
            self._joint_to_actuator_matrix = np.zeros(
                (servo_length + 1, servo_length + 1), dtype=np.float32)
            self._joint_to_actuator_matrix[:, servo_length] = 7500
            self._joint_to_actuator_matrix[servo_length, servo_length] = 1
            for i in range(servo_length):
                self._joint_to_actuator_matrix[i, i] = 30
        return self._joint_to_actuator_matrix

    @property
    def actuator_to_joint_matrix(self):
        if self._actuator_to_joint_matrix is None:
            self._actuator_to_joint_matrix = np.linalg.inv(
                self.joint_to_actuator_matrix)
        return self._actuator_to_joint_matrix

    def servo_states(self):
        servo_on_indices = np.where(
            self.reference_angle_vector() != 32768)[0]
        if len(servo_on_indices) > 0:
            servo_on_ids = self.servo_sorted_ids[servo_on_indices]
            return servo_on_ids
        return []


if __name__ == '__main__':
    interface = RCB4Interface()
    print(interface.auto_open())

    from datetime import datetime
    while True:
        start = datetime.now()
        print(interface.angle_vector())
        end = datetime.now()
        print(end - start)
