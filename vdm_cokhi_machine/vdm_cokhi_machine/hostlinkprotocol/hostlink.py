import re
import socket
import binascii
import struct

from . import hostlinkerror
from . import hostlinkconst as const


def get_device_number(device):
    """Extract device number.

    Ex: "DM10000" → "10000"
    """
    device_num = re.search(r"\d.*", device)
    if device_num is None:
        raise ValueError("Invalid device number, {}".format(device))
    else:
        device_num_str = device_num.group(0)
    return device_num_str


class DeviceContext:
    def __init__(self, headdevice: str, size: int, format: str = "", name: str = None):
        self.headdevice = headdevice
        self.size = size
        self.format = format
        self.name = name


# class CommTypeError(Exception):
#     """Communication type error. Communication type must be "binary" or "ascii" """

#     def __init__(self):
#         pass

#     def __str__(self):
#         return 'communication type must be "binary" or "ascii"'


class PLCTypeError(Exception):
    """PLC type error. PLC type must be "KV" """

    def __init__(self):
        pass

    def __str__(self):
        return "plctype must be 'KV'"


class HostLink:
    """hostlink protocol communication class.

    Attributes:
        Header:                 A header of Ethernet. Normally, it is added automatically
        timer(int):             time to raise Timeout error(/250msec). default=4(1sec)
                                If PLC elapsed this time, PLC returns Timeout answer.
                                Note: python socket timeout is always set timer+1sec. To receivee Timeout answer.
    """

    plctype = const.KV_SERIES
    timer = 4  # hostlink protocol timeout. 250msec * 4 = 1 sec
    soc_timeout = 2  # 2 sec
    _is_connected = False
    _SOCKBUFSIZE = 4096
    _answerDataIndex = -2
    _answerStatusIndex = 2
    _debug = False

    def __init__(self, plctype="KV"):
        """Constructor"""
        self._set_plctype(plctype)

    def _set_debug(self, debug=False):
        """Turn on debug mode"""
        self._debug = debug

    def connect(self, ip, port):
        """Connect to PLC

        Args:
            ip (str):       ip address(IPV4) to connect PLC
            port (int):     port number of connect PLC
            timeout (float):  timeout second in communication

        """
        self._ip = ip
        self._port = port
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._sock.settimeout(self.soc_timeout)
        self._sock.connect((ip, port))
        self._is_connected = True

    def close(self):
        """Close connection"""
        self._sock.close()
        self._is_connected = False

    def _send(self, send_data):
        """send hostlink protorocl data

        Args:
            send_data(bytes): hostlink protocol data

        """
        if self._is_connected:
            if self._debug:
                print(binascii.hexlify(send_data))
            try:
                self._sock.sendall(send_data)
            except Exception as e:
                print("Send data Error: ", e)
                self._is_connected = False
        else:
            raise Exception("socket is not connected. Please use connect method")

    def _recv(self):
        """receive hostlink protocol data

        Returns:
            recv_data
        """
        recv_data = self._sock.recv(self._SOCKBUFSIZE)
        return recv_data

    def _set_plctype(self, plctype):
        """Check PLC type. If plctype is vaild, set self.commtype.

        Args:
            plctype(str):      PLC type. "KV"

        """
        if plctype == "KV":
            self.plctype = const.KV_SERIES
        else:
            raise PLCTypeError()

    def _make_senddata(self, subheader: str, requestdata: str):
        """Makes send hostlink protorocl data.

        Args:
            requestdata(str): hostlink protocol request data.

        Returns:
            hostlink_data(str): send hostlink protocol data

        """
        hl_data = subheader + " "
        hl_data += requestdata
        hl_data += "\x0d"
        return hl_data.encode()

    def _make_devicedata(self, device):
        """make hostlink protocol device data. (device code and device number)

        Args:
            device(str): device. (ex: "D1000", "Y1")

        Returns:
            device_data(bytes): device data

        """

        device_data = ""

        devicetype = re.search(r"\D+", device)
        if devicetype is None:
            raise ValueError("Invalid device ")
        else:
            devicetype = devicetype.group(0)

        devicecode = const.DeviceConstants.get_str_devicecode(self.plctype, devicetype)

        devicenum = get_device_number(device)
        device_data += devicecode
        device_data += devicenum
        return device_data

    def _check_cmdanswer(self, recv_data: bytes, isHasOkResp: bool):
        """check command answer. If answer status is not OK, raise error according to answer"""
        answerstatus = recv_data.decode()[: self._answerStatusIndex]
        hostlinkerror.check_hostlink_error(answerstatus, isHasOkResp)
        return None

    def convert_unsigned_16bit_dec2char(self, data: int):
        # Chuyển đổi mỗi thanh ghi thành 2 ký tự ASCII
        string_result = chr(data >> 8) + chr(data & 0xFF)
        string_cleaned = string_result.replace("\x00", "").strip()
        return string_cleaned

    def convert_signed_32bit_dec2float(self, value: int):
        value_f = struct.unpack("!f", struct.pack("!I", value))[0]
        return value_f

    def change_mode_CPU(self, mode: int):
        """This command is used to toggle the CPU unit to PROGRAM mode or Run mode.

        Args:
            mode(int):
                When "0" is specified as the mode No., the CPU unit will switch PROGRAM mode.
                When "1" is specified, the CPU unit will switch to RUN mode.

        Returns:
            bitunits_values(list[int]):  bit units value(0 or 1) list

        """
        subheader = "M"

        send_data = subheader + str(mode) + "\x0d"
        send_data = send_data.encode()
        # send hostlink data
        self._send(send_data)
        # receive hostlink data
        recv_data = self._recv()
        self._check_cmdanswer(recv_data, True)
        return None

    def clear_error_CPU(self):
        "This command is used to clear the error occurred in CPU unit."
        subheader = "ER"

        send_data = subheader + "\x0d"
        send_data = send_data.encode()
        # send hostlink data
        self._send(send_data)
        # receive hostlink data
        recv_data = self._recv()
        self._check_cmdanswer(recv_data, True)
        return None

    def verity_error_code(self):
        """This command is used to detect error or abnormality occurred on the CPU unit.
        Returns:
            The content of the error occurred on the CPU unit is written as an error code.
            If no error occurred on the CPU unit, then "000" will be written.
        """
        subheader = "?E"

        send_data = subheader + "\x0d"
        send_data = send_data.encode()
        # send hostlink data
        self._send(send_data)
        # receive hostlink data
        recv_data = self._recv()
        self._check_cmdanswer(recv_data, False)
        error_code = recv_data.decode()[0:-2]
        return error_code

    def query_model(self):
        """This command is used to detect the model No. of PLC.
        Returns:
            The model No. of PLC is written as a number.
        """
        subheader = "?K"

        send_data = subheader + "\x0d"
        send_data = send_data.encode()
        # send hostlink data
        self._send(send_data)
        # receive hostlink data
        recv_data = self._recv()
        self._check_cmdanswer(recv_data, False)
        model_no = recv_data.decode()[0:-2]
        return const.ModelNoPlc.get_str_modelno(int(model_no))

    def verify_operation_mode(self):
        """This command is used to verify current operation status of the CPU unit.
        Returns:
            The current status (mode) of the CPU unit is written as a number. If the CPU unit is in
            PROGRAM mode or the ladder is not registered, "0" will be written. If the CPU unit is in RUN
            mode, "1" will be written.
        """
        subheader = "?M"

        send_data = subheader + "\x0d"
        send_data = send_data.encode()
        # send hostlink data
        self._send(send_data)
        # receive hostlink data
        recv_data = self._recv()
        self._check_cmdanswer(recv_data, False)
        current_mode = recv_data.decode()[0:-2]
        return current_mode

    def set_time(self, year, month, day, hour, minutes, seconds, weeks):
        """This command is used to set the time of CPU unit.
        Args:
            year: Enter the year using two digits."00" represents the year 2000. The range is 0 to 99.
            month: Enter the month using two digits. The range is 01 to 12.
            day: Enter the day using two digits. The range is 01 to 31.
            hour: Enter the hour using two digits. The range is 00 to 23.
            minutes: Enter the minutes using two digits. The range is 00 to 59.
            seconds: Enter the seconds using two digits. The range is 00 to 59.
            weeks: Enter the week using 1 digit. The table below shows the relationship between day of the week and input value
        """
        subheader = "WRT"
        data = [year, month, day, hour, minutes, seconds, weeks]

        send_data = subheader
        for i in data:
            send_data += " " + str(i)
        send_data += "\x0d"
        send_data = send_data.encode()
        # send hostlink data
        self._send(send_data)
        # receive hostlink data
        recv_data = self._recv()
        self._check_cmdanswer(recv_data, True)
        return None

    def forced_set_or_reset(self, headdevice, set: bool = True):
        """This command is used to perform forced set/reset (ON/OFF) on the specified contact of device.
        Args:
            headdevice: Set or reset head device. (ex: "MR10000")
            set: forced set if set is True else forced reset
        """
        if set:
            subheader = "ST"
        else:
            subheader = "RS"

        request_data = self._make_devicedata(headdevice)
        send_data = self._make_senddata(subheader, request_data)
        # send hostlink data
        self._send(send_data)
        # receive hostlink data
        recv_data = self._recv()
        self._check_cmdanswer(recv_data, True)
        return None

    def continuous_forced_set_or_reset(
        self,
        device: DeviceContext,
        set: bool = True,
    ):
        """This command is used to perform forced set/reset on the contacts of specified number of devices.
        Args:
            headdevice: Set or reset head device. (ex: "MR10000")
            size: number of devices
            set: forced set if set is True else forced reset
        """
        if set:
            subheader = "STS"
        else:
            subheader = "RSS"

        request_data = self._make_devicedata(device.headdevice)
        request_data += " " + str(device.size)
        send_data = self._make_senddata(subheader, request_data)
        # send hostlink data
        self._send(send_data)
        # receive hostlink data
        recv_data = self._recv()
        self._check_cmdanswer(recv_data, True)
        return None

    def read_data(self, device: DeviceContext):
        """This command is used to read data from one specified device.
        Args:
            headdevice: Read head device. (ex: "MR10000")
            data_format: The device data format (suffix) is specified as the .U/.S/.D/.L/.F/.C/.H/(no specifying) format

        Returns:
           deivce_value(int):  device value

        """
        subheader = "RD"

        request_data = self._make_devicedata(device.headdevice)
        if device.format == ".C":
            request_data += ".U"
        elif device.format == ".F":
            request_data += ".L"
        else:
            request_data += device.format
        send_data = self._make_senddata(subheader, request_data)
        # send mc data
        self._send(send_data)
        # receive mc data
        recv_data = self._recv()
        self._check_cmdanswer(recv_data, False)

        device_value = recv_data.decode()
        device_value = int(device_value[:-2])

        if device.format == ".C":
            device_value = self.convert_unsigned_16bit_dec2char(device_value)
        elif device.format == ".F":
            device_value = self.convert_signed_32bit_dec2float(device_value)

        return device_value

    def continuous_read_data(self, device: DeviceContext):
        """This command is used to read data of specified number of devices continuously.
        Args:
            headdevice: Read head device. (ex: "MR10000")
            readsize: Number of read devices
            data_format: The device data format (suffix) is specified as the .U/.S/.D/.L/.F/.C/.H/ (no specifying) format

        Returns:
           device_values(list[int]):  device values

        """
        subheader = "RDS"

        request_data = self._make_devicedata(device.headdevice)
        if device.format == ".C":
            request_data += ".U"
        elif device.format == ".F":
            request_data += ".L"
        else:
            request_data += device.format
        request_data += " " + str(device.size)
        send_data = self._make_senddata(subheader, request_data)
        # send mc data
        self._send(send_data)
        recv_data = b""
        if device.format in [".U", ".S", ".D", ".L", ".C", ".F"]:
            if device.format == ".U" or device.format == ".C":
                base_len = 6
            elif device.format == ".S":
                base_len = 7
            elif device.format == ".D":
                base_len = 11
            elif device.format == ".L" or device.format == ".F":
                base_len = 12

            while len(recv_data) < (device.size * base_len + 1):
                packet = self._recv()
                if not packet:
                    break
                recv_data += packet
                self._check_cmdanswer(recv_data, False)
        else:
            recv_data = self._recv()
            self._check_cmdanswer(recv_data, False)
        recv_data = recv_data.decode()[:-2]
        recv_data = recv_data.split(" ")
        device_values = []
        for i in range(device.size):
            value = int(recv_data[i])
            if device.format == ".C":
                device_values.append(self.convert_unsigned_16bit_dec2char(value))
            elif device.format == ".F":
                device_values.append(self.convert_signed_32bit_dec2float(value))
            else:
                device_values.append(value)

        return device_values

    def write_data(self, device: DeviceContext, value: int = 0):
        """This command is used to write data into one specified device.
        Args:
            headdevice: Write head device. (ex: "MR10000")
            data_format: The device data format (suffix) is specified as the .U/.S/.D/.L/.H/ (no specifying) format
            value: Write value
        """
        subheader = "WR"

        request_data = self._make_devicedata(device.headdevice)
        request_data += device.format
        request_data += " " + str(value)
        send_data = self._make_senddata(subheader, request_data)
        # send mc data
        self._send(send_data)
        # receive mc data
        recv_data = self._recv()
        self._check_cmdanswer(recv_data, True)
        return None

    def continuous_write_data(self, device: DeviceContext, values: list = [0]):
        """This command is used to write data of specified number of devices continuously.
        Args:
            headdevice: Write head device. (ex: "MR10000")
            writesize: Number of write devices
            data_format: The device data format (suffix) is specified as the .U/.S/.D/.L/.H/ (no specifying) format
            values: Write values
        """
        subheader = "WRS"

        request_data = self._make_devicedata(device.headdevice)
        request_data += device.format
        request_data += " " + str(device.size)
        for i in range(device.size):
            request_data += " " + str(values[i])
        send_data = self._make_senddata(subheader, request_data)
        # send mc data
        self._send(send_data)
        # receive mc data
        recv_data = self._recv()
        self._check_cmdanswer(recv_data, True)
        return None

    def monitor_register(self, headdevices: list, bit: bool = True, data_format: str = ""):
        """This command is used to register the specified device into bit device register table (MBS) or word device register table (MWS).
        Args:
            headdevices: Write head devices. (ex: ["MR10000", "MR20000"])
            bit: bit device register table or word device register table
        """
        if bit:
            subheader = "MBS"
        else:
            subheader = "MWS"

        request_data = ""
        for i in range(len(headdevices)):
            request_data += self._make_devicedata(headdevices[i])
            if not bit:
                request_data += data_format
            if i < (len(headdevices) - 1):
                request_data += " "

        send_data = self._make_senddata(subheader, request_data)
        # send mc data
        self._send(send_data)
        # receive mc data
        recv_data = self._recv()
        self._check_cmdanswer(recv_data, True)
        return None

    def read_monitor(self, bit: bool = True):
        """This command is used to display the device contents in the device register table
            MBR: To register the bit device to be monitored.
            MWR : To register the word device to be monitored.
        Args:
            bit: bit device register table or word device register table

        Returns:
           device_values(list[int]):  device values

        """
        if bit:
            subheader = "MBR"
        else:
            subheader = "MWR"

        send_data = subheader + "\x0d"
        send_data = send_data.encode()
        # send mc data
        self._send(send_data)
        # receive mc data
        recv_data = self._recv()
        self._check_cmdanswer(recv_data, False)
        recv_data = recv_data.decode()[:-2]
        recv_data = recv_data.split(" ")
        device_values = []
        for data in recv_data:
            device_values.append(int(data))
        return device_values

    def read_comment(self, headdevice):
        """This command is used to read the comment of a specified device.
        Args:
            headdevices: bit device register table or word device register table

        Returns:
          device_comment: comment of the device specified by commands (32 characters). If
          comment is less than 32 characters, the remaining characters are filled with spaces

        """
        subheader = "RDC"

        request_data = self._make_devicedata(headdevice)
        send_data = self._make_senddata(subheader, request_data)
        # send mc data
        self._send(send_data)
        # receive mc data
        recv_data = self._recv()
        self._check_cmdanswer(recv_data, False)
        recv_data = recv_data.decode()[:-2]
        return recv_data

    def switch_bank(self, bank_no: int):
        """This command is used to switch the memory bank for file registers.
        Args:
            bank_no: Bank No. will be specified for file register in the range of 0 to 3.
        """
        subheader = "BE"

        if bank_no < 0 or bank_no > 3:
            raise ValueError(f"Invalid bank_no: {bank_no}, bank_no in range of 0 to 3.")

        request_data = str(bank_no)
        send_data = self._make_senddata(subheader, request_data)
        # send mc data
        self._send(send_data)
        # receive mc data
        recv_data = self._recv()
        self._check_cmdanswer(recv_data, True)
        return None

    def read_expansion_unit_buffer_memory(
        self, unit_no: int, address: int, readsize, data_format: str = ""
    ):
        """This command is used to read continuously specified number of data from the expansion unit buffer memory.
        Args:
            unit_no: Unit No. can be specified in the range of 0 to 48
            address: Address of the expansion unit buffer memory can be specified in the range of 00000 to 32767
            readsize: Number of read devices
            data_format: The device data format (suffix) is specified as the .U/.S/.D/.L/.H/ (no specifying) format

        Returns:
           device_values(list[int]):  device values

        """
        subheader = "URD"

        request_data = str(unit_no) + " " + str(address)
        request_data += data_format
        request_data += " " + str(readsize)
        send_data = self._make_senddata(subheader, request_data)
        # send mc data
        self._send(send_data)
        # receive mc data
        recv_data = self._recv()
        self._check_cmdanswer(recv_data, False)
        recv_data = recv_data.decode()[:-2]
        recv_data = recv_data.split(" ")
        device_values = []
        for i in range(readsize):
            device_values.append(int(recv_data[i]))
        return device_values

    def read_expansion_unit_buffer_memory(
        self,
        unit_no: int,
        address: int,
        writesize,
        data_format: str = "",
        values: list = [0],
    ):
        """This command is used to specified number of data will be written into the expansion unit buffer memory.
        Args:
            unit_no: Unit No. can be specified in the range of 0 to 48
            address: Address of the expansion unit buffer memory can be specified in the range of 00000 to 32767
            readsize: Number of read devices
            data_format: The device data format (suffix) is specified as the .U/.S/.D/.L/.H/ (no specifying) format
            values: Write values
        """
        subheader = "UWR"

        request_data = str(unit_no) + " " + str(address)
        request_data += data_format
        request_data += " " + str(writesize)
        for i in range(writesize):
            request_data += " " + str(values[i])
        send_data = self._make_senddata(subheader, request_data)
        # send mc data
        self._send(send_data)
        # receive mc data
        recv_data = self._recv()
        self._check_cmdanswer(recv_data, True)
        return None
