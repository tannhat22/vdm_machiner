"""This file is collection of hostlink protocol error.

"""


class HostLinkError(Exception):
    """devicecode error. Device is not exsist.

    Attributes:
        plctype(str):       PLC type. "KV5500"
        devicename(str):    devicename. (ex: "Q", "P", both of them does not support hostlink protocol.)

    """

    def __init__(self, errorcode):
        self.errorcode = errorcode

    def __str__(self):
        return f"hostlink protocol error: error code {self.errorcode}"


class DeviceError(Exception):
    def __init__(self):
        pass

    def __str__(self):
        return "Device No. abnormal"


class ComandError(Exception):
    def __init__(self):
        pass

    def __str__(self):
        return "Command abrnormal"


class ProgramError(Exception):
    def __init__(self):
        pass

    def __str__(self):
        return "Program not registered"


class WriteDisabled(Exception):
    def __init__(self):
        pass

    def __str__(self):
        return "Write disabled"


class HostError(Exception):
    def __init__(self):
        pass

    def __str__(self):
        return "Host error"


class NoComment(Exception):
    def __init__(self):
        pass

    def __str__(self):
        return "No comment"


def check_hostlink_error(status: str, isHasOkResp: bool):
    """Check hostlink protocol command error.
    If errot exist(status != 0), raise Error.

    """
    if isHasOkResp:
        match status:
            case "OK":
                return None
            case "E0":
                raise DeviceError
            case "E1":
                raise ComandError
            case "E2":
                raise ProgramError
            case "E4":
                raise WriteDisabled
            case "E5":
                raise HostError
            case "E6":
                raise NoComment
            case default:
                raise HostLinkError(status)
    else:
        match status:
            case "E0":
                raise DeviceError
            case "E1":
                raise ComandError
            case "E2":
                raise ProgramError
            case "E4":
                raise WriteDisabled
            case "E5":
                raise HostError
            case "E6":
                raise NoComment
            case default:
                return None
