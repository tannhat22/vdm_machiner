"""This file defines mcprotocol constant.

"""

# PLC definetion
KV_SERIES = "KV"


class ModelNoPlc:
    """This class defines hostlink protocol model No. of PLC.

    Attributes:
        KV_700(int):  KV_700 code (48)

    """

    # These device supports KV series
    KV_700_NEM = "KV_700_NON_EM"
    KV_700_EM = "KV_700_EM"
    KV_1000 = "KV_1000"
    KV_3000 = "KV_3000"
    KV_5000 = "KV_5000"
    KV_5500 = "KV_5500"
    KV_7500 = "KV_7500"
    KV_8500 = "KV_8500"

    def __init__(self):
        """Constructor"""
        pass

    @staticmethod
    def get_str_modelno(model_no):
        """Static method that returns model of PLC from model no.

        Args:
            model_no(int):    model No. of PLC

        Returns:
            plc_model(str):   model of PLC
        """
        if model_no == 48:
            return ModelNoPlc.KV_700_NEM
        elif model_no == 49:
            return ModelNoPlc.KV_700_EM
        elif model_no == 50:
            return ModelNoPlc.KV_1000
        elif model_no == 51:
            return ModelNoPlc.KV_3000
        elif model_no == 52:
            return ModelNoPlc.KV_5000
        elif model_no == 53:
            return ModelNoPlc.KV_5500
        elif model_no == 54:
            return ModelNoPlc.KV_7500
        elif model_no == 55:
            return ModelNoPlc.KV_8500
        else:
            raise ValueError(f"Model no invalid: [{model_no}], can't find model of PLC!")


class DeviceCodeError(Exception):
    """devicecode error. Device is not exsist.

    Attributes:
        plctype(str):       PLC type. "KV",
        devicename(str):    devicename
    """

    def __init__(self, plctype, devicename):
        self.plctype = plctype
        self.devicename = devicename

    def __str__(self):
        error_txt = "devicename: {} is not support {} series PLC".format(
            self.devicename, self.plctype
        )
        return error_txt


class DeviceConstants:
    """This class defines hostlink protocol device constatnt.

    Attributes:
        R_DEVICE(str):  R devide code (R)

    """

    # These device supports KV series
    R_DEVICE = "R"
    B_DEVICE = "B"
    MR_DEVICE = "MR"
    LR_DEVICE = "LR"
    CR_DEVICE = "CR"
    T_DEVICE = "T"
    C_DEVICE = "C"
    VB_DEVICE = "VB"
    DM_DEVICE = "DM"
    EM_DEVICE = "EM"
    FM_DEVICE = "FM"
    ZF_DEVICE = "ZF"
    W_DEVICE = "W"
    TM_DEVICE = "TM"
    Z_DEVICE = "Z"
    TC_DEVICE = "TC"
    TS_DEVICE = "TS"
    CC_DEVICE = "CC"
    CS_DEVICE = "CS"
    CTH_DEVICE = "CTH"
    CTC_DEVICE = "CTC"
    AT_DEVICE = "AT"
    CM_DEVICE = "CM"
    VM_DEVICE = "VM"

    # BIT_DEVICE = "bit"
    # WORD_DEVICE = "word"

    def __init__(self):
        """Constructor"""
        pass

    @staticmethod
    def get_str_devicecode(plctype, devicename):
        """Static method that returns devicecode from device name.

        Args:
            plctype(str):       PLC type. "KV5500"
            devicename(str):    Device name. (ex: "MR", "DM", "W")

        Returns:
            devicecode(str):    Device code defined hostlink protocol (ex: "MR" → "MR")
        """
        if devicename == "R":
            return DeviceConstants.R_DEVICE
        elif devicename == "B":
            return DeviceConstants.B_DEVICE
        elif devicename == "MR":
            return DeviceConstants.MR_DEVICE
        elif devicename == "LR":
            return DeviceConstants.LR_DEVICE
        elif devicename == "CR":
            return DeviceConstants.CR_DEVICE
        elif devicename == "T":
            return DeviceConstants.T_DEVICE
        elif devicename == "C":
            return DeviceConstants.C_DEVICE
        elif devicename == "VB":
            return DeviceConstants.VB_DEVICE
        elif devicename == "DM":
            return DeviceConstants.DM_DEVICE
        elif devicename == "EM":
            return DeviceConstants.EM_DEVICE
        elif devicename == "FM":
            return DeviceConstants.FM_DEVICE
        elif devicename == "ZF":
            return DeviceConstants.ZF_DEVICE
        elif devicename == "W":
            return DeviceConstants.W_DEVICE
        elif devicename == "TM":
            return DeviceConstants.TM_DEVICE
        elif devicename == "Z":
            return DeviceConstants.Z_DEVICE
        elif devicename == "TC":
            return DeviceConstants.TC_DEVICE
        elif devicename == "TS":
            return DeviceConstants.TS_DEVICE
        elif devicename == "CC":
            return DeviceConstants.CC_DEVICE
        elif devicename == "CS":
            return DeviceConstants.CS_DEVICE
        elif devicename == "CTH":
            return DeviceConstants.CTH_DEVICE
        elif devicename == "CTC":
            return DeviceConstants.CTC_DEVICE
        elif devicename == "AT":
            return DeviceConstants.AT_DEVICE
        elif devicename == "CM":
            return DeviceConstants.CM_DEVICE
        elif devicename == "VM":
            return DeviceConstants.VM_DEVICE
        else:
            raise DeviceCodeError(plctype, devicename)
