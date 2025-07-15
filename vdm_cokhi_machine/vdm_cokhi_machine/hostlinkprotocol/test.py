from hostlink import HostLink, DeviceContext


def hostLink_test(plctype, ip, port):
    pyplc = HostLink(plctype)
    pyplc.connect(ip, port)
    # check access to word unit
    # pyplc.write_data("DM20001", "U", 1000)
    testDevice = DeviceContext("ZF5120", 6, ".F")

    value = pyplc.continuous_read_data(testDevice)
    print(value)

    # check access to bit unit
    # odd size test
    # pyplc.write_data("MR1000", "U", 1)
    # value = pyplc.read_data("MR1000", "U")
    # print(value)

    # check access to word units
    # pyplc.continuous_write_data("DM20010", 5, "U", [1, 2, 3, 4, 5])
    # value = pyplc.continuous_read_data("DM20010", 5, "U")
    # print(value)

    # check access to bit units
    # odd size test
    # pyplc.continuous_write_data("MR10001", 5, "U", [1, 1, 0, 0, 1])
    # value = pyplc.continuous_read_data("MR10001", 5, "U")
    # print(value)

    # pyplc.change_mode_CPU(1)


if __name__ == "__main__":
    plctype, ip, port = ["KV", "192.168.1.1", 8501]
    hostLink_test(plctype, ip, port)
