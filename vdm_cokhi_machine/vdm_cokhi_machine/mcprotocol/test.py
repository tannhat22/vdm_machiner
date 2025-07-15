from type1e import Type1E, DeviceContext
from datetime import datetime


def split_signed_32bit(value):
    # Chuyển giá trị âm thành unsigned 32-bit
    if value < 0:
        value += 0x100000000  # Chuyển về dạng unsigned 32-bit

    # Tách thành hai phần 16-bit
    msb = (value >> 16) & 0xFFFF  # Lấy 16 bit cao
    lsb = value & 0xFFFF  # Lấy 16 bit thấp

    # Chuyển thành số nguyên có dấu 16-bit
    if msb >= 0x8000:
        msb -= 0x10000
    if lsb >= 0x8000:
        lsb -= 0x10000

    return msb, lsb


def Type1E_test(plctype, ip, port):
    pyplc = Type1E(plctype)
    pyplc.connect(ip, port)
    # check batch access to word units

    msb, lsb = split_signed_32bit(1200)
    testDevice = DeviceContext("D5000", 1, ".L")
    pyplc.batchwrite_wordunits(testDevice, [lsb, msb])
    values = pyplc.batchread_wordunits(testDevice)
    print(values)

    current_time = datetime.now()
    pyplc.set_time(
        current_time.year - 2000,
        current_time.month,
        current_time.day,
        current_time.hour,
        current_time.minute,
        current_time.second,
        current_time.isoweekday() % 7,
    )

    # check batch access to bit units
    # odd size test
    # pyplc.batchwrite_bitunits("M220", [0])
    # value = pyplc.batchread_bitunits("M220", 1)
    # print(value)

    # even size test
    # pyplc.batchwrite_bitunits("M220", [1, 0, 1, 0])
    # value = pyplc.batchread_bitunits("M220", 4)
    # print(value)

    # test random bit access
    # pyplc.randomwrite_bitunits(["M220", "M225", "M230"], [1, 1, 1])
    # pyplc.randomwrite_wordunits(["D198", "D200", "D202"], [12, -13, 15])
    # value = pyplc.batchread_wordunits("D198", 6)
    # print(value)
    # pyplc.remote_stop()
    # pyplc.remote_run()


if __name__ == "__main__":
    plctype, ip, port = ["F", "192.168.1.251", 8000]
    Type1E_test(plctype, ip, port)
