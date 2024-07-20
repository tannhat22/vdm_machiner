# from mcprotocol.type1e import Type1E

# pyPLC = Type1E('F')
# pyPLC.connect('192.168.1.251', 8000)
# print("Connect success")
# a = pyPLC.batchread_wordunits('D700', 7)
# print(len(a))
# print('data: ',a)

from datetime import datetime

# Giả sử bạn có một dictionary như sau
data = {
    '10/07/2024-DAY': 'value1',
    '25/02/2024-NIGHT': 'value2',
    '23/08/2024-DAY': 'value3',
    '30/08/2024-DAY': 'value3',
    '01/03/2024-DAY': 'value3',
    '30/05/2024-DAY': 'value3',
    '22/03/2024-DAY': 'value3',
    '30/12/2024-DAY': 'value3',
    '30/11/2024-DAY': 'value3',
}

# Hàm để tách và chuyển đổi ngày
def extract_date(key):
    date_str = key.split('-')[0]  # Lấy phần ngày
    return datetime.strptime(date_str, '%d/%m/%Y')

# Sắp xếp dictionary
sorted_data = dict(sorted(data.items(), key=lambda item: extract_date(item[0])))

# In ra kết quả
print(sorted_data)