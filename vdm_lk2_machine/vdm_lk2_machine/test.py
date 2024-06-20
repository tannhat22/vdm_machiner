from mcprotocol.type1e import Type1E

pyPLC = Type1E('F')
pyPLC.connect('192.168.1.251', 8000)
print("Connect success")
a = pyPLC.batchread_wordunits('D700', 7)
print(len(a))
print('data: ',a)
