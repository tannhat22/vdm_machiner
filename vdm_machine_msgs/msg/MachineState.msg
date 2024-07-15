uint8 MACHINE_OFF = 0
uint8 MACHINE_UNDERLOAD = 1
uint8 MACHINE_NOLOAD = 2
uint8 MACHINE_OVERLOAD = 3
uint8 MACHINE_REMAIN_MATERIAL = 0
uint8 MACHINE_OUT_OF_MATERIAL = 1

string name
string type
int32 signal_light
int32 material
int32 underload
int32 noload
int32 error
int32 offtime
# ValueSetting value_setting
# int32 time_reachspeed