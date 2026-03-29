import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/gastrobot/gastrobot_ws/install/gastrobot_perception'
