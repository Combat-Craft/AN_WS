import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/tasc/AN_WS-1/src/cam_stream/install/cam_stream'
