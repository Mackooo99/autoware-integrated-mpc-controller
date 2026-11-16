import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/nit-user/control_benchmark/monitor_ws/install/control_benchmark_monitor'
