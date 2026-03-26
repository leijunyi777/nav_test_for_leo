import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/student13/debuging/lei_github/nav_test_for_leo/install/my_robot_bringup'
