import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/hasanakhiar/Codes/ProjectAltair-Recruitment/Week 3/ROS/install/Task4'
