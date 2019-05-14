from ctypes import *

class message(Structure):
	_pack_=1
	_fields_ = [	("velocity", c_double), 
			("theta", c_double), 
                        ("rst", c_uint8)
                    ]

class imu(Structure):
        _pack_ = 1
        _fields_ = [    ("ax", c_double), 
                        ("ay", c_double), 
                        ("az", c_double),
                        ("mx", c_double), 
                        ("my", c_double), 
                        ("mz", c_double)
                    ]
class odo(Structure):
        _pack_= 1
        _fields_  = [   ("x pos", c_double),
                        ("y pos", c_double), 
                        ("z pos", c_double)
                    ]

class receive(Structure):
        _pack_ = 1
        _fields_ =  [   ("Imu Data", POINTER(imu)),
                        ("Odometry Data", POINTER(odo)),
                        ("heading", c_double)
                    ]
