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
        _fields_  = [   ("x", c_double),
                        ("y", c_double), 
                        ("z", c_double)
                    ]

class receive(Structure):
        _pack_ = 1
        _fields_ =  [   ("ImuData", imu),
                        ("OdometryData", odo),
                        ("heading", c_double)
                    ]
