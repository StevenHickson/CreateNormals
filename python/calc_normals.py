import numpy as np
import ctypes
import os

class NormalCalculation:
    def __init__(self, camera_params, normal_params, flat_labels):
        self.camera_params = camera_params
        self.normal_params = normal_params
        self.flat_labels = flat_labels
        lib_file = os.path.split(os.path.realpath(__file__))[0] + '/../build/libCreateNormals.so'
        
        self.normals_lib = ctypes.CDLL(lib_file)
        self.normals_lib.CalculateNormals.argtypes = (ctypes.POINTER(ctypes.c_float),
                                      ctypes.c_int,
                                      ctypes.POINTER(ctypes.c_float),
                                      ctypes.c_int,
                                      ctypes.POINTER(ctypes.c_int),
                                      ctypes.c_int,
                                      ctypes.c_int,
                                      ctypes.c_int,
                                      ctypes.POINTER(ctypes.c_uint16),
                                      ctypes.POINTER(ctypes.c_uint16),
                                      ctypes.POINTER(ctypes.c_float))
        
    def Calculate(self, depth, labels):
        shape = depth.shape
        normals = np.zeros(shape + (3,), dtype=np.float32)
        height, width = shape
        array_camera_params = ctypes.c_float * len(self.camera_params)
        array_normal_params = ctypes.c_float * len(self.normal_params)
        array_flat_labels = ctypes.c_int * len(self.flat_labels)
        # We need depth and labels to be cast as uint16
        self.normals_lib.CalculateNormals(array_camera_params(*self.camera_params),
                             ctypes.c_int(len(self.camera_params)),
                             array_normal_params(*self.normal_params),
                             ctypes.c_int(len(self.normal_params)),
                             array_flat_labels(*self.flat_labels),
                             ctypes.c_int(len(self.flat_labels)),
                             ctypes.c_int(width),
                             ctypes.c_int(height),
                             depth.astype(np.uint16).ctypes.data_as(ctypes.POINTER(ctypes.c_uint16)),
                             labels.astype(np.uint16).ctypes.data_as(ctypes.POINTER(ctypes.c_uint16)),
                             normals.ctypes.data_as(ctypes.POINTER(ctypes.c_float)))
        return normals
