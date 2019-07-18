# Internal python details

The code we use can be seen in the calc_normals.py file.
The way it works is shown below:

```
import ctypes
import numpy as np

normals_lib = ctypes.CDLL('build/libCreateNormals.so')
normals_lib.CalculateNormals.argtypes = (ctypes.POINTER(ctypes.c_float),
                                      ctypes.c_int,
                                      ctypes.POINTER(ctypes.c_float),
                                      ctypes.c_int,
                                      ctypes.POINTER(ctypes.c_bool),
                                      ctypes.c_int,
                                      ctypes.c_int,
                                      ctypes.c_int,
                                      ctypes.POINTER(ctypes.c_uint16),
                                      ctypes.POINTER(ctypes.c_uint16),
                                      ctypes.POINTER(ctypes.c_float))
                                      
camera_params = [2.3844389626620386e+02,0,3.1304475870804731e+02,0,5.8269103270988637e+02,2.4273913761751615e+02,0,0,1]
normal_params = [5,0.02,30,0.04]
flat_labels = [4,11,21]
flat_labels_bool = [bool(x) for x in flat_labels]
new_normals = np.zeros(labels.shape + (3,), dtype=np.float32)

height, width = labels.shape
array_camera_params = ctypes.c_float * len(camera_params)
array_normal_params = ctypes.c_float * len(normal_params)
array_flat_labels = ctypes.c_bool * len(flat_labels_bool)

normals_lib.CalculateNormals(array_camera_params(*camera_params),
                             ctypes.c_int(len(camera_params)),
                             array_normal_params(*normal_params),
                             ctypes.c_int(len(normal_params)),
                             array_flat_labels(*flat_labels_bool),
                             ctypes.c_int(len(flat_labels_bool)),
                             ctypes.c_int(width),
                             ctypes.c_int(height),
                             depth.ctypes.data_as(ctypes.POINTER(ctypes.c_uint16)),
                             labels.ctypes.data_as(ctypes.POINTER(ctypes.c_uint16)),
                             new_normals.ctypes.data_as(ctypes.POINTER(ctypes.c_float)))
```

