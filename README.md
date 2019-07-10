# CreateNormals

[The paper can be found here](https://arxiv.org/abs/1906.06792)


This Contains the CreateNormals library. Using a depth and semantic label pair of images, we create a denoised and semantically corrected surface normals image.

![alt text](https://raw.githubusercontent.com/StevenHickson/CreateNormals/master/example.png)

Above is the visualization of different ways of computing the "ground truth" normals. Top left: a sample image from the
NYUDv2 dataset. Top-right: computed using method similar to [4] with a small window. Bottom-left: results of our
method using larger depth-adaptive smoothing. Bottomright: results of our method after semantic smoothing (if
labels are available). Note that the back and right wall are
cleaned up to a large degree due to this correction.


# Compiling

This library requires CMake, PCL, and Opencv. To compile run:
```
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
./CreateNormalsMain ../nyu_params.txt ../nyu_list.txt
```

# How to use command line

The example params and list are included to see how the example script runs.
Params is set up as two lines. The first line is the camera parameters flattened as a vector. Fx, scew, Cx, 0, Fy, Cy, 0, 0, scale.
The second line are the surface normal computation parameters. These are the inpainting window (we use 5), the normal max depth change factor (we use 0.02), and the normal adaptive smoothing size (we use 30 for Kinect1/2 data and 10 for synthetic data).
The third line is the mapping of semantic labels that are considered flat (i.e. floors). 1 is a semantic label that is flat, 0 is not.

The list file is comma-delimited depth_file,labels_file,output_file.

# Python library/use

An example of using the library in python is shown in the Test_Normals.ipynb file. It is a little slower than the normal method and only runs at 10FPS on my machine.
The gist of it is:
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
normal_params = [5,0.02,30]
flat_labels = [0,0,0,0,1,0,0,0,0,0,0,1,0,0,0,1,0,0,0,1,0,1,0,0,0,0,0,0,1,0,0,0,0,0,1,0,1,1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
flat_labels_bool = [bool(x) for x in flat_labels]
new_normals = np.zeros(image.shape, dtype=np.float32)

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
