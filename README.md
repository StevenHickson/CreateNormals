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
The third line are the semantic labels that are considered flat (i.e. floors). Ex 4,11,21.

The list file is comma-delimited depth_file,labels_file,output_file.

# Python library/use

An example of using the library in python is shown in the Test_Normals.ipynb file utilizing the python directory. It is a little slower than the normal method and only runs at 10FPS on my machine.
Full details of the in depth ctypes code can be found in the [python Readme](https://raw.githubusercontent.com/StevenHickson/CreateNormals/master/python/README.md)
The gist of it is:
```
import sys
sys.path.append('/home/steve/git/CreateNormals/')
from python.calc_normals import NormalCalculation

camera_params = [2.3844389626620386e+02,0,3.1304475870804731e+02,0,5.8269103270988637e+02,2.4273913761751615e+02,0,0,1]
normal_params = [5,0.02,30]
flat_labels = [4,11,15,19,21,28,34,36,37,45,64]
norm_calc = NormalCalculation(camera_params, normal_params, flat_labels)
new_normals = norm_calc.Calculate(depth, labels)
```
