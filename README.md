# CreateNormals

[The paper can be found here](https://arxiv.org/abs/1906.06792)


This Contains the CreateNormals library. Using a depth and semantic label pair of images, we create a denoised and semantically corrected surface normals image.

![alt text](https://raw.githubusercontent.com/StevenHickson/CreateNormals/master/example.png)

Above is the visualization of different ways of computing the "ground truth" normals. Top left: a sample image from the
NYUDv2 dataset. Top-right: computed using method similar to [4] with a small window. Bottom-left: results of our
method using larger depth-adaptive smoothing. Bottomright: results of our method after semantic smoothing (if
labels are available). Note that the back and right wall are
cleaned up to a large degree due to this correction.


This library requires CMake, PCL, and Opencv. To compile run:
```
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
./CreateNormalsMain ../nyu_params.txt ../nyu_list.txt
```

The example params and list are included to see how the example script runs.
Params is set up as two lines. The first line is the camera parameters flattened as a vector. Fx, scew, Cx, 0, Fy, Cy, 0, 0, scale.
The second line are the surface normal computation parameters. These are the inpainting window (we use 5), the normal max depth change factor (we use 0.02), and the normal adaptive smoothing size (we use 30 for Kinect1/2 data and 10 for synthetic data).
The third line is the mapping of semantic labels that are considered flat (i.e. floors). 1 is a semantic label that is flat, 0 is not.

The list file is comma-delimited depth_file,labels_file,output_file.
