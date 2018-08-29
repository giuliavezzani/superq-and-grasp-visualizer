# superq-and-grasp-visualizer
Read or receive the object point cloud and visualize the superquadrics estimated by `superquadric-model` and the grasping poses computed by `superquadric-grasp` using VTK. Thus, the module manages the visualization and queries the modules for computing the object model and the grasping poses.

### Dependencies
- [Yarp](https://github.com/robotology/yarp)
- [iCub](https://github.com/robotology/icub-main)
- [Ipopt](https://github.com/coin-or/Ipopt)
- [VTK](https://github.com/Kitware/VTK)

### Command-line options
- `--file file-name`: specify the file containing the point cloud given in the following plain format:
  ```
  x0 y0 z0 [r0 g0 b0]
  x1 y1 z1 [r1 g1 b1]
  ...
  ```
  RGB colors are optional.
- `--remove-outliers "(<radius> <minpts>)"`: outliers removal based on spatial density clustering. The aggregation of points in clusters is regulated through the distance _radius_, whereas _minpts_ represents the minimum number of points of a valid cluster. Only points belonging to the largest cluster will survive as inliers.
- `--uniform-sample <int>`: specify the integer step for performing uniform down-sampling as follows:
  - `1` means no down-sampling
  - `> 1` enables down-sampling
- `--random-sample <double>`: specify the percentage in [0,1] for performing random down-sampling.
- `--disable-viewer`: specify not to launch the viewer.
- `--background-color "(<r> <g> <b>)"`: change background color by specifying RGB components as double in the range [0,1]. 

### Real-time mode
If no `--file` option is passed through the command line, the module will open up a port called `/find-superquadric/points:rpc` to which the point cloud can be sent as a `yarp::sig::PointCloud<yarp::sig::DataXYZRGBA>` object.


### Example
```
$ superq-and-grasp-visualizer --remove-outliers "(0.01 10)" --random-sample 0.2 --file ./data/cylinder
```

