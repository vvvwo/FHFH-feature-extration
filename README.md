# FHFH-feature-extration
Based on the pcl, we extract FPFH features from a point cloud and store into a file

The project is generated based on VS2022 and PCL 1.8.1.

The input point cloud file should be .obj

To improve the computation speed of FPFH extraction, we use openmp version of PCL to compute FPFH. Therefore, your platform should suppost openmp.

The EXE file is also provided for quick use of our code.
