#define BOOST_USE_WINDOWS_H
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/normal_3d.h>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>

using namespace std;


void save_FPFH(vector<vector<float>> FPFH_descriptor, string Path) {

    ofstream fout(Path, ios::app);
    fout << FPFH_descriptor.size() << endl;
    for (int i = 0; i < FPFH_descriptor.size(); i++) {
        for (int j = 0; j < FPFH_descriptor[i].size(); j++) {
            if (j == FPFH_descriptor[i].size() - 1) {
                fout << FPFH_descriptor[i][j] << endl;            
            }
            else {
                fout << FPFH_descriptor[i][j] << " ";            
            }                
        }        
    }    
    fout.close();

}

int main(int argc, char** argv)
{
    if (argc < 2)
    {
        std::cout << "Hello! you should input a filepath!"<< std::endl;
        return -1;
    }

    std::string fileName = argv[1];
    std::cout << "Reading " << fileName << std::endl;

    

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);    


    if (pcl::io::loadOBJFile<pcl::PointXYZ>(fileName, *cloud) == -1) // load the file
    {
        PCL_ERROR("Couldn't read file\n");
        return (-1);
    }

    int objindex = fileName.find(".obj");
    string fileNameFPFH = fileName.substr(0, objindex)+".fpfh";

    std::cout << "Loaded " << cloud->size() << " points." << std::endl;

    // Compute the normals
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimation;
    normal_estimation.setInputCloud(cloud);

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    normal_estimation.setSearchMethod(tree);

    pcl::PointCloud<pcl::Normal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::Normal>);

    normal_estimation.setRadiusSearch(0.03);

    normal_estimation.compute(*cloud_with_normals);

    // Setup the feature computation    

    pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_estimation;

    //Set thread numbers
    fpfh_estimation.setNumberOfThreads(10);

    // Provide the original point cloud (without normals)
    fpfh_estimation.setInputCloud(cloud);
    // Provide the point cloud with normals
    fpfh_estimation.setInputNormals(cloud_with_normals);

    // fpfhEstimation.setInputWithNormals(cloud, cloudWithNormals); PFHEstimation does not have this function
    // Use the same KdTree from the normal estimation
    fpfh_estimation.setSearchMethod(tree);

    pcl::PointCloud<pcl::FPFHSignature33>::Ptr pfh_features(new pcl::PointCloud<pcl::FPFHSignature33>);

    fpfh_estimation.setRadiusSearch(0.2);

    // Actually compute the spin images
    fpfh_estimation.compute(*pfh_features);

    std::cout << "output size (): " << pfh_features->size() << std::endl;

    // Display and retrieve the shape context descriptor vector for the 0th point.
    vector<vector<float>> FPFH_descriptor(pfh_features->size());

//#pragma omp parallel for 

    for (int i = 0; i < FPFH_descriptor.size(); i++) {

        vector<float> FPFH_descriptor_i(33);
        pcl::FPFHSignature33 descriptor = (*pfh_features)[i];

        for (int j = 0; j < 33; j++) {

            FPFH_descriptor_i[j] = descriptor.histogram[j];
        
        }
        FPFH_descriptor[i] = FPFH_descriptor_i;
    }

    save_FPFH(FPFH_descriptor, fileNameFPFH);   

    return 0;
}