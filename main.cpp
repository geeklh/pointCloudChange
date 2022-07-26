#include <iostream>
#include <fstream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/point_types.h>
//=======pcl
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/PolygonMesh.h>

//=========vtk

//========other
#include <vector>
#include <algorithm>
#include <time.h>

using namespace std;
int pcd2txt(string pcdfilePath, string txtfilePath){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ> );
    pcl::io::loadPCDFile(pcdfilePath, *cloud);//文件路径

    ofstream outfile;
    outfile.open(txtfilePath);

    for( size_t i = 0; i < cloud->points.size(); ++i ){
        outfile << cloud->points[i].x << "\t" << cloud->points[i].y << "\t" << cloud->points[i].z << "\n";
    }
    return 0;
}

int txt2pcd(string txtfilePath, string pcdfilePath) {
    ifstream infile;
    infile.open(txtfilePath);
    float x, y, z;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    while( infile >> x >> y >> z){
        cloud->push_back(pcl::PointXYZ(x, y, z));
    }
    pcl::io::savePCDFileBinary( pcdfilePath, *cloud);//pcd路径

    return 0;
}

int pcd2ply(string pcdfilePath, string plyfilePath) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile(pcdfilePath, *cloud);
    pcl::io::savePLYFileBinary(plyfilePath, *cloud);
    return 0;
}

int pcd2plyAngly(string pcdfilePath, string plyfilePath) {
    // Load input file
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile(pcdfilePath, *cloud);

    // Normal estimation*
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);
    n.setInputCloud(cloud);
    n.setSearchMethod(tree);
    n.setKSearch(10);
    n.compute(*normals);
    //* normals should not contain the point normals + surface curvatures

    // Concatenate the XYZ and normal fields*
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
    //* cloud_with_normals = cloud + normals

    // Create search tree*
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud(cloud_with_normals);

    // Initialize objects
    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
    pcl::PolygonMesh triangles;

    // Set the maximum distance between connected points (maximum edge length)
    gp3.setSearchRadius(1);

    // Set typical values for the parameters
    gp3.setMu(2.5);
    gp3.setMaximumNearestNeighbors(100);
    gp3.setMaximumSurfaceAngle(M_PI / 4); // 45 degrees
    gp3.setMinimumAngle(M_PI / 18); // 10 degrees
    gp3.setMaximumAngle(2 * M_PI / 3); // 120 degrees
    gp3.setNormalConsistency(false);

    // Get result
    gp3.setInputCloud(cloud_with_normals);
    gp3.setSearchMethod(tree2);
    gp3.reconstruct(triangles);
    pcl::io::savePLYFile(plyfilePath, triangles);
    return 0;
}

int ply2pcd(string plyfilePath, string pcdfilePath) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPLYFile(plyfilePath, *cloud);
    pcl::io::savePCDFileBinary(pcdfilePath, *cloud);
    return 0;
}

int objSelply2pcd(string changefile, string pcdfilePath) {
    return 0;
}

int pcd2obj(string pcdfilePath, string objfilePath) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile(pcdfilePath, *cloud);
    pcl::PolygonMesh mesh;
    pcl::toPCLPointCloud2(*cloud, mesh.cloud);
    pcl::io::saveOBJFile(objfilePath, mesh);
    return 0;
}
struct Coordinate{
    float x, y, z;
    bool operator<(const Coordinate& rhs) {
        return x<rhs.x || (x == rhs.x&&y<rhs.y) || (x == rhs.x&&y == rhs.y&&z<rhs.z);
    }
    bool operator==(const Coordinate& rhs) {
        return x == rhs.x&&y == rhs.y && z == rhs.z;
    }
};

vector<Coordinate> vecSorted, vecOrigin;
vector<Coordinate>::iterator iter, iterBegin;

int numberOfFacets;
int numberOfPoints;
int indexnumber;

char c1[] = "ply\nformat binary_little_endian 1.0\ncomment By ET \nelement vertex ";
char c2[] = "\nproperty float x\nproperty float y\nproperty float z\nelement face ";
char c3[] = "\nproperty list uchar int vertex_indices\nend_header\n";

int stl2ply(int argc, char **argv) {
    cout << ".exe .STL .ply" << endl;
    clock_t start, finish;
    double totaltime;
    start = clock();

    int length;
    int position = 80;
    fstream fileIn(argv[1], ios::in | ios::binary);
    fileIn.seekg(0, ios::end);
    length = (int)fileIn.tellg();
    fileIn.seekg(0, ios::beg);
    char* buffer = new char[length];
    fileIn.read(buffer, length);
    fileIn.close();

    numberOfFacets = *(int*)&(buffer[position]);
    position += 4;
    cout << "Number of Facets: " << numberOfFacets << endl;
    for (int i = 0; i < numberOfFacets; i++)
    {
        Coordinate tmpC;
        position += 12;
        for (int j = 0; j < 3; j++)
        {
            tmpC.x = *(float*)&(buffer[position]);
            position += 4;
            tmpC.y = *(float*)&(buffer[position]);
            position += 4;
            tmpC.z = *(float*)&(buffer[position]);
            position += 4;
            vecOrigin.push_back(tmpC);
        }
        position += 2;
    }

    free(buffer);

    vecSorted = vecOrigin;
    sort(vecSorted.begin(), vecSorted.end());
    iter = unique(vecSorted.begin(), vecSorted.end());
    vecSorted.erase(iter, vecSorted.end());
    numberOfPoints = vecSorted.size();

    ofstream fileOut(argv[2], ios::binary | ios::out | ios::trunc);

    fileOut.write(c1, sizeof(c1) - 1);
    fileOut << numberOfPoints;
    fileOut.write(c2, sizeof(c2) - 1);
    fileOut << numberOfFacets;
    fileOut.write(c3, sizeof(c3) - 1);


    buffer = new char[numberOfPoints * 3 * 4];
    position = 0;
    for (int i = 0; i < numberOfPoints; i++)
    {
        buffer[position++] = *(char*)(&vecSorted[i].x);
        buffer[position++] = *((char*)(&vecSorted[i].x) + 1);
        buffer[position++] = *((char*)(&vecSorted[i].x) + 2);
        buffer[position++] = *((char*)(&vecSorted[i].x) + 3);
        buffer[position++] = *(char*)(&vecSorted[i].y);
        buffer[position++] = *((char*)(&vecSorted[i].y) + 1);
        buffer[position++] = *((char*)(&vecSorted[i].y) + 2);
        buffer[position++] = *((char*)(&vecSorted[i].y) + 3);
        buffer[position++] = *(char*)(&vecSorted[i].z);
        buffer[position++] = *((char*)(&vecSorted[i].z) + 1);
        buffer[position++] = *((char*)(&vecSorted[i].z) + 2);
        buffer[position++] = *((char*)(&vecSorted[i].z) + 3);
    }


    fileOut.write(buffer, numberOfPoints * 3 * 4);

    free(buffer);

    buffer = new char[numberOfFacets * 13];

    for (int i = 0; i < numberOfFacets; i++)
    {
        buffer[13 * i] = (unsigned char)3;
    }

    iterBegin = vecSorted.begin();
    position = 0;
    for (int i = 0; i < numberOfFacets; i++)
    {
        position++;
        for (int j = 0; j < 3; j++)
        {
            iter = lower_bound(vecSorted.begin(), vecSorted.end(), vecOrigin[3 * i + j]);
            indexnumber = iter - iterBegin;
            buffer[position++] = *(char*)(&index);
            buffer[position++] = *((char*)(&index) + 1);
            buffer[position++] = *((char*)(&index) + 2);
            buffer[position++] = *((char*)(&index) + 3);

        }
    }

    fileOut.write(buffer, numberOfFacets * 13);

    free(buffer);
    fileOut.close();

    finish = clock();
    totaltime = (double)(finish - start) / CLOCKS_PER_SEC * 1000;
    cout << "All Time: " << totaltime << "ms\n";

    return 0;
}

int main() {

//    pcd转txt
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ> );
//    pcl::io::loadPCDFile("test_pcd.pcd", *cloud);
//
//    ofstream outfile;
//    outfile.open("test_pcd.txt");
//
//    for( size_t i = 0; i < cloud->points.size(); ++i ){
//        outfile << cloud->points[i].x << "\t" << cloud->points[i].y << "\t" << cloud->points[i].z << "\n";
//    }
    string pcdpath = "test_pcd.pcd";
    string txtpath = "test_pcd.txt";
    string plypath = "pcd.ply";
//    pcd2txt(pcdpath, txtpath);
//    txt2pcd(txtpath, pcdpath);
//    pcd2ply(pcdpath, plypath);
//    pcd2plyAngly(pcdpath, plypath);
    cout << "finish" << endl;
    return 0;
}
