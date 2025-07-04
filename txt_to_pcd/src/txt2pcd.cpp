#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>

using namespace std;

struct tagPOINT_3D {
    double x;  // mm world coordinate x
    double y;  // mm world coordinate y
    double z;  // mm world coordinate z
    int r;     // Red color value
    int g;     // Green color value
    int b;     // Blue color value
};

int main() {
    // Load data from txt file
    vector<tagPOINT_3D> m_vTxtPoints;
    ifstream file("DBB.txt");

    if (!file.is_open()) {
        cerr << "Failed to load txt data!" << endl;
        return -1;
    }

    tagPOINT_3D TxtPoint;
    while (file >> TxtPoint.x >> TxtPoint.y >> TxtPoint.z >> TxtPoint.r) {
        // If the file contains RGB values, you can read them here
        // For example:
        // file >> TxtPoint.r >> TxtPoint.g >> TxtPoint.b;
        // But here, we're just assigning default green color for simplicity
        // TxtPoint.r = 0;
        // TxtPoint.g = 255;
        // TxtPoint.b = 0;

        m_vTxtPoints.push_back(TxtPoint);
    }
    file.close();

    int number_Txt = m_vTxtPoints.size();

    // Initialize the point cloud object
    pcl::PointCloud<pcl::PointXYZRGB> cloud;

    // Fill in the cloud data
    cloud.width = number_Txt;
    cloud.height = 1;
    cloud.is_dense = false;
    cloud.points.resize(cloud.width * cloud.height);

    // Assign data to the point cloud
    for (size_t i = 0; i < cloud.points.size(); ++i) {
        cloud.points[i].x = m_vTxtPoints[i].x;
        cloud.points[i].y = m_vTxtPoints[i].y;
        cloud.points[i].z = m_vTxtPoints[i].z;
        cloud.points[i].r = m_vTxtPoints[i].r;
        cloud.points[i].g = 0;
        cloud.points[i].b = 255;
    }

    // Save the point cloud to a PCD file
    pcl::io::savePCDFileASCII("DBB.pcd", cloud);
    cout << "Saved " << cloud.points.size() << " data points to DBB.pcd." << endl;

    // Optional: print the points to verify
    for (size_t i = 0; i < cloud.points.size(); ++i) {
        cout << cloud.points[i].x << " " << cloud.points[i].y << " "
             << cloud.points[i].z << " " << cloud.points[i].r << endl;
    }

    return 0;
}
