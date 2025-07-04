#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <iostream>

int main(int argc, char** argv)
{
  // ---- parameters ----------------------------------------------------------
  const std::string file_a   = "DBB_l.pcd";
  const std::string file_b   = "DBB_r.pcd";
  const std::string file_out = "DBB.pcd";
  const double      epsilon  = 0.05;        // [m] = 1 cm search radius
  const float       eps_sq   = static_cast<float>(epsilon * epsilon);

  // ---- load both clouds ----------------------------------------------------
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr A (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr B (new pcl::PointCloud<pcl::PointXYZRGB>);

  if (pcl::io::loadPCDFile(file_a, *A) == -1 ||
      pcl::io::loadPCDFile(file_b, *B) == -1)
  {
    PCL_ERROR ("Couldn't read one of the input PCD files\n");
    return -1;
  }
  std::cout << "Loaded A: " << A->size() << " points,  B: " << B->size() << " points\n";

  // ---- build a KD-tree on cloud A ------------------------------------------
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kdtree (new pcl::search::KdTree<pcl::PointXYZRGB>);
  kdtree->setInputCloud(A);

  // ---- iterate over cloud B -------------------------------------------------
  for (auto &pt : *B)
  {
    std::vector<int>   indices(1);
    std::vector<float> dists  (1);

    // nearest-neighbor query (1 nn is enough)
    if (kdtree->nearestKSearch(pt, 1, indices, dists) > 0 && dists[0] <= eps_sq)
    {
      // intersection → paint yellow
      pt.r = 0;  pt.g = 255;  pt.b = 0;
    }
  }

  // ---- concatenate A + modified B ------------------------------------------
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr merged (new pcl::PointCloud<pcl::PointXYZRGB>);
  *merged = *A;
  *merged += *B;            // appends B

  // ---- save ----------------------------------------------------------------
  pcl::io::savePCDFileBinary(file_out, *merged);
  std::cout << "Merged cloud saved to " << file_out
            << "   (" << merged->size() << " points)\n";

  return 0;
}
