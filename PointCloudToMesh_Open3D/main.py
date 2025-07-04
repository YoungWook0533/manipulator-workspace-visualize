import numpy as np
import open3d as o3d


dataname = "DBB_l.pcd"
pcd      = o3d.io.read_point_cloud(dataname)          # XYZ[RGB] cloud

pcd.estimate_normals(
    search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1,
                                                      max_nn=50))
pcd.orient_normals_consistent_tangent_plane(k=50) 

########################################################################
# 2)  choose a *multi-scale* radius list  (key for fewer holes)
########################################################################
dists  = pcd.compute_nearest_neighbor_distance()
r_avg  = np.mean(dists)

# use 4–6 radii: two smaller than mean, two larger
radii  = [0.1 * r_avg,
          0.4 * r_avg,
          0.8 * r_avg,
          1.2 * r_avg,
          2.0 * r_avg,
          3.0 * r_avg,
          5.0 * r_avg]

bpa_mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
              pcd, o3d.utility.DoubleVector(radii))

########################################################################
# 3)  (optional) fill tiny leftover holes (Open3D ≥ 0.18)
########################################################################
try:
    bpa_mesh = bpa_mesh.fill_holes(max_hole_size=100)   # triangles per hole
except AttributeError:
    pass  # older Open3D – silently skip

########################################################################
# 4)  simplify & clean
########################################################################
dec_mesh = bpa_mesh.simplify_quadric_decimation(target_number_of_triangles=100_000)

dec_mesh.remove_degenerate_triangles()
dec_mesh.remove_duplicated_triangles()
dec_mesh.remove_duplicated_vertices()
dec_mesh.remove_non_manifold_edges()

########################################################################
# 5)  export & view
########################################################################
o3d.io.write_triangle_mesh("bpa_mesh.obj", dec_mesh)
o3d.visualization.draw_geometries([dec_mesh])
