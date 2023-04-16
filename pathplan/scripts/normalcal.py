import open3d
import numpy as np
# import mayavi.mlab as mlab

# 4. 法向量的计算
def open3d_vector_compute():
    pcd_path = "./src/data/curvenoisefilter.pcd"
    pcd = open3d.io.read_point_cloud(pcd_path)
    pcd.estimate_normals(
        search_param=open3d.geometry.KDTreeSearchParamKNN(knn=20)                        # 计算近邻的20个点
    )
    normals = np.array(pcd.normals)    # 法向量结果与点云维度一致(N, 3)
    points = np.array(pcd.points)
    print(normals.shape, points.shape)
    # print(np.asarray(pcd.points))
    print(np.asarray(normals))
    print(len(normals))
    print(min(sum(np.transpose((points-[2,1,0])**2))))
    # 验证法向量模长为1(模长会有一定的偏差，不完全为1)
    normals_length = np.sum(normals**2, axis=1)
    flag = np.equal(np.ones(normals_length.shape, dtype=float), normals_length).all()
    print('all equal 1:', flag)
    open3d.geometry.PointCloud.orient_normals_to_align_with_direction(pcd, orientation_reference=np.array([0.0, 0.0, 1.0]))  #设定法向量在z轴方向上，全部z轴正方向一致
    # 法向量可视化
    open3d.visualization.draw_geometries([pcd],
                                         window_name="Open3d",
                                         point_show_normal=True,
                                         width=800,   # 窗口宽度
                                         height=600)  # 窗口高度


if __name__ == '__main__':
    open3d_vector_compute()
