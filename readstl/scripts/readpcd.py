#! /usr/env/python
import open3d as o3d
import copy
import numpy as np
import math

## 函数区
# 考虑x/y方向的路径
def XdirectionZigPath(xmin,xmax,ymin,ymax,Nx):
    paths = []
    dx = float(xmax-xmin)/(Nx-1)  # the y step-over
    flag=1      #奇偶分别
    path=[]
    for n in range(0,Nx):
        x = xmin+n*dx              # current y-coordinate 
        if (n==Nx-1):
            assert( x==xmax)
        elif (n==0):
            assert( x==xmin)
        if flag==1:
            p1 = [x,ymin,0]   # start-point of line
            p2 = [x,ymax,0]   # end-point of line
            # print(flag)
        if flag==-2:
            p1 = [x,ymax,0]   # start-point of line
            p2 = [x,ymin,0]   # end-point of line
            # print(flag)
        path.append(p1)       # add the line to the path
        path.append(p2)
        flag=~flag
        # paths.append(path)    
    return path

# 得到步进x/y方向的路径
def feedPath(path,step,step_num):
    flag=1      #奇偶分别
    fpath=[]
    n=len(path)
    for i in range(n//2):
        for j in range(step_num):
            if flag==1:
                p=[path[2*i][0],path[2*i][1]+j*step,path[2*i][2]]
            if flag==-2:
                p=[path[2*i][0],path[2*i][1]-j*step,path[2*i][2]]
            fpath.append(p)
        flag=~flag
    return fpath

# 依据点云坐标得到z方向的坐标
def ZdirectionPath(fpath,points,scan_height,rapid_height):
    zpath=[]
    n=len(fpath)
    # 初始点
    zpath.append([fpath[0][0],fpath[0][1],rapid_height])
    # 中间点
    for i in range(n):
        temp=[]
        for m in range(len(points)):
            temp.append(math.sqrt((fpath[i][0]-points[m][0])**2+(fpath[i][1]-points[m][1])**2))
        min_index=np.argmin(temp)
        # print(min_index)
        scan_z=points[min_index][2]+scan_height
        zpath.append([fpath[i][0],fpath[i][1],scan_z])
    # 结束点
    zpath.append([fpath[n-1][0],fpath[n-1][1],rapid_height])
    return zpath

# 主函数
if __name__ == "__main__": 
    mesh_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=[0, 0, 0])
    data = o3d.io.read_point_cloud("./src/data/point_cloud/pointcloud_1.pcd")
    # o3d.visualization.draw_geometries([data,mesh_frame])
    # 红色：x
    # 绿色：y
    # 蓝色：z

    # 使用欧拉角创建旋转矩阵
    pcd = copy.deepcopy(data).translate((0, 0, -0.24))
    # o3d.visualization.draw_geometries([pcd,mesh_frame])

    # 旋转网格
    R = mesh_frame.get_rotation_matrix_from_xyz((np.pi, 0, 0))
    pcd.rotate(R, center=(0, 0, 0))
    R = mesh_frame.get_rotation_matrix_from_xyz((0, 0, np.pi/12))
    pcd.rotate(R, center=(0, 0, 0))
    # o3d.visualization.draw_geometries([pcd,mesh_frame])

    # 使用RANSAC从点云中分割平面，用segement_plane函数。这个函数需要三个参数：
    # destance_threshold：定义了一个点到一个估计平面的最大距离，这些距离内的点被认为是内点（inlier），
    # ransac_n：定义了使用随机抽样估计一个平面的点的个数，
    # num_iterations：定义了随机平面采样和验证的频率（迭代次数）。
    # 这个函数返回（a,b,c,d）作为一个平面，对于平面上每个点(x,y,z)满足ax+by+cz+d=0。这个函数还会返回内点索引的列表。
    plane_model, inliers = pcd.segment_plane(distance_threshold=0.005,
                                            ransac_n=10,
                                            num_iterations=1000)

    [a, b, c, d] = plane_model
    print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")

    inlier_cloud = pcd.select_by_index(inliers)
    # inlier_cloud.paint_uniform_color([1.0, 0, 0])
    outlier_cloud = pcd.select_by_index(inliers, invert=True)
    # outlier_cloud.paint_uniform_color([0, 1, 0])
    # o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud])
    # o3d.visualization.draw_geometries([inlier_cloud,mesh_frame])  # 桌面
    # o3d.visualization.draw_geometries([outlier_cloud,mesh_frame])  # 工件

    # 计算法向量
    outlier_cloud.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamKNN(knn=200)                        # 计算近邻的20个点
    )
    o3d.geometry.PointCloud.orient_normals_to_align_with_direction(outlier_cloud, orientation_reference=np.array([0.0, 0.0, -1.0]))  #设定法向量在z轴方向上，全部z轴正方向一致
    normals = np.array(outlier_cloud.normals)    # 法向量结果与点云维度一致(N, 3)
    points = np.array(outlier_cloud.points)
    # print(points)

    # 法向量可视化
    o3d.visualization.draw_geometries([outlier_cloud],
                                         window_name="Open3d",
                                         point_show_normal=True,
                                         width=800,   # 窗口宽度
                                         height=600)  # 窗口高度

    # 获取工件x/y方向的最大值和最小值
    xmin=np.min(points[:,0])
    xmax=np.max(points[:,0])
    ymin=np.min(points[:,1])
    ymax=np.max(points[:,1])
    # 确定x方向分Nx条线
    Nx=10  # number of lines in the x-direction
    path = XdirectionZigPath(xmin,xmax,ymin,ymax,Nx)
    # print(path)
    
    # 设置每条线走几步
    step_num=10
    step=(ymax-ymin)/(step_num-1)
    fpath=feedPath(path,step,step_num)

    scan_height=0.01
    rapid_height=0.3
    zpath=ZdirectionPath(fpath,points,scan_height,rapid_height)
    # print(zpath)

    # 将路径画在点云上,可视化
    #绘制顶点
    lines=[]
    for i in range(len(zpath)-1):
        l=[i,i+1]
        lines.append(l) #连接的顺序
    color = [[0, 0, 0.8] for i in range(len(lines))] 
    #添加顶点，点云
    points_pcd = o3d.geometry.PointCloud()# 传入3d点云
    points_pcd.points = o3d.utility.Vector3dVector(zpath)  # point_points 二维 numpy 矩阵,将其转换为 open3d 点云格式
    points_pcd.paint_uniform_color([0.8, 0, 0]) #点云颜色 
    #绘制线条
    lines_pcd = o3d.geometry.LineSet()
    lines_pcd.lines = o3d.utility.Vector2iVector(lines)
    lines_pcd.colors = o3d.utility.Vector3dVector(color) #线条颜色
    lines_pcd.points = o3d.utility.Vector3dVector(zpath)
    # 可视化
    o3d.visualization.draw_geometries([points_pcd,lines_pcd,outlier_cloud,mesh_frame])

