 
import open3d as o3d
import numpy as np
 
def polygon():
    #绘制顶点
    polygon_points = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1],[0,0,5]])
    lines = [[0, 1], [1, 2], [2, 3],[3,0]] #连接的顺序，封闭链接
    color = [[1, 0, 0] for i in range(len(lines))] 
    #添加顶点，点云
    points_pcd = o3d.geometry.PointCloud()
    points_pcd.points = o3d.utility.Vector3dVector(polygon_points)
    points_pcd.paint_uniform_color([0, 0.3, 0]) #点云颜色
 
    #绘制线条
    lines_pcd = o3d.geometry.LineSet()
    lines_pcd.lines = o3d.utility.Vector2iVector(lines)
    lines_pcd.colors = o3d.utility.Vector3dVector(color) #线条颜色
    lines_pcd.points = o3d.utility.Vector3dVector(polygon_points)
 
    return lines_pcd, points_pcd
 
