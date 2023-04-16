#! /usr/bin/env python
# encoding: utf-8


# simple parallel finish toolpath example
# Anders Wallin 2014-02-23

import time
import math
from opencamlib import ocl, camvtk
import ngc_writer # G-code output is produced by this module
import rospy
from std_msgs.msg import String
# import tf2_ros
from geometry_msgs.msg import PoseStamped
import open3d
import numpy as np
from pyquaternion import Quaternion

# create a simple "Zig" pattern where we cut only in one direction.
# the first line is at ymin
# the last line is at ymax
def YdirectionZigPath(xmin,xmax,ymin,ymax,Ny):
    paths = []
    dy = float(ymax-ymin)/(Ny-1)  # the y step-over
    for n in range(0,Ny):
        path = ocl.Path() 
        y = ymin+n*dy              # current y-coordinate 
        if (n==Ny-1):
            assert( y==ymax)
        elif (n==0):
            assert( y==ymin)
        p1 = ocl.Point(xmin,y,0)   # start-point of line
        p2 = ocl.Point(xmax,y,0)   # end-point of line
        l = ocl.Line(p1,p2)        # line-object
        path.append( l )           # add the line to the path
        paths.append(path)
    return paths

def XdirectionZigPath(xmin,xmax,ymin,ymax,Nx):
    paths = []
    dx = float(xmax-xmin)/(Nx-1)  # the y step-over
    for n in range(0,Nx):
        path = ocl.Path() 
        x = xmin+n*dx              # current y-coordinate 
        if (n==Nx-1):
            assert( x==xmax)
        elif (n==0):
            assert( x==xmin)
        p1 = ocl.Point(x,ymin,0)   # start-point of line
        p2 = ocl.Point(x,ymax,0)   # end-point of line
        l = ocl.Line(p1,p2)        # line-object
        path.append( l )           # add the line to the path
        paths.append(path)
    return paths

# run the actual drop-cutter algorithm
def adaptive_path_drop_cutter(surface, cutter, paths):
    apdc = ocl.AdaptivePathDropCutter()
    apdc.setSTL(surface)
    apdc.setCutter(cutter)
    apdc.setSampling(0.04)      # maximum sampling or "step-forward" distance
                                # should be set so that we don't loose any detail of the STL model
                                # i.e. this number should be similar or smaller than the smallest triangle
    apdc.setMinSampling(0.01) # minimum sampling or step-forward distance
                                # the algorithm subdivides "steep" portions of the toolpath
                                # until we reach this limit.
    # 0.0008
    cl_paths=[]
    n_points=0
    for path in paths:
        apdc.setPath( path )
        apdc.run()
        cl_points = apdc.getCLPoints()
        n_points = n_points + len( cl_points )
        cl_paths.append( apdc.getCLPoints() )
    return (cl_paths, n_points)

# this could be any source of triangles
# as long as it produces an ocl.STLSurf() we can work with
def STLSurfaceSource(filename):
    stl = camvtk.STLSurf(filename)
    polydata = stl.src.GetOutput()
    s = ocl.STLSurf()
    camvtk.vtkPolyData2OCLSTL(polydata, s)
    return s

# filter a single path
def filter_path(path,tol):
    f = ocl.LineCLFilter()
    f.setTolerance(tol)
    for p in path:
        p2 = ocl.CLPoint(p.x,p.y,p.z)
        f.addCLPoint(p2)
    f.run()
    return  f.getCLPoints()

# to reduce the G-code size we filter here. (this is not strictly required and could be omitted)
# we could potentially detect G2/G3 arcs here, if there was a filter for that.
# idea:
# if a path in the raw toolpath has three points (p1,p2,p3) 
# and point p2 lies within tolerance of the straight line p1-p3
# then we simplify the path to (p1,p3) 
def filterCLPaths(cl_paths, tolerance=0.001):
    cl_filtered_paths = []
    t_before = time.time()
    n_filtered=0
    for cl_path in cl_paths:
        cl_filtered = filter_path(cl_path,tol)
        
        n_filtered = n_filtered + len(cl_filtered)
        cl_filtered_paths.append(cl_filtered)
    return (cl_filtered_paths, n_filtered)

# this uses ngc_writer and writes G-code to stdout or a file
def write_zig_gcode_file(filename, n_triangles, t1,n1,tol,t2,n2, toolpath):
    ngc_writer.clearance_height= 5 # XY rapids at this height
    ngc_writer.feed_height = 3     # use z plunge-feed below this height
    ngc_writer.feed = 200          # feedrate 
    ngc_writer.plunge_feed = 100   # plunge feedrate
    ngc_writer.metric = False      # metric/inch flag
    ngc_writer.comment( " OpenCAMLib %s" % ocl.version() ) # git version-tag
    # it is probably useful to include this in all g-code output, so that bugs/problems can be tracked
    
    ngc_writer.comment( " STL surface: %s" % filename )
    ngc_writer.comment( "   triangles: %d" % n_triangles )
    ngc_writer.comment( " OpenCAMLib::AdaptivePathDropCutter run took %.2f s" % t1 )
    ngc_writer.comment( " got %d raw CL-points " % n1 )
    ngc_writer.comment( " filtering to tolerance %.4f " % ( tol )  )
    ngc_writer.comment( " got %d filtered CL-points. Filter done in %.3f s " % ( n2 , t2 ) )
    ngc_writer.preamble()
    # a "Zig" or one-way parallel finish path
    # 1) lift to clearance height
    # 2) XY rapid to start of path
    # 3) plunge to correct z-depth
    # 4) feed along path until end 
    for path in toolpath:
        ngc_writer.pen_up()  
        first_pt = path[0]
        ngc_writer.xy_rapid_to( first_pt.x, first_pt.y )
        ngc_writer.pen_down( first_pt.z )
        for p in path[1:]:
            ngc_writer.line_to(p.x,p.y,p.z)
    ngc_writer.postamble() # end of program

def vtk_visualize_parallel_finish_zig(stlfile, toolpaths):
	myscreen = camvtk.VTKScreen()
	stl = camvtk.STLSurf(stlfile)
	myscreen.addActor(stl)
	stl.SetSurface() # try also SetWireframe()
	stl.SetColor(camvtk.cyan)
	myscreen.camera.SetPosition(0.1, 0.1, 0.05)
	myscreen.camera.SetFocalPoint(0.5, 5, 0)
	
	rapid_height= 0.3 # XY rapids at this height
	feed_height = 0.2 
	rapidColor = camvtk.pink
	XYrapidColor = camvtk.green
	plungeColor = camvtk.red
	feedColor = camvtk.yellow
	# zig path algorithm:
	# 1) lift to clearance height
	# 2) XY rapid to start of path
	# 3) plunge to correct z-depth
	# 4) feed along path until end
	pos = ocl.Point(0,0,0) # keep track of the current position of the tool
	first = True 
	for path in toolpaths:
		first_pt = path[0]
		if (first == True): # green sphere at path start
			myscreen.addActor( camvtk.Sphere(center=(first_pt.x,first_pt.y,rapid_height) , radius=0.01, color=camvtk.green) ) 
			pos = ocl.Point(first_pt.x,first_pt.y,first_pt.z) # at start of program, assume we have already a rapid move here
			first = False
		else: # not the very first move
			# retract up to rapid_height
			myscreen.addActor( camvtk.Line(p1=( pos.x,pos.y,pos.z),p2=(pos.x,pos.y,feed_height),color=plungeColor) )
			myscreen.addActor( camvtk.Line(p1=( pos.x,pos.y,feed_height),p2=(pos.x,pos.y,rapid_height),color=rapidColor) )
			# XY rapid into position
			myscreen.addActor( camvtk.Line(p1=( pos.x,pos.y,rapid_height),p2=( first_pt.x,first_pt.y,rapid_height),color=XYrapidColor) )
			pos = ocl.Point(first_pt.x,first_pt.y,first_pt.z)
		
		# rapid down to the feed_height
		myscreen.addActor( camvtk.Line(p1=( pos.x,pos.y,rapid_height),p2=(pos.x,pos.y,feed_height),color=rapidColor) )
		# feed down to CL
		myscreen.addActor( camvtk.Line(p1=( pos.x,pos.y,feed_height),p2=(pos.x,pos.y,pos.z),color=plungeColor) )
		
		# feed along the path
		for p in path[1:]:
			myscreen.addActor( camvtk.Line(p1=( pos.x,pos.y,pos.z),p2=(p.x,p.y,p.z),color=feedColor) )
			pos = ocl.Point(p.x,p.y,p.z)
		
	# END retract up to rapid_height
	myscreen.addActor( camvtk.Line(p1=( pos.x,pos.y,pos.z),p2=(pos.x,pos.y,feed_height),color=plungeColor) )
	myscreen.addActor( camvtk.Line(p1=( pos.x,pos.y,feed_height),p2=(pos.x,pos.y,rapid_height),color=rapidColor) )
	myscreen.addActor( camvtk.Sphere(center=(pos.x,pos.y,rapid_height) , radius=0.01, color=camvtk.red) ) 

	camvtk.drawArrows(myscreen,center=(-0.5,-0.5,-0.5)) # XYZ coordinate arrows
	camvtk.drawOCLtext(myscreen)
	myscreen.render()    
	myscreen.iren.Start()


if __name__ == "__main__":     
    stlfile = "./src/data/curvenoisefilter.stl"
    # stlfile = "./src/data/gnu_tux_mod.stl"
    surface = STLSurfaceSource(stlfile)
    rospy.loginfo(surface)
    # choose a cutter for the operation:
    # http://www.anderswallin.net/2011/08/opencamlib-cutter-shapes/
    diameter=0.25
    length=0.5
    # cutter = ocl.BallCutter(diameter, length)
    # cutter = ocl.CylCutter(diameter, length)

    corner_radius = 0.05
    cutter = ocl.BullCutter(diameter, corner_radius, length)
    angle = math.pi/4
    cutter = ocl.ConeCutter(diameter, angle, length)
    cutter = cutter.offsetCutter( 0.4 )
 
    # toolpath is contained in this simple box
    ymin=0
    ymax=0.3
    xmin=0
    xmax=0.4
    # Ny=10  # number of lines in the y-direction
    # paths = YdirectionZigPath(xmin,xmax,ymin,ymax,Ny)

    Nx=10  # number of lines in the x-direction
    paths = XdirectionZigPath(xmin,xmax,ymin,ymax,Nx)
    

    # now project onto the STL surface
    t_before = time.time()
    (raw_toolpath, n_raw) = adaptive_path_drop_cutter(surface,cutter,paths)
    t1 = time.time()
    
    # filter raw toolpath to reduce size
    tol = 0.001    
    (toolpaths, n_filtered) = filterCLPaths(raw_toolpath, tolerance=0.001)    
    t2 = time.time()-t_before

    # 输出路径	
    rapid_height= 0.3 # XY rapids at this height
    feed_height = 0.2 
    pos = ocl.Point(0,0,0) # keep track of the current position of the tool
    first = True 
    temp=[]
    for path in toolpaths:
        first_pt = path[0]
        if (first == True): # green sphere at path start
            temp.append( [first_pt.x,first_pt.y,rapid_height] ) 
            pos = ocl.Point(first_pt.x,first_pt.y,first_pt.z) # at start of program, assume we have already a rapid move here
            first = False
        else: # not the very first move
            # retract up to rapid_height
            temp.append( [pos.x,pos.y,pos.z] )
            temp.append( [pos.x,pos.y,feed_height] )
            temp.append( [pos.x,pos.y,rapid_height] )
            # XY rapid into position
            temp.append( [first_pt.x,first_pt.y,rapid_height] )
            pos = ocl.Point(first_pt.x,first_pt.y,first_pt.z)

        # rapid down to the feed_height
        temp.append( [pos.x,pos.y,rapid_height] )
        # feed down to CL
        temp.append( [pos.x,pos.y,feed_height] )
        temp.append( [pos.x,pos.y,pos.z] )

        # feed along the path
        for p in path[1:]:
            temp.append( [pos.x,pos.y,pos.z] )
            temp.append( [p.x,p.y,p.z] )
            pos = ocl.Point(p.x,p.y,p.z)

    # print(temp)
    
    # output a g-code file
    # write_zig_gcode_file( stlfile, surface.size() , t1, n_raw ,tol,t2,n_filtered, toolpaths )

    # and/or visualize with VTK
    vtk_visualize_parallel_finish_zig(stlfile, toolpaths)

    # 发布话题
    rospy.init_node("pathplan")
    pub = rospy.Publisher("chatter",PoseStamped,queue_size=10)
    msg=String()
    count=0
    rate=rospy.Rate(1)

    msg.data=str(temp)
    # pub.publish(msg)
    # rospy.loginfo("发出的数据为：%s",msg.data)

    
    # while not rospy.is_shutdown():
    #      msg.data=msg_front+str(temp)
    #      pub.publish(msg)
    #      rospy.loginfo("%s",msg.data)
    #      count+=1
    
    ps = PoseStamped()
    # rospy.sleep(3)
    
    # 计算法向量
    pcd_path = "./src/data/curvenoisefilter.pcd"
    pcd = open3d.io.read_point_cloud(pcd_path)
    pcd.estimate_normals(
        search_param=open3d.geometry.KDTreeSearchParamKNN(knn=20)                        # 计算近邻的20个点
    )
    normals = np.array(pcd.normals)    # 法向量结果与点云维度一致(N, 3)
    points = np.array(pcd.points)
    # 验证法向量模长为1(模长会有一定的偏差，不完全为1)
    normals_length = np.sum(normals**2, axis=1)
    flag = np.equal(np.ones(normals_length.shape, dtype=float), normals_length).all()
    print('all equal 1:', flag)
    open3d.geometry.PointCloud.orient_normals_to_align_with_direction(pcd, orientation_reference=np.array([0.0, 0.0, -1.0]))  #设定法向量在z轴方向上，全部z轴正方向一致

    # 法向量可视化
    open3d.visualization.draw_geometries([pcd],
                                         window_name="Open3d",
                                         point_show_normal=True,
                                         width=800,   # 窗口宽度
                                         height=600)  # 窗口高度

    while not rospy.is_shutdown():
        ps.pose.position.x=temp[count][0]
        ps.pose.position.y=temp[count][1]
        ps.pose.position.z=temp[count][2]
        for i in range(len(normals)):
            if np.sum((points[i]-temp[count])**2)==min(sum(np.transpose((points-temp[count])**2))):
                rotate_matrix=[[0,0,0],[0,0,0],[0,0,0]]
                # 构建点
                Z = np.array([normals[i][0], normals[i][1], normals[i][2]])
                X = np.array([Z[1], -Z[0], 0])
                Z = Z/np.linalg.norm(Z)
                X = X/np.linalg.norm(X)
                rotate_matrix[0][2]=Z[0]    #normals[i][0]/math.sqrt(sum(normals[i]**2))
                rotate_matrix[1][2]=Z[1]    #normals[i][1]/math.sqrt(sum(normals[i]**2))
                rotate_matrix[2][2]=Z[2]    #normals[i][2]/math.sqrt(sum(normals[i]**2))
                rotate_matrix[0][0]=X[0]
                rotate_matrix[1][0]=X[1]
                rotate_matrix[2][0]=X[2]
                # 计算叉乘
                Y = np.cross(Z,X)
                Y = Y/np.linalg.norm(Y)
                rotate_matrix[0][1]=Y[0]
                rotate_matrix[1][1]=Y[1]
                rotate_matrix[2][1]=Y[2]
                rotateMatrix = np.array(rotate_matrix)
                print(rotateMatrix)
                q = Quaternion(matrix=rotateMatrix)
                ps.pose.orientation.x=q.x
                ps.pose.orientation.y=q.y
                ps.pose.orientation.z=q.z
                ps.pose.orientation.w=q.w
        # 发布话题
        pub.publish(ps)
        rospy.loginfo("%s",str(ps))
        rospy.loginfo("len%d",len(temp))
        
        count+=1
        if count>=len(temp):
            count=0
        