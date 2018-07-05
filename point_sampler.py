from pyntcloud import PyntCloud
import numpy

def getPointCloudFromMesh(filename, flag):
    cloud = PyntCloud.from_file(filename + ".ply")
    #cloud.plot()
    voxelgrid_id = cloud.add_structure("voxelgrid", n_x=32, n_y=32, n_z=32)
    points = cloud.get_sample("mesh_random", n=1000, normals=True)
    points_array = numpy.array(points.to_records())
    point_list = []
    kd_tree_list = []
    for i in range(0, len(points_array)):
        #point = [points_array[i][1], points_array[i][2], points_array[i][3]]
        if flag == 'hand':
            point_list.append(points_array[i][1])
            point_list.append(points_array[i][2])
            point_list.append(points_array[i][3])
            point_list.append(points_array[i][4])
            point_list.append(points_array[i][5])
            point_list.append(points_array[i][6])
        else:
            kd_tree_list.append(points_array[i][1])
            kd_tree_list.append(points_array[i][2])
            kd_tree_list.append(points_array[i][3])

    if flag == 'hand':
        numpy.savetxt(filename + '.out', point_list, delimiter=',')
        new_cloud = PyntCloud(points)
        new_cloud.to_file(filename + '.npz')
    else:
        numpy.savetxt(filename + '.out', kd_tree_list, delimiter=',')
