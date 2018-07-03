import openravepy
import numpy

def bounding_box(item):
	obj = item.ComputeAABB()
	max_xyz =  obj.pos()+obj.extents()
	min_xyz = obj.pos()-obj.extents()
	return min_xyz, max_xyz


def get_positions_wrt_obj(distance, skew, bounding_box, step):
    faces = [[]] * 12
    min_xyz = bounding_box[0]
    max_xyz = bounding_box[1]
    for face in range(0, 3): #varying along position in x, y or z
        print bounding_box , '  1  '
        for start in range(0, 2): #two sides for each face varying in x, y or z
            point = bounding_box[start]
            print bounding_box , '  2  '
            point[(face+1)%3] = min_xyz[:][(face+1)%3]
            point[(face+2)%3] = min_xyz[:][(face+2)%3]
            print bounding_box , '  3  '
            a = float(bounding_box[:][1][face] - bounding_box[:][0][face])
            for i in range(0, step):
                print bounding_box , '  4  '
                point[face] += i * a/step
                faces[face*2 + start] += [point]
    return faces
