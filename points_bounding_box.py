import numpy

def draw_BoundingBox(orig, extents, faces, N):
	arr1 = []
	arr2 = []
	arr3 = []
	arr4 = []
	if faces == 'y':	# face y
		var = [extents[1], -extents[1]]
		for y in var:
			# Along Z
			a1 = [0,y,extents[2]]
			f1_1 = orig + a1
			a1[2] = -a1[2]
			f1_2 = orig + a1
			diff = abs(f1_1[2] - f1_2[2])
			step_size = diff / N
			b = []
			each_step = f1_2[2]
			for i in range(N):
				each_step = each_step + step_size
				temp = []
				temp = [f1_1[0], f1_1[1], each_step]
				b.append(temp)

			# Along X
			a2 = [extents[0], y, 0]
			f1_3 = orig + a2
			a2[0] = -a2[0]
			f1_4 = orig + a2
			diff1 = abs(f1_3[0] - f1_4[0])
			step_size1 = diff1 / N
			c = []
			each_step1 = f1_4[0]
			for j in range(N):
				each_step1 = each_step1 + step_size1
				temp = []
				temp = [each_step1, f1_3[1], f1_3[2]]
				c.append(temp)
			if y == extents[1]:
				arr1 = b
				arr2 = c
			else:
				arr3 = b
				arr4 = c

	elif faces == 'x':	#face x
		var = [extents[0], -extents[0]]
		for x in var:
			# Along Y
			a1 = [x,extents[1],0]
			f1_1 = orig + a1
			a1[1] = -a1[1]
			f1_2 = orig + a1
			diff = abs(f1_1[1] - f1_2[1])
			step_size = diff / N
			b = []
			each_step = f1_2[1]
			for i in range(N):
				each_step = each_step + step_size
				temp = []
				temp = [f1_1[0], each_step, f1_1[2]]
				b.append(temp)

			# Along Z
			a2 = [x, 0, extents[2]]
			f1_3 = orig + a2
			a2[2] = -a2[2]
			f1_4 = orig + a2
			diff1 = abs(f1_3[2] - f1_4[2])
			step_size1 = diff1 / N
			c = []
			each_step1 = f1_4[2]
			for j in range(N):
				each_step1 = each_step1 + step_size1
				temp = []
				temp = [f1_3[0], f1_3[1], each_step1]
				c.append(temp)
			if x == extents[0]:
				arr1 = b
				arr2 = c
			else:
				arr3 = b
				arr4 = c

	elif faces == 'z':	# face z
		var = [extents[2], -extents[2]]
		for z in var:
			# Along Y
			a1 = [0,extents[1],z]
			f1_1 = orig + a1
			a1[1] = -a1[1]
			f1_2 = orig + a1
			diff = abs(f1_1[1] - f1_2[1])
			step_size = diff / N
			b = []
			each_step = f1_2[1]
			for i in range(N):
				each_step = each_step + step_size
				temp = []
				temp = [f1_1[0], each_step, f1_1[2]]
				b.append(temp)

			# Along X
			a2 = [extents[0], 0, z]
			f1_3 = orig + a2
			a2[0] = -a2[0]
			f1_4 = orig + a2
			diff1 = abs(f1_3[0] - f1_4[0])
			step_size1 = diff1 / N
			c = []
			each_step1 = f1_4[0]
			for j in range(N):
				each_step1 = each_step1 + step_size1
				temp = []
				temp = [each_step1, f1_3[1], f1_3[2]]
				c.append(temp)
			if z == extents[2]:
				arr1 = b
				arr2 = c
			else:
				arr3 = b
				arr4 = c

	return arr1, arr2, arr3, arr4
