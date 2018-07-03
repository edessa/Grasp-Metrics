import vtk
import numpy

def processVTI():
	filename = '/home/eadom/Grasp-Metrics/SprayBottle.vti'
	r = vtk.vtkXMLImageDataReader()
	r.SetFileName(filename)
	r.Update()
	data = r.GetOutput()
	dim = numpy.asarray(data.GetDimensions())
	dim = [dim[0]/1000.0, dim[1]/1000.0, dim[2]/1000.0]
	spacing = data.GetSpacing()
	spacing = [spacing[0]/1000.0, spacing[1]/1000.0, spacing[2]/1000.0]
	#x = numpy.zeros(data.GetNumberOfPoints())
	#y = x.copy()
	#z = x.copy()
	#Can do data.SetOrigin(data.GetOrigin() blah blah blah)
	bounds = data.GetBounds()
	bounds = numpy.array(bounds)/1000

	extent = data.GetExtent()

	pd = data.GetPointData()
	a = pd.GetArray(0)
	xshape = numpy.array((extent[1],extent[3],extent[5])) - numpy.array((extent[0],extent[2],extent[4])) + 1
	xshape_rev = xshape[-1::-1]
	x = numpy.zeros(xshape_rev,numpy.float32)
	for zi in range(xshape[2]):
	    for yi in range(xshape[1]):
	        for xi in range(xshape[0]):
	            idx = zi*xshape[0]*xshape[1] + yi*xshape[0] + xi
		    x[zi,yi,xi] = a.GetValue(idx) / 1000.0
	return x, bounds, extent, spacing
