import get_data
from scipy.stats import multivariate_normal



def generateGaussianNoise(xyzrpy, num_uncertainties):
    mean = xyzrpy
    cov = numpy.diag(numpy.array([1]*6))
    distr = np.random.multivariate_normal(mean, cov, num_uncertainties)
    return distr
