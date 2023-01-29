import numpy as np

# msake sure locations is coordinates in centimetres
def covariance(locations):
    locations = np.array(locations)
    x = locations[:, 0]
    y = locations[:, 1]
    x_mean = np.mean(x)
    y_mean = np.mean(y)
    covariance = np.sum((x - x_mean) * (y - y_mean)) / (len(x) - 1)
    return np.array([[np.var(x, ddof=1), covariance], [covariance, np.var(y, ddof=1)]])



# print("our code: ", covariance([[10,10],[20,10],[15,20],[40,30],[10,10],[10,10],[20,10],[15,20],[40,30],[30,10]]))
# print("correct code: ", np.cov([[10,10],[20,10],[15,20],[40,30],[10,10],[10,10],[20,10],[15,20],[40,30],[30,10]], rowvar=False))