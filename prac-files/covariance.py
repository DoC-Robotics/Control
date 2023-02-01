import numpy as np

# msake sure locations is coordinates in centimetres
def covariance(locations):
    locations = np.array(locations)
    x = locations[:, 0]
    y = locations[:, 1]
    x_mean = np.mean(x)
    y_mean = np.mean(y)
    # covariance = np.sum((x - x_mean) * (y - y_mean)) / (len(x) - 1)
    covariance = np.sum((x - x_mean) * (y - y_mean)) / (len(x)-1)

    return np.array([[np.var(x, ddof=1), covariance], [covariance, np.var(y, ddof=1)]])

def covariace_two(arr_param):

    """
    Get the numpy arrays
    """
    arr_param = np.array(arr_param)
    y_vals = arr_param[:, 1]
    x_vals = arr_param[:, 0]
    x_mean = np.mean(x_vals)
    y_mean = np.mean(y_vals)
    var_x = np.sum((x_vals-x_mean)**2)/len(x_vals)
    var_y = np.sum((y_vals-y_mean)**2)/len(x_vals)
    cov_formula = np.sum((x_vals- x_mean) * (y_vals - y_mean)) / (len(x_vals))
    return np.array([[var_x, cov_formula], [cov_formula, var_y]])


def test_covariance():

    tmp = covariance([[15,10],[20,10],[15,20],[40,30],[10,10],[10,10],[20,10],[15,20],[40,30],[30,13]])
    print("Omar's code Test: ", tmp)
    #it seems my code is right because we have a scale factor of 10/9 to map between the found values of covariance. 
    tmp_2 = covariace_two([[15,10],[20,10],[15,20],[40,30],[10,10],[10,10],[20,10],[15,20],[40,30],[30,13]])
    print("James' Code Test - scale factor of 9/10",tmp_2)


if __name__=='__main__':
    test_covariance()
    # print("Omar Output:",covariance(measured_array))
    measured_array = [[1.1,0],[-0.6,0.2],[-0.7,0.7],[-0.9,0.3],[-1.3,0.5],[-1.7,0.4],[-0.1,-0.7],[0.5,-0.7],[-0.6,-1.0],[0.3,-1.3]]
    print("covariance of measured array: ", covariance(measured_array))
    # print("James Output:",covariace_two(measured_array))


# 



# print("correct code: ", np.cov([[10,10],[20,10],[15,20],[40,30],[10,10],[10,10],[20,10],[15,20],[40,30],[30,10]], rowvar=False))