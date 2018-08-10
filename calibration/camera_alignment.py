#!/usr/bin/env python3

import sys
import numpy as np
import pandas as pd

# returns a numpy array [X, Y, Z] with matching Label names
def getCoordinates(points_dataframe, name):
    
    # get the row with matching label name 
    row = points_dataframe.loc[points_dataframe["Label"] == name]
    
    if row.empty:
        raise KeyError("Label {} not found".format(name))
    else:
        # using np.squeeze to return a 1D array
        # otherwise it returns a 3x1 matrix which is annoying to work with
        return np.squeeze(np.array([row['X'], row['Y'], row['Z']]))



def getCameraTransform(points_dataframe, camera_prefix):
    marker1 = getCoordinates(pointdata, "{}1".format(camera_prefix))
    marker2 = getCoordinates(pointdata, "{}2".format(camera_prefix))
    marker3 = getCoordinates(pointdata, "{}3".format(camera_prefix))
    marker4 = getCoordinates(pointdata, "{}4".format(camera_prefix))
    
    #     The markers are arranged like this:
    #     1-----2
    #     ---C---
    #     4-----3

    #     We define axes as follows for each camera:
    #     X <----|
    #            |    Z = cross(X, Y)
    #            v Y
    
    # to get the X vector, average both top and bottom vectors aligned with the X axis (vector21 and vector34)
    x_vector = (marker1 - marker2) + (marker4 - marker3)
    
    # to get the Y vector, average both left and right vectors aligned with the Y axis (vector14 and vector23)
    y_intermediate = (marker4 - marker1) + (marker3 - marker2)

    # cross product guarantees X_vector and z_vector are perpendicular
    z_vector = np.cross(x_vector, y_intermediate)
    
    # use them to define a y vector which can be guaranteed to be perpendicular to both X and Y
    y_vector = np.cross(z_vector, x_vector)
    
    # return normalized axis vectors
    return np.array([x_vector/np.linalg.norm(x_vector), y_vector/np.linalg.norm(y_vector), z_vector/np.linalg.norm(z_vector)]).T

if __name__ == "__main__":
    filename = sys.argv[1]

    pointdata = pd.read_csv(filename,
        names=["pointNo", "X", "Y", "Z", "Label"], index_col=0)

    print("read {}".format(filename))
    print()

    # always display exactly 8 decimal points for floats
    np.set_printoptions(precision=8, suppress='true', floatmode='fixed')
    temp_mat = [[0, 0, 0, 1]]

    a_translation = np.atleast_2d(getCoordinates(pointdata, "A"))
    a_rotation = getCameraTransform(pointdata, "A")
    a_transform = np.concatenate((a_rotation, a_translation.T), axis=1)
    a_transform = np.concatenate((a_transform, temp_mat), axis=0)
    print('transform[0] << {}'.format(a_transform))
    print()

    b_translation = np.atleast_2d(getCoordinates(pointdata, "B"))
    b_rotation = getCameraTransform(pointdata, "B")
    b_transform = np.concatenate((b_rotation, b_translation.T), axis=1)
    b_transform = np.concatenate((b_transform, temp_mat), axis=0)
    print('transform[1] << {}'.format(b_transform))
    print()

    c_translation = np.atleast_2d(getCoordinates(pointdata, "C"))
    c_rotation = getCameraTransform(pointdata, "C")
    c_transform = np.concatenate((c_rotation, c_translation.T), axis=1)
    c_transform = np.concatenate((c_transform, temp_mat), axis=0)
    print('transform[2] << {}'.format(c_transform))
    print()

    d_translation = np.atleast_2d(getCoordinates(pointdata, "D"))
    d_rotation = getCameraTransform(pointdata, "D")
    d_transform = np.concatenate((d_rotation, d_translation.T), axis=1)
    d_transform = np.concatenate((d_transform, temp_mat), axis=0)
    print('transform[3] << {}'.format(d_transform))
    print()

    e_translation = np.atleast_2d(getCoordinates(pointdata, "E"))
    e_rotation = getCameraTransform(pointdata, "E")
    e_transform = np.concatenate((e_rotation, e_translation.T), axis=1)
    e_transform = np.concatenate((e_transform, temp_mat), axis=0)
    print('transform[4] << {}'.format(e_transform))
    print()

    f_translation = np.atleast_2d(getCoordinates(pointdata, "F"))
    f_rotation = getCameraTransform(pointdata, "F")
    f_transform = np.concatenate((f_rotation, f_translation.T), axis=1)
    f_transform = np.concatenate((f_transform, temp_mat), axis=0)
    print('transform[5] << {}'.format(f_transform))
    print()

    g_translation = np.atleast_2d(getCoordinates(pointdata, "G"))
    g_rotation = getCameraTransform(pointdata, "G")
    g_transform = np.concatenate((g_rotation, g_translation.T), axis=1)
    g_transform = np.concatenate((g_transform, temp_mat), axis=0)
    print('transform[6] << {}'.format(g_transform))
    print()

    h_translation = np.atleast_2d(getCoordinates(pointdata, "H"))
    h_rotation = getCameraTransform(pointdata, "H")
    h_transform = np.concatenate((h_rotation, h_translation.T), axis=1)
    h_transform = np.concatenate((h_transform, temp_mat), axis=0)
    print('transform[7] << {}'.format(h_transform))
    print()