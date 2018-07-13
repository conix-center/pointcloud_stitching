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
    np.set_printoptions(precision=8, floatmode='fixed')
    temp_mat = [[0, 0, 0, 1]]

    levo_translation = np.atleast_2d(getCoordinates(pointdata, "LEVO"))
    levo_rotation = getCameraTransform(pointdata, "L")
    levo_transform = np.concatenate((levo_rotation, levo_translation.T), axis=1)
    levo_transform = np.concatenate((levo_transform, temp_mat), axis=0)
    print("levo camera transform matrix:")
    print(levo_transform)
    print()

    dextro_translation = np.atleast_2d(getCoordinates(pointdata, "DEXTRO"))
    dextro_rotation = getCameraTransform(pointdata, "D")
    dextro_transform = np.concatenate((dextro_rotation, dextro_translation.T), axis=1)
    dextro_transform = np.concatenate((dextro_transform, temp_mat), axis=0)
    print("dextro camera transform matrix:")
    print(dextro_transform)
    print()

    # returns translation vector pointing from dextro camera to levo camera
    # translation_vector = levo_translation - dextro_translation
    # print("dextro to levo translation:")
    # print(translation_vector)
    # print()
    