import numpy as np
import time as time
import os
from os import walk
import shutil as shutil
from sklearn.neighbors import KDTree
import File as File
import FacePoint as Fp
import Layering123 as Lyr
import MinMax as Mm
import glob

# import trajectory as Detraj

# PARAMETER NEED TO BE SET


# We set the distance between points in Grid (For Surface Fitting)
GridPoint_dist = 1


## ----- MAIN PROGRAM -----

# 0. PRE START THE PROGRAM
# List the files inside Source File Folder
print('\nSOURCE FILE LIST:\n')

# for part_no in range(0, len(ref_file)):

Source_list = []
SrcFilepath = 'Source File/'
for (dirpath, dirnames, filenames) in walk(SrcFilepath):
    Source_list.extend(filenames)
    break

# Show all files name
for file_no in range(0, len(Source_list)):
    print('{}.'.format(file_no), Source_list[file_no])

# Waiting for user choose which file for Reference and Raw
file_no = int(input('\nInput Ref file no: '))
Ref_Name = Source_list[file_no]
file_no = int(input('Input Raw file no: '))
Raw_Name = Source_list[file_no]

# Waiting for user to set the 1st Layer thickness in mm
Layer1_height = float(input('Input First Layer Height (mm) : '))
# Waiting for user to set the other Layer thickness in mm
Layer_height = float(input('Input Other Layer Height (mm) : '))

# Set the Path of the source file
Ref_File = 'Source File/' + Ref_Name
Raw_File = 'Source File/' + Raw_Name

# Create Output File Folder
OutputFolder = 'Output File'
if os.path.exists(OutputFolder):
    print('Note: If it is error, Make sure to close the "Layer" folder first and run it again.')
    shutil.rmtree(OutputFolder)
os.makedirs(OutputFolder)





# --- START the PROGRAM ---
print('\nPROGRAM START')
Prog_start_time = time.time()

#1. Read Reference File

start_time = time.time()
print('\nStep 1 READING REFERENCE FILE')

# Check File Format, make sure the file should be typed .STL (Big font)
RefFileType = File.CheckFileFormat(Ref_File)
print('RefFileType:', RefFileType)
RawFileType = File.CheckFileFormat(Raw_File)
print('RawFileType:', RawFileType)

# Read the "golden model" file based on type of file
if RefFileType == '.STL':
    Vertice1, Normals1, VertNorm1 = File.ReadSTLFile(Ref_File)              # STL

elif RefFileType == '.xyz':
    NewPoint, NewPointIdx, NewNormal = File.ReadXYZNormalFile(Ref_File)     # XYZ and Normal


end_time = time.time()
print('[Duration:', int((end_time-start_time)/60), 'minutes', int((end_time-start_time) % 60), 'seconds]')

#2. Create new point from reference file (Increase the number of points if the file is STL )
start_time = time.time()
print('\nStep 2 CREATING NEW POINT (for STL file)')

# if it is not XYZ file
if RefFileType != '.xyz':
    NewPoint, NewPointIdx, NewNormal = Fp.NewPoint(Vertice1, Normals1)

    # Filter the unnecessary points (For example, we just want to focus on propeller blade)
    print('Filtering the unnecessary point clouds')
    NewPoint, NewPointIdx, NewNormal = Mm.PointFilter(NewPoint, NewPointIdx, NewNormal, x1=-21, x2=21, z1=-5.5, z2=17.33)  # 72, 115
    File.SaveFile('Output File/NewPoint', NewPoint)

else:
    print('It is XYZ File, so skip.')
    # Filter the unnecessary points
    print("Filtering the unnecessary point clouds, we don't need bottom and top part")

end_time = time.time()
print('[Duration:', int((end_time-start_time)/60), 'minutes', int((end_time-start_time) % 60), 'seconds]')

#3. Read Raw File ("raw model" file)
start_time = time.time()
print('\nStep 3 READING RAW FILE ("before" file)')

if RawFileType == '.xyz':
    Vertice2 = File.ReadXyzFile(Raw_File)   # XYZ

elif RawFileType == '.pcd':
    Vertice2 = File.ReadPcdFile(Raw_File)

end_time = time.time()
print('[Duration:', int((end_time-start_time)/60), 'minutes', int((end_time-start_time) % 60), 'seconds]')


#4. KDTree for Raw File
# KDTree is algorithm by splitting the point based on location, it will increase the speed of finding the point in the future.
start_time = time.time()
print('\nStep 4 KdTree THE POINT')

Vertice2 = np.asarray(Vertice2)
tree = KDTree(Vertice2, leaf_size=8)

end_time = time.time()
print('[Duration:', int((end_time-start_time)/60), 'minutes', int((end_time-start_time) % 60), 'seconds]')


#5. Layering
start_time = time.time()
print('\nStep 5 LAYERING')

shutil.rmtree('c:\\Users\\User\\Desktop\\陳昱廷\\Layering\\Layertraj')
os.makedirs('c:\\Users\\User\\Desktop\\陳昱廷\\Layering\\Layertraj')

# Explanation:
# Layer1Depth : the thickness between first layer and base.
# LayerDepth : the thickness of other layers after first layer.
PointNormal = Lyr.Layering(NewPoint, NewPointIdx, NewNormal, tree, Vertice2, LayerDepth=Layer_height)
# PointNormal = Lyr1.Layering(NewPoint, NewPointIdx, NewNormal, tree, Vertice2, LayerDepth=Layer_height, Layer1Depth=Layer1_height)
# Grindtraj = Detraj.TrajPoint(file=BPoint)
# Grindtraj = Detraj.TrajPoint(file=NPoint)

end_time = time.time()
print('[Duration:', int((end_time-start_time)/60), 'minutes', int((end_time-start_time) % 60), 'seconds]')



# ########## THIS IS FOR CLUSTERING THE LAYER (Eg. GROUPING THE LAYER AT THE TOP AND BOTTOM BLADE SURFACE ##############
# #6. Clustering
# start_time = time.time()
# print('\nStep 6 CLUSTERING')
#
# # Explanation:
# # This is for clustering the points on Top(cluster1) and Bottom(cluster2)
# # Make sure the program contains layer folder
# TopWingPointList, TopWingPointNormalList, BottomWingPointList, BottomWingPointNormalList = Clustering.LayerCluster(ActiveTheColoredCluster=True)

# end_time = time.time()
# print('[Duration:', int((end_time-start_time)/60), 'minutes', int((end_time-start_time) % 60), 'seconds]')
# #7. ConvexPoint (for creating the edge of surface fitting) (This algorithm is not perfect)
# start_time = time.time()
# print('\nStep 7 CONVEX POINT')
#
# print('Distance of each Grid Point =', GridPoint_dist)

# #8. Surface fitting top wing (This algorithm is not perfect)
# print('\n== Top Part ==')
# ConvexPoint.ProjectTo2D(TopWingPointList, TopWingPointNormalList, part='Top', GridPoint_dist=GridPoint_dist)
#
# # Surface fitting bottom wing (This algorithm is not perfect)
# print('== Bottom Part ==')
# ConvexPoint.ProjectTo2D(BottomWingPointList, BottomWingPointNormalList, part='Bottom', GridPoint_dist=GridPoint_dist)
#
#
# end_time = time.time()
# print('[Duration:', int((end_time-start_time)/60), 'minutes', int((end_time-start_time) % 60), 'seconds]')

########################################################################################################################


print('\nPROGRAM ENDED')
Prog_end_time = time.time()
print('[Running Program Duration:', int((Prog_end_time-Prog_start_time)/60), 'minutes', int((Prog_end_time-Prog_start_time) % 60), 'seconds]')



