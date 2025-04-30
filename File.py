import numpy as np
import math as math
from MinMax import MinMax

def SaveFile(Pcd_File_Name, PCDList):
    np.savetxt('{}.xyz'.format(Pcd_File_Name), PCDList, delimiter=' ')
    print('Saved File: [{}.xyz].'.format(Pcd_File_Name))

def ReadSTLFile(filename):
    print('File Path:',filename)
    f = open(filename, "r")
    lines = f.readlines()

    VerticesList = []
    NormalList = []
    VertNorm = []

    VerticesIdx = -1  # Initial the index number of Vertices
    for x in range(0, len(lines)):
        RawData = lines[x].strip().split(" ")

        if RawData[0] == 'facet':
            NormalMagn = math.sqrt(pow(float(RawData[2]),2)+pow(float(RawData[3]),2)+pow(float(RawData[4]),2))
            NormalList.append([np.float(RawData[2])/NormalMagn, np.float(RawData[3])/NormalMagn, np.float(RawData[4])/NormalMagn])

        elif RawData[0] == 'outer':
            start = 0
            VerticesIdx = VerticesIdx + 1 # Buat kasih tau ini vertices yg urutan ke berapa
            VerticesList.append([])

        elif RawData[0] == 'vertex' and start > -1:
            VerticesList[VerticesIdx].append([np.float(RawData[1]), np.float(RawData[2]), np.float(RawData[3])]) #vertex X Y Z

        elif RawData[0] == 'endloop':
            start = -1

    for x in range(0, len(VerticesList)):
        VertNorm.append([VerticesList[x][0][0] + 0.1*(NormalList[x][0]), VerticesList[x][0][1] + 0.1 * (NormalList[x][1]), VerticesList[x][0][2] + 0.1 * (NormalList[x][2])])

    print('No of Vertices [STL]:', len(VerticesList))
    return VerticesList, NormalList, VertNorm

def ReadPcdFile(filename):
    print('File Path:', filename)
    f = open(filename, "r")
    lines = f.readlines()

    VerticeList = []

    for x in range(0, len(lines)):
        RawData = lines[x].strip().split(" ")  # [x y z] from File

        if RawData[0].replace('.', '', 1).isdigit():
            VerticeList.append([float(RawData[0]), float(RawData[1]), float(RawData[2])])

    print('No of Points [PCD]:', len(VerticeList))
    return VerticeList

def ReadPcdFileVx(filename):
    print('File Path:', filename)
    f = open(filename, "r")
    lines = f.readlines()

    NormalList= []
    NormalValue = []
    VerticeList = []

    minX = minY = minZ = 999999
    maxX = maxY = maxZ = -999999
    for x in range(0, len(lines)):
        RawData = lines[x].strip().split(" ") #[x y z] from File

        #//Separate Vertices and NormalValue
        if(RawData[0] == "vn"):
            #// Normal Magnitude for making Point Normal to be unit vector (length of vector = 1)
            NormalMagnitude = math.sqrt(pow(float(RawData[1]), 2) + pow(float(RawData[2]), 2) + pow(float(RawData[3]), 2))
            NormalValue.append([float(RawData[1])/NormalMagnitude, float(RawData[2])/NormalMagnitude, float(RawData[3])/NormalMagnitude])
        elif(RawData[0] == "v"):
            VerticeList.append([float(RawData[1]), float(RawData[2]), float(RawData[3])])
            minX,maxX = MinMax(minX, maxX, float(RawData[1]))
            minY,maxY = MinMax(minY, maxY, float(RawData[2]))
            minZ,maxZ = MinMax(minZ, maxZ, float(RawData[3]))


        elif(RawData[0] == "#" ):
            continue

    BoundaryRange = [minX,maxX,minY,maxY,minZ,maxZ]

    #// Normal = Vertice + NormalValue
    for x in range(0, len(VerticeList)):
        NormalList.append([(NormalValue[x][0]),
                          (NormalValue[x][1]),
                           (NormalValue[x][2])])

    print('No of Points [OBJ]:', len(VerticeList))
    return VerticeList, NormalList, BoundaryRange

def ReadXyzFile(filename):
    print('File Path:', filename)
    f = open(filename, "r")
    lines = f.readlines()
    print('No of Points [XYZ]:', len(lines))
    PointList = []

    for x in range(0, len(lines)):
        RawData = lines[x].strip().split(" ") #[x y z] from File
        PointList.append([float(RawData[0]), float(RawData[1]), float(RawData[2])])


    return PointList

def ReadXYZNormalFile(filename):
    print('File Path:', filename)
    f = open(filename, "r")
    lines = f.readlines()
    print('No of Points [XYZ]:', len(lines))
    PointList = []
    PointIdx = []
    NormalList = []

    for x in range(0, len(lines)):
        RawData = lines[x].strip().split(" ") #[x y z] from File

        # print(RawData)
        # print(float(RawData[0]))
        PointList.append([float(RawData[0]), float(RawData[1]), float(RawData[2])])

        NormalMagn = math.sqrt(pow(float(RawData[3]), 2) + pow(float(RawData[4]), 2) + pow(float(RawData[5]), 2))
        if NormalMagn == 0:
            NormalMagn = 0.00001
        NormalList.append([np.float(RawData[3]) / NormalMagn, np.float(RawData[4]) / NormalMagn, np.float(RawData[5]) / NormalMagn])


        # NormalList.append([float(RawData[3]), float(RawData[4]), float(RawData[5])])
        PointIdx.append(False)


    return PointList, PointIdx, NormalList

def ReadLayerFile(filename):
    print('Read File Path: [{}]'.format(filename))
    f = open(filename, "r")
    lines = f.readlines()

    Data = []
    for x in range(0, len(lines)):
        a = lines[x].strip().split(" ")
        Data.append([float(a[0]), float(a[1]), float(a[2])])

    return Data

def Readmatricfile(filename):
    print('File Path:', filename)
    f = open(filename, "r")
    lines = f.readlines()
    # print('No of Points [XYZ]:', len(lines))
    PointList = []

    for x in range(0, len(lines)):
        RawData = lines[x].strip().split() #[x y z] from File
        # print('r = ', len(RawData))
        PointList.append([float(RawData[0]), float(RawData[1]), float(RawData[2]), float(RawData[3])])


    return PointList


def CheckFileFormat(filename):
    for letter_no in range(len(filename) - 1, -1, -1):
        if str(filename[letter_no]) == '.':
            # To see " . " is at which character number
            dot = letter_no
            break

    file_format = (filename[dot:len(filename)])
    return file_format