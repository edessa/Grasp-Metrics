#!/usr/bin/env python

import sys
import string
import os.path
import getopt


def convert(stlfilename, objfilename):

    # use options
    useBinary = False
    useFast = False

    pointList = []
    facetList = []

    def GetPointId(point,list, useFast):
        if not useFast:
            for i,pts in enumerate(list):
                if pts[0] == point[0] and pts[1] == point[1] and pts[2] == point[2] :
                    #obj start to count at 1
                    return i+1
        list.append(point)
        #obj start to count at 1
        return len(list)

    # start reading the STL file
    stlfile = open(stlfilename, "r")
    line = stlfile.readline()
    lineNb = 1
    while line != "":
        tab = string.split(string.strip(line))
        if len(tab) > 0:
            if cmp(tab[0],"facet") == 0:
                vertices = []
                normal = map(float,tab[2:])
                while cmp(tab[0],"endfacet") != 0:
                    if cmp(tab[0],"vertex") == 0:
                        pts = map(float,tab[1:])
                        vertices.append(GetPointId(pts,pointList,useFast))
                    line = stlfile.readline()
                    lineNb = lineNb +1
                    tab = string.split(string.strip(line))
                if len(vertices) == 0:
                    print_error("Unvalid facet description at line ",lineNb)
                facetList.append({"vertices":vertices, "normal": normal})

        line = stlfile.readline()
        lineNb = lineNb +1


    stlfile.close()



    # Write the target file
    objfile = open(objfilename, "w")
    objfile.write("# File type: ASCII OBJ\n")
    objfile.write("# Generated from "+os.path.basename(stlfilename)+"\n")

    for pts in pointList:
        objfile.write("v "+string.join(map(str,pts)," ")+"\n")

    for f in facetList:
        objfile.write("f "+string.join(map(str,f["vertices"])," ")+"\n")

    objfile.close()
