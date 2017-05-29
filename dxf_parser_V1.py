#!/usr/bin/python
from __future__ import  print_function, division
import os
import math
import numpy
import unittest2
import dxfgrabber

from preflow_dispensing_module.path_parsing import Frame, MoveCmd, TrajectoryParserBase
from preflow_dispensing_module.trajectories import Pose, LinearSegment, CircularSegment, Trajectory
from preflow_dispensing_module.circlemath import arc_middle_from_center



class DXFParser(TrajectoryParserBase):

    def __init__(self):
        super(DXFParser, self).__init__()
        print ("Initializing DXF file parser")

        self._unit_factor = 1  # unit factor to apply to convert drawing units to meters [SI units]



    def __pointxyz(self, ent_Point):

        inner_trj = Trajectory()  # Define an empty inner trajectory movement

        xyz_Point = [ent_Point[0], ent_Point[1], ent_Point[2]] #convert tuple1 to list

        print("Coordinates of single point {}".format(xyz_Point))

        sp = Pose(xyz_Point) #Creat a position holding the co-ordinates of single point
        inner_trj.segments.append(sp)

        #return sp
        return inner_trj

    def __linexyz(self, ent_S, ent_E):
        """
        :param ent_S: start point of line
        :param ent_E: end point of line
        :return: a trajectory with line made of linear segment
        """

        inner_trj = Trajectory()  # Define an empty inner trajectory movement

        print("START {}".format(ent_S))
        print("END {}".format(ent_E))

        s = Pose(ent_S) #Define start position
        e = Pose(ent_E) #Define end position

        line = LinearSegment(s, e) #Creat a linear movement segment and insert two coordinate points
        inner_trj.segments.append(line)

        #return line
        return inner_trj

    def __circlexyz(self, ent_C, ent_R, ent_Evct):

        inner_trj = Trajectory()  # Define an empty trajectory movement

        xyz_Center = [ent_C[0], ent_C[1], ent_C[2]] #convert tuple1 to list
        radious = ent_R #hold the radious length
        print("Extrusion direction vector for both arcs X: {}, Y: {}, Z: {}".format(*ent_Evct))
        dir_Vector= [ent_Evct[0], ent_Evct[1], ent_Evct[2]]

        x_S = xyz_Center[0] + radious
        y_S = xyz_Center[1]
        z_S = xyz_Center[2] #assuming all points are in the same plane with central point

        xyz_S = [x_S, y_S, z_S] #creat a list with coordinates for start point
        print("Initial-final half arc1-arc2 point {}".format(xyz_S))

        x_E = xyz_Center[0] - radious
        y_E = xyz_Center[1]
        z_E = xyz_Center[2] #assuming all points are in the same plane with central point

        xyz_E = [x_E, y_E, z_E]  # create a list with coordinates for end point
        print("Final-initial half arc1-arc2 point{}".format(xyz_E))

        # calculate coordinates for mid point in first arc
        array_M_arc1 = arc_middle_from_center(xyz_S, xyz_E, xyz_Center, dir_Vector) #returns array
        xyz_M_arc1 = [array_M_arc1[0], array_M_arc1[1], array_M_arc1[2]] #convert array to list
        print("First ARC1 middle Point{}".format(xyz_M_arc1))

        s = Pose(xyz_S) #Define start position
        e = Pose(xyz_E) #Define end position
        m = Pose(xyz_M_arc1) #Define arc middle point position

        arc1 = CircularSegment(s, e, m)
        inner_trj.segments.append(arc1)

        # calculate coordinates for mid point in second arc
        array_M_arc2 = arc_middle_from_center(xyz_E, xyz_S, xyz_Center, dir_Vector) #returns array
        xyz_M_arc2 = [array_M_arc2[0], array_M_arc2[1], array_M_arc2[2]] #convert array to list
        print("First ARC1 middle Point{}".format(xyz_M_arc2))

        s = Pose(xyz_S) #Define start position
        e = Pose(xyz_E) #Define end position
        m = Pose(xyz_M_arc2) #Define arc middle point position

        arc2 = CircularSegment(s, e, m)
        inner_trj.segments.append(arc2)

        #return {'arc1':arc1, 'arc2':arc2}
        return inner_trj


    def __arcxyz(self, ent_C, ent_R, ent_degS, ent_degE, ent_Evct):
        inner_trj = Trajectory()  # Define an empty trajectory movement

        xyz_Center = [ent_C[0], ent_C[1], ent_C[2]] #convert tuple1 to list
        radious = ent_R #hold the radious length
        rad_S = ent_degS/(180/math.pi) #convert degrees to rads for the start point angle
        rad_E = ent_degE/(180/math.pi) # convert degrees to rads for the end point angle
        print("S rad: {}".format(rad_S))
        print("E rad: {}".format(rad_E))
        print("Extrusion direction vector X: {}, Y: {}, Z: {}".format(*ent_Evct))
        dir_Vector = [ent_Evct[0], ent_Evct[1], ent_Evct[2]]

        x_S = xyz_Center[0]+(radious * math.cos(rad_S))
        y_S = xyz_Center[1]+(radious * math.sin(rad_S))
        z_S = xyz_Center[2] #assuming all points are in the same plane with central point

        xyz_S = [x_S, y_S, z_S] #creat a list with coordinates for start point
        print("START {}".format(xyz_S))

        x_E = xyz_Center[0]+(radious * math.cos(rad_E))
        y_E = xyz_Center[1]+(radious * math.sin(rad_E))
        z_E = xyz_Center[2] #assuming all points are in the same plane with central point

        xyz_E = [x_E, y_E, z_E]  # create a list with coordinates for end point
        print("END {}".format(xyz_E))

        # calculate coordinates for mid point in arc
        array_M = arc_middle_from_center(xyz_S, xyz_E, xyz_Center, dir_Vector) #returns array
        xyz_M = [array_M[0], array_M[1], array_M[2]] #convert array to list
        print("ARC MIDDLE {}".format(xyz_M))

        s = Pose(xyz_S) #Define start position
        e = Pose(xyz_E) #Define end position
        m = Pose(xyz_M) #Define arc middle point position

        arc = CircularSegment(s, e, m)
        inner_trj.segments.append(arc)

        #return arc
        return inner_trj

    def __calc_center(self, v1, bulge, v2):
        # Method for calculating center of arc
        chord = numpy.subtract(v2, v1)
        # Calculate chord length- distance between start and end points
        chord_length = numpy.linalg.norm(chord)
        # Calculate height of the arc (sangitta)
        sagitta = (bulge * chord_length) / 2.0
        # Calculate inner angle theta given bulge in radians
        inc_angle = numpy.arctan(bulge) * 4.0
        # Calculate radius
        radius = (chord_length / 2.0) / numpy.sin(inc_angle / 2.0)
        if bulge >= 0:
            perp = (numpy.cross(chord, (0, 0, -1)))
        else:
            perp = (numpy.cross(chord, (0, 0, -1)))
        # Calculate chord migle point
        chord_mid_pt = numpy.add(numpy.multiply(chord, (.5, .5, .5)), v1)
        unit_vec = perp / numpy.linalg.norm(perp)
        # Calculate center point of the arc
        arc_center = numpy.add(numpy.multiply((radius - sagitta), unit_vec), chord_mid_pt)
        return arc_center

    def __lwpolylinexyz(self, ent_Points, ent_Bulge, ent_closed, ent_Evct):
        inner_trj = Trajectory()  # Define an empty trajectory movement or "move" in rcw_interface.py file

        i = 0
        #if ent_closed == False: #Case: LWpolyline is open
        for i in range(len(ent_Points) - 1):
            fstP = ent_Points[i]
            sndP = ent_Points[i + 1]
            print("Point1: {} Point2: {} Bulge {}".format(fstP, sndP, ent_Bulge[i]))
            if ent_Bulge[i] == 0:
                l_segmnt = self.__linexyz(ent_Points[i], ent_Points[i + 1])
                inner_trj.segments.append(l_segmnt)

            elif ent_Bulge[i] > 0: #CCW direction

                array_C = self.__calc_center(fstP , ent_Bulge[i], sndP)
                xyz_Center = [array_C[0], array_C[1], array_C[2]]  # convert array to list
                xyz_S = [fstP[0], fstP[1], fstP[2]]  # convert tuple1 to list
                xyz_E = [sndP[0], sndP[1], sndP[2]]  # convert tuple1 to list
                dir_Vector = [0, 0, 1] # set direction in vector CCW

                print("Center poin of arc {}".format(xyz_Center))

                # calculate coordinates for mid point in arc
                array_M = arc_middle_from_center(xyz_S, xyz_E, xyz_Center, dir_Vector)  # returns array
                xyz_M = [array_M[0], array_M[1], array_M[2]]  # convert array to list
                print("ARC MIDDLE {}".format(xyz_M))

                s = Pose(xyz_S)  # Define start position
                e = Pose(xyz_E)  # Define end position
                m = Pose(xyz_M)  # Define arc middle point position

                arc_segmnt = CircularSegment(s, e, m)
                #arc_segmnt = CircularSegment(ent_Points[i], ent_Points[i + 1], _)
                inner_trj.segments.append(arc_segmnt)
            else:  # CW direction
                array_C = self.__calc_center(fstP , ent_Bulge[i], sndP)
                xyz_Center = [array_C[0], array_C[1], array_C[2]]  # convert array to list
                xyz_S = [fstP[0], fstP[1], fstP[2]]  # convert tuple1 to list
                xyz_E = [sndP[0], sndP[1], sndP[2]]  # convert tuple1 to list
                dir_Vector = [0, 0, -1] # set direction in vector CW

                print("Center poin of arc {}".format(xyz_Center))

                # calculate coordinates for mid point in arc
                array_M = arc_middle_from_center(xyz_S, xyz_E, xyz_Center, dir_Vector)  # returns array
                xyz_M = [array_M[0], array_M[1], array_M[2]]  # convert array to list
                print("ARC MIDDLE {}".format(xyz_M))

                s = Pose(xyz_S)  # Define start position
                e = Pose(xyz_E)  # Define end position
                m = Pose(xyz_M)  # Define arc middle point position

                arc_segmnt = CircularSegment(s, e, m)
                #arc_segmnt = CircularSegment(ent_Points[i], ent_Points[i + 1], _)
                inner_trj.segments.append(arc_segmnt)

        #**********************************************
        if ent_closed == True:  # Case: LWpolyline is Closed
        #**********************************************
            PointsNo = len(ent_Points) - 1
            fstP = ent_Points[PointsNo]
            sndP = ent_Points[0]
            print("Point last: {} Point first: {} Bulge {}".format(fstP, sndP, ent_Bulge[PointsNo]))
            if ent_Bulge[PointsNo] == 0:
                l_segmnt = self.__linexyz(fstP, sndP)
                inner_trj.segments.append(l_segmnt)

            elif ent_Bulge[PointsNo] > 0:  # CCW direction

                array_C = self.__calc_center(fstP, ent_Bulge[PointsNo], sndP)
                xyz_Center = [array_C[0], array_C[1], array_C[2]]  # convert array to list
                xyz_S = [fstP[0], fstP[1], fstP[2]]  # convert tuple1 to list
                xyz_E = [sndP[0], sndP[1], sndP[2]]  # convert tuple1 to list
                dir_Vector = [0, 0, 1]  # set direction in vector CCW

                print("Center poin of arc {}".format(xyz_Center))

                # calculate coordinates for mid point in arc
                array_M = arc_middle_from_center(xyz_S, xyz_E, xyz_Center, dir_Vector)  # returns array
                xyz_M = [array_M[0], array_M[1], array_M[2]]  # convert array to list
                print("ARC MIDDLE {}".format(xyz_M))

                s = Pose(xyz_S)  # Define start position
                e = Pose(xyz_E)  # Define end position
                m = Pose(xyz_M)  # Define arc middle point position

                arc_segmnt = CircularSegment(s, e, m)
                # arc_segmnt = CircularSegment(ent_Points[i], ent_Points[i + 1], _)
                inner_trj.segments.append(arc_segmnt)
            else:  # CW direction
                array_C = self.__calc_center(fstP, ent_Bulge[PointsNo], sndP)
                xyz_Center = [array_C[0], array_C[1], array_C[2]]  # convert array to list
                xyz_S = [fstP[0], fstP[1], fstP[2]]  # convert tuple1 to list
                xyz_E = [sndP[0], sndP[1], sndP[2]]  # convert tuple1 to list
                dir_Vector = [0, 0, -1]  # set direction in vector CW

                print("Center poin of arc {}".format(xyz_Center))

                # calculate coordinates for mid point in arc
                array_M = arc_middle_from_center(xyz_S, xyz_E, xyz_Center, dir_Vector)  # returns array
                xyz_M = [array_M[0], array_M[1], array_M[2]]  # convert array to list
                print("ARC MIDDLE {}".format(xyz_M))

                s = Pose(xyz_S)  # Define start position
                e = Pose(xyz_E)  # Define end position
                m = Pose(xyz_M)  # Define arc middle point position

                arc_segmnt = CircularSegment(s, e, m)
                # arc_segmnt = CircularSegment(ent_Points[i], ent_Points[i + 1], _)
                inner_trj.segments.append(arc_segmnt)
        #**********************************************
        return inner_trj

    def _set_unit_factor(self, dxf):
        """
        set the conversion factor to meters from a dxf instance(?)
        :param dxf: dxf instance from dxfgrabber.readfile
        :return: None
        """
        unit = dxf.header['$INSUNITS']  # Extract file units from the header variable
        units_name = {0 : 'Unitless', 1 : 'Inches', 2 : 'Feet', 3 : 'Miles', 4 : 'Millimeters', 5 : 'Centimeters', 6 : 'Meters', 7 : 'Kilometers', 8 :  'Microinches', 9 : 'Mils', 10 : 'Yards', 11 : 'Angstroms', 12 : 'Nanometers', 13 : 'Microns', 14 : 'Decimeters', 15 : 'Decameters', 16 : 'Hectometers', 17 : 'Gigameters', 18 : 'Astronomical units', 19 : 'Light years', 20 : 'Parsecs'}

        if unit == 0:
            # assume unitless is at the right scale
            self._unit_factor = 1.0
        elif unit == 1:
            self._unit_factor = 25.4 / 1000
        elif unit == 4:
            self._unit_factor = 1 / 1000
        elif unit == 5:
            self._unit_factor = 1 / 100
        elif unit == 6:
            self._unit_factor = 1
        else:
            print("Unit %s not supported by parser. Will assume the file is in meters." % units_name[unit])

        print("Units in file: {}" .format(units_name[unit]))


    def __convert_units(self, ent_Points):

        ent_Points_scalling = []
        for p in ent_Points:
            ent_Points_scalling.append([self._unit_factor * p[0],
                                        self._unit_factor * p[1],
                                        self._unit_factor * p[2]])
        return ent_Points_scalling

    #Main loop for parsing entites from dxf file
    def parse_file(self, filepath):

        filepath = os.path.expanduser(filepath)

        DEFAULT_OPTIONS = {
            "grab_blocks": True,
            "assure_3d_coords": True, #False, #Set False for XY only coordinates
            "resolve_text_styles": True,
        }

        dxf = dxfgrabber.readfile(filepath, DEFAULT_OPTIONS)
        print("DXF version: {}".format(dxf.dxfversion))
        print("DXF file name: {}".format(dxf.filename)) #Filename

        #Show the layers in file
        layer_names = [l.name for l in dxf.layers]
        print("Number of layers: {}\n"
              "Layers: {}".format(len(dxf.layers), layer_names))

        #block_definition_count = len(dxf.blocks)  # dict like collection of block definitions
        #print (block_definition_count)

        #See the header variables for extra  file info
        #header_var = dxf.header  # dict of dxf header vars
        #print("Header variables: {}".format(header_var))
        #Default drawing units for AutoCAD DesignCenter blocks:
        self._set_unit_factor(dxf)


        select_layer = '0' #Name of selected layer, I assume that all import entities are at layer 0
        file_entities_count = len(dxf.entities)
        layer_entities = [lentity for lentity in dxf.entities if lentity.layer == select_layer]
        layer_entities_count = len(layer_entities)

        print("All entities No. in file: {}".format(file_entities_count))
        print("Entities No. in layer {}: {}\n"
              "\n".format(select_layer, layer_entities_count))

        if file_entities_count <= 0: #If file is empty
            print ("Empty file no drawing entities")
        elif file_entities_count != 0 and layer_entities_count <= 0:
            print ("Empty layer no drawing entities, select a different layer")
        else:
            t = Trajectory()  # Define an empty trajectory movement or "move" in rcw_interface.py file

            for entity in dxf.entities:
            # Read from modelspace only, scale 1:1 and select one layer for reading
                if not entity.paperspace and entity.layer == select_layer:
                    print("Entity type: {}".format(entity.dxftype))
                    if entity.dxftype == "POINT":
                        # scale to SI units [meters]
                        Point_scaled = [self._unit_factor * el for el in entity.point]  # convert tuple to list and scale the units
                        p = self.__pointxyz(Point_scaled)
                        t.segments.append(p)
                        self.trajectories.append(p)
                        print("Point XYZ: {}".format(entity.point))

                    elif entity.dxftype == "LINE":
                        #scale to SI units [meters]
                        start_Point_scaled = [self._unit_factor * el for el in entity.start]# convert tuple to list and scale the units
                        end_Point_scaled = [self._unit_factor * el for el in entity.end]  # convert tuple to list and scale the units

                        #l = self.__linexyz(entity.start, entity.end)
                        l = self.__linexyz(start_Point_scaled, end_Point_scaled)
                        #t.segments.append(l)  # Fill the trajectory appending consequently segments
                        self.trajectories.append(l)
                        #print("Number of segments in trajectory: {}".format(len(t.segments)))
                        print("Number of segments in trajectory: {}".format(len(l.segments)))
                    elif entity.dxftype == "CIRCLE":
                        # scale to SI units [meters]
                        center_Point_scaled = [self._unit_factor * el for el in entity.center]  # convert tuple to list and scale the units
                        radius_Point_scaled = self._unit_factor * entity.radius  # convert tuple to list and scale the units

                        #c = self.__circlexyz(entity.center, entity.radius, entity.extrusion)
                        c = self.__circlexyz(center_Point_scaled, radius_Point_scaled, entity.extrusion)
                        #t.segments.append(c['arc1'])
                        #t.segments.append(c['arc2'])
                        self.trajectories.append(c)
                        print("Number of segments in trajectory: {}".format(len(c.segments)))
                        print("Center XYZ: {}".format(entity.center))
                        print("Radious leng: {}".format(entity.radius))
                    elif entity.dxftype == "ARC":
                        # scale to SI units [meters]
                        center_Point_scaled = [self._unit_factor * el for el in entity.center]  # convert tuple to list and scale the units
                        radius_Point_scaled = self._unit_factor * entity.radius  # convert tuple to list and scale the units

                        a = self.__arcxyz(center_Point_scaled, radius_Point_scaled, entity.start_angle, entity.end_angle, entity.extrusion)
                        #t.segments.append(a)
                        self.trajectories.append(a)
                        print("Center XYZ: {}".format(entity.center))
                        print("Radious leng: {}".format(entity.radius))
                        print("Start angle: {}".format(entity.start_angle))
                        print("End angle: {}".format(entity.end_angle))
                    elif entity.dxftype == "LWPOLYLINE" or entity.dxftype == "POLYLINE":  # Partially support for polyline (2D only)

                        ent_Points_scaled = self.__convert_units(entity.points)

                        """
                        # convert points to units
                        ent_Points_scaled = []
                        for p in entity.points:
                            ent_Points_scaled.append([self._unit_factor * p[0],
                                                      self._unit_factor * p[1],
                                                      self._unit_factor * p[2]])
                        """

                        lwp = self.__lwpolylinexyz(ent_Points_scaled, entity.bulge, entity.is_closed, entity.extrusion)

                        #t.segments.append(lwp)
                        self.trajectories.append(lwp)
                        print("LWPOLY Points XYZ: {}".format(entity.points))
                        print("Bulge: {}".format(entity.bulge))
                        print("Is closed ?: {}".format(entity.is_closed))
                    else:
                        raise NotImplementedError("Not supported structure entity: {}".format(entity.dxftype))
            return

"""
Execute as individual program the following part of code, !!if class imported this part is not executed!! 
"""
if __name__ == '__main__':

    filepath = "~/RCW_workspaces/sandbox_ws/src/preflow_dispensing_module/test/drawing1.dxf"
    #filepath = "drawing.dxf"
    #DXFfile = os.path.expanduser(filepath)

    parser=DXFParser()
    m = parser.parse_file(filepath)
    #m = movement.impdxf(DXFfile)

    #print("\nFinal Number of segments in trajectory: {}".format(len(m)))
    print("\nFinal Number of trajectories: {}".format(len(parser.trajectories)))


    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D # required for 3d plot

    # make figure
    fig = plt.figure()
    ax = fig.gca(projection='3d')
    ax.set_aspect('equal')
    ax.set_autoscale_on('True')
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.set_zlabel('z [m]')

    for t in parser.trajectories:
        t.plot(ax)

    plt.show()
    plt.gcf().canvas.get_supported_filetypes()
    #_= raw_input("Press enter to exit.")
