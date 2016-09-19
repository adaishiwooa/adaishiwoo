#!/usr/bin/env python
# SYS
import argparse
import numpy as np
import yaml
from uuid import uuid4
import math
# CRI
from printx_controller.conversions import (
    points_to_ros_workspace_trajectory
)
import printx_controller.patterns as patt
from criprint_msgs.srv import (
    PlanWorkspaceTrajectory,
    PlanWorkspaceTrajectoryRequest,
    PlanUndirectedPrint,
    PlanUndirectedPrintRequest,
    UpdatePrintedBodies,
    UpdatePrintedBodiesRequest

)
# OR
# ROS
import rospy


class PrintingDemo(object):
    """ Responsible for constructing and sending printing configurations
    """
    def __init__(self):
        """ Constructor for printing demo.

        Construct all 3D world's information. Then constructs both planner and
        executor object.
        Attributes:
            robot_name      (string):   Name of robot, needed to setup correct services' name.
            zeroed_height   (float) :   Ground level height of the printing plate
            first_layer_height (float)  : Distance from the ground level to the first printing layer
            norm_layer_height   (float) : Distance between consecutive layers

        Subscribed Services:
            /denso/plan_workspace_trajectory           : Plan a single workspace trajectory
            /denso/printing/update_printed_bodies      : Update Rave's world with a simulated
        printing path
        """
        # Get parameters
        robot_name = rospy.get_param('/denso/robot_name', 'denso')

        # Printing related parameters
        #self.zeroed_height = 0.413-0.014 #+0.145 #table
        self.zeroed_height = 0.413
        # 0.03
        # conrete
        # self.first_layer_height = 7e-3
        # self.norm_layer_height = 10e-3
        # clay
        self.first_layer_height = 8.5e-3
        self.norm_layer_height = 8.5e-3
        # Services
        servicename = 'plan_workspace_trajectory'
        self.plan_service = rospy.ServiceProxy(
            '{0}/{1}'.format(robot_name, servicename), PlanWorkspaceTrajectory
        )
        self.plan_service.wait_for_service()
        rospy.loginfo("[{0}] service is loaded!".format(servicename))

        servicename = 'printing/update_printed_bodies'
        self.update_printed_bodies_service = rospy.ServiceProxy(
            '{0}/{1}'.format(robot_name, servicename), UpdatePrintedBodies
        )
        self.update_printed_bodies_service.wait_for_service()
        rospy.loginfo("[{0}] service is loaded!".format(servicename))

        servicename = 'printing/plan_undirected_printing'
        self.plan_undirected_printing = rospy.ServiceProxy(
            '{0}/{1}'.format(robot_name, servicename), PlanUndirectedPrint
        )
        self.plan_undirected_printing.wait_for_service()
        rospy.loginfo("[{0}] service is loaded!".format(servicename))

    def print_single_straight_walls(self):
        """ Demo function. Plan a single wall.

        Args:
            length  (float)     : Length of the wall
            dir_right (bool)    : Direction of the z-axis during printing

        Returns:
            succeed (bool)  : True if planning is successful.
        """
        # Params
        self.first_layer_height = 0.007
        self.norm_layer_height = 0.006
        length = 0.4
        x0 = 0.70
        no_layer = 6

        # Setup configuration
        segments = []
        z_list = [self.zeroed_height + self.first_layer_height]
        # Number of elements in z_list equals to number of layers
        for _i in range(no_layer - 1):
            z_list.append(z_list[-1] + self.norm_layer_height)
        for i, z in enumerate(z_list):
            if i % 2 == 0:
                single_segment = np.array([[x0, y, z] for y in np.linspace(length / 2, - length / 2)])
                print single_segment
                segments.append(single_segment)
            # Reverse direction when going from left to right
            else:
                single_segment = np.array([[x0, y, z] for y in np.linspace(- length / 2, length / 2)])
                segments.append(single_segment)
        x0 = 0.65
        for i, z in enumerate(z_list):
            if i % 2 == 0:
                single_segment = np.array([[x0, y, z] for y in np.linspace(length / 2, - length / 2)])
                print single_segment
                segments.append(single_segment)
            # Reverse direction when going from left to right
            else:
                single_segment = np.array([[x0, y, z] for y in np.linspace(- length / 2, length / 2)])
                segments.append(single_segment)
        x0 = 0.60
        for i, z in enumerate(z_list):
            if i % 2 == 0:
                single_segment = np.array([[x0, y, z] for y in np.linspace(length / 2, - length / 2)])
                print single_segment
                segments.append(single_segment)
            # Reverse direction when going from left to right
            else:
                single_segment = np.array([[x0, y, z] for y in np.linspace(- length / 2, length / 2)])
                segments.append(single_segment)
        

        velocity = 240e-3  # 20 mm per sec
        request = PlanUndirectedPrintRequest()
        request.velocity = velocity
        request.serialized_segments = yaml.dump(segments)
        # Send plan request to robot motion server
        succeed = self.plan_undirected_printing.call(request)
        return succeed

    def print_box(self):
        """ Demo Function, plan a box.
        Args:
            length  (float)     : Length of the wall
            dir_right (bool)    : Direction of the z-axis during printing

        Returns:
            succeed (bool)  : True if planning is successful.
        """
        self.first_layer_height = 0.014
        self.norm_layer_height = 0.014
        # Params
        x_length = 0.8
        y_length = 0.6
        length = 0.7
        y0 = 0.4
        x0 = 0.8
        x1 = 0.4
        y1 = -0.4
        no_layer = 3

        # Setup configuration
        segments = []
        z_list = [self.zeroed_height + self.first_layer_height]
        # Number of elements in z_list equals to number of layers
        for _i in range(no_layer - 1):
            z_list.append(z_list[-1] + self.norm_layer_height)
            
        for i, z in enumerate(z_list):
           
                single_segment = np.array([[x0, y, z] for y in np.linspace(-y_length/2, y_length/2)])
                segments.append(single_segment)
                single_segment = np.array([[x, y0, z] for x in np.linspace(length, 0.4)])
                
                segments.append(single_segment)

                single_segment = np.array([[x1, y, z] for y in np.linspace(y_length/2, -y_length/2)])
                segments.append(single_segment)
                single_segment = np.array([[x, y1, z] for x in np.linspace(0.4, length)])
                segments.append(single_segment)
                
        print segments
        velocity = 120e-3  # 20 mm per sec
        request = PlanUndirectedPrintRequest()
        request.velocity = velocity
        request.serialized_segments = yaml.dump(segments)
        # Send plan request to robot motion server
        succeed = self.plan_undirected_printing.call(request)
        return succeed


    def print_x_direction(self):
        """ Demo function. Plan a single wall.

        Args:
            length  (float)     : Length of the wall
            dir_right (bool)    : Direction of the z-axis during printing

        Returns:
            succeed (bool)  : True if planning is successful.
        """
        """
        self.first_layer_height = 0.0014
        self.norm_layer_height = 0.0014
        """
        # Params
        length = 0.8
        y0 = 0.02
        no_layer = 8

        # Setup configuration
        segments = []
        z_list = [self.zeroed_height + self.first_layer_height]
        # Number of elements in z_list equals to number of layers
        for _i in range(no_layer - 1):
            z_list.append(z_list[-1] + self.norm_layer_height)
        for i, z in enumerate(z_list):
            if i % 2 == 0:
                single_segment = np.array([[x, y0, z] for x in np.linspace(length, 0.5)])
                print single_segment
                segments.append(single_segment)
            # Reverse direction when going from left to right
            else:
                single_segment = np.array([[x, y0, z] for x in np.linspace(0.5, length)])
                segments.append(single_segment)
                print single_segment
        velocity = 120e-3  # 20 mm per sec
        request = PlanUndirectedPrintRequest()
        request.velocity = velocity
        request.serialized_segments = yaml.dump(segments)
        # Send plan request to robot motion server
        succeed = self.plan_undirected_printing.call(request)
        return succeed

    def print_single_sine_walls(self):
        """ Demo function. Plan a single wall.

        Args:
            alpha_min/max:  Angle of the big arcs
            wave:           Angle of the wave
            dir_right (bool)    : Direction of the z-axis during printing

        Returns:
            succeed (bool)  : True if planning is successful.
        """
        # Params
        n0 = 0.8
        no_layer = 1
        R = 0.75
        r = 0.045

        # Setup configuration
        segments_list = []
        z_list = [self.zeroed_height + self.first_layer_height]
        # Increase z-gradually
        for _i in range(no_layer - 1):
            z_list.append(z_list[-1] + self.norm_layer_height)
        for i, z in enumerate(z_list):
            if i == 0:
                n = n0 * 0.9
            else:
                n = n0
            alpha_min = np.pi / 12 / n
            alpha_max = - np.pi / 12 / n
            wave = 1.5 / n
            p = _create_sine_wall(R, r, z, alpha_min, alpha_max, wave=wave)
            if i % 2 == 0:
                segments_list.append(p)
            else:
                segments_list.append(np.flipud(p))

        
        velocity = 120e-3  # 10 mm per sec
        request = PlanUndirectedPrintRequest()
        request.velocity = velocity
        request.serialized_segments = yaml.dump(segments_list)
        succeed = self.plan_undirected_printing.call(request)
        return succeed

    def print_sin_wave(self):
        amplitude = 0.1
        distance = 0.2
        segments_list = []
        no_layer = 4
        # segment = []
        z_list = [self.zeroed_height + self.first_layer_height]
        for _i in range(no_layer - 1):
            z_list.append(z_list[-1] + self.norm_layer_height)
        for i, z in enumerate(z_list):
            segment = []
            if i % 2 == 0:
                for y in range (-180,180): 
                    x = amplitude*math.sin(math.radians(y))+0.5
                    segment.append([x, math.radians(y)/np.pi*distance, z])
                segments_list.extend(segment)
            else:
                for y in range (-180,180): 
                    temp = -1 * math.sin(math.radians(y))
                    x = amplitude*temp+0.5
                    

                    segment.append([x, math.radians(y)/np.pi*distance, z])
                segments_list.extend(segment)


        print segments_list
        segments_list = [segments_list]
        velocity = 120e-3  # 10 mm per sec
        request = PlanUndirectedPrintRequest()
        request.velocity = velocity
        request.serialized_segments = yaml.dump(segments_list)
        succeed = self.plan_undirected_printing.call(request)
        return succeed


    def print_modulating_wall(self):
        """ Demo function. Plan a modulating wall.

        Args:
            alpha               : Angle of the big arcs
            r1, r2              : Radiuses of the wall
            n   (int)           : Number of modulations
            velocity (float)    : Speed of the print
            no_layer    : Number of layers
        Returns:
            succeed (bool)  : True if planning is successful.
        """
        # Params
        alpha = np.pi / 6
        r1 = 0.78
        r2 = 0.9
        n = 3  # Number of modulations
        velocity = 50e-3
        no_layer = 15
        path_width = 20e-3  # 15 mms
        flipped = False

        # Setup configuration
        segments_list = []
        z_list = [self.zeroed_height + self.first_layer_height]
        # Increase z-gradually
        for _i in range(no_layer - 1):
            z_list.append(z_list[-1] + self.norm_layer_height)
        for i, z in enumerate(z_list):
            segment = []
            # Create the list of waypoints
            # Small radius segment
            for beta in np.linspace(- alpha / 2, alpha / 2, 100):
                segment.append([r1 * np.cos(beta), r1 * np.sin(beta), z])
            # Large radius segment
            for beta in np.linspace(alpha / 2, - alpha / 2, 100):
                segment.append([r2 * np.cos(beta), r2 * np.sin(beta), z])
            segment.append([(r1 + path_width / 2) * np.cos(- alpha / 2), (r1 + path_width / 2) * np.sin(- alpha / 2), z])
            # Modulating segments
            gamma = 1.0 * alpha / n
            for j in range(n):
                angle1 = - alpha / 2 + j * gamma + gamma / 2
                angle2 = angle1 + gamma / 2
                segment.append([(r2 - path_width / 2) * np.cos(angle1), (r2 - path_width / 2) * np.sin(angle1), z])
                segment.append([(r1 + path_width / 2) * np.cos(angle2), (r1 + path_width / 2) * np.sin(angle2), z])
            if flipped:
                if i % 2 == 0:
                    segments_list.append(segment)
                else:
                    segments_list.append(np.flipud(segment))
            else:
                segments_list.append(segment)
        request = PlanUndirectedPrintRequest()
        request.velocity = velocity
        request.serialized_segments = yaml.dump(segments_list)
        succeed = self.plan_undirected_printing.call(request)
        return succeed

    def print_hexagons_struct(self):
        """ Print a hexagonal structure
        """
        # # Clay parameters
        # rospy.loginfo("Start constructing the hexagonal structure for clay")
        # no_layer = 10
        # velocity = 30e-3  # 40 mm persec
        # path_width = 0.0015
        # edge_length = 0.05
        # Concrete parameters
        rospy.loginfo("Start constructing the hexagonal structure for concrete")
        no_layer = 6
        velocity = 120e-3  # 40 mm persec
        path_width = 0.01
        edge_length = 0.15
        # Points in 2D
        pts, dirs = patt.hexagon(np.array([0.7, 0.0]), path_width)
        #print (pts)
        #print (dirs)
        points2d = []
        for (point, direction) in zip(pts, dirs):
            points2d.extend(patt.four_triangles_pattern(point, direction, edge_length))
        points2d = np.array(points2d)
        # Points in 3D
        segment = []
        for i, ilayer in enumerate(range(no_layer)):
            z = self.zeroed_height + self.first_layer_height + i * self.norm_layer_height
            for point in points2d:
                segment.append(np.r_[point, z])
        segments_list = [segment]
        print segments_list
        request = PlanUndirectedPrintRequest()
        request.velocity = velocity
        request.serialized_segments = yaml.dump(segments_list)
        succeed = self.plan_undirected_printing.call(request)
        
        return succeed

    def print_maze(self):

        x0 = 0.8
        velocity = 160e-3  # 20 mm per sec
        vert = 0.20
        hori = 0.014
        no_layer = 5
        flipped = True
        segments_list = []
        initial_height = self.zeroed_height
        first_layer_height = 0.0085
        norm_layer_height = 0.0085
        try_h = 0.15
        # try_v = 0.4
        
        # for y in np.linspace(try_v, 0, num=150, endpoint=False):
        #     segments_list.append([x0, y, initial_height])
        # for x in np.linspace(x0, x0 + try_h, num=50, endpoint=False):
        #     segments_list.append([x, 0, initial_height])

        # for y in np.linspace(0, try_v, num=150, endpoint=False):
        #     segments_list.append([x0 + try_h, y, initial_height])
        # for x in np.linspace(x0 + try_h, x0 + 2*try_h, num=50, endpoint=False):
        #     segments_list.append([x, try_v, initial_height]) 
        flow_test = []
        # for y in np.linspace(try_v, 0, num=400, endpoint=False):
        #     flow_test.append([x0 - try_h, y, initial_height])  
        for x in np.linspace(x0 - try_h, x0, num=200, endpoint=False):
            flow_test.append([x, 0, initial_height]) 
        segments_list.extend(flow_test)

        z_list = [initial_height + first_layer_height]
        # Number of elements in z_list equals to number of layers
        for _i in range(no_layer - 1):
            z_list.append(z_list[-1] + norm_layer_height)


        for i, z in enumerate(z_list):
            segment = []

            # Create the list of waypoints
            # if i % 2 == 0:
                # first vertical line
            for y in np.linspace(0, vert, num=100, endpoint=False):
                segment.append([x0, y, z])
            # first horizontal line
            for x in np.linspace(x0, x0 + hori, num=10, endpoint=False):
                segment.append([x, vert, z])
            # second vertical line
            for y in np.linspace(vert, 0, num=100, endpoint=False):
                segment.append([x0 + hori, y, z])
            # second horizontal line
            for x in np.linspace(x0 + hori, x0 + 2*hori, num=10, endpoint=False):
                segment.append([x, 0, z])
            # third vertical line   
            for y in np.linspace(0, vert, num=100, endpoint=False):
                segment.append([x0 + 2*hori, y, z])
            # third horizontal line
            for x in np.linspace(x0 + 2*hori, x0 + 3*hori, num=10, endpoint=False):
                segment.append([x, vert, z])
            # fourth vertical line   
            for y in np.linspace(vert, 0, num=100, endpoint=False):
                segment.append([x0 + 3*hori, y, z])
            # fourth horizontal line
            for x in np.linspace(x0 + 3*hori, x0 + 4*hori, num=10, endpoint=False):
                segment.append([x, 0, z])   
            # last vertical line   
            for y in np.linspace(0, vert, num=100):
                segment.append([x0 + 4*hori, y, z]) 
            # else: 
            #     ## flipped
            #     for x in np.linspace(x0 + vert, x0, num=45, endpoint=False):
            #         segment.append([x, 4*hori, z])   
            #     for y in np.linspace(4*hori, 3*hori, num=10, endpoint=False):
            #         segment.append([x0, y, z])
            #     for x in np.linspace(x0, x0 + vert, num=45, endpoint=False):
            #         segment.append([x, 3*hori, z])   
            #     for y in np.linspace(3*hori, 2*hori, num=10, endpoint=False):
            #         segment.append([x0 + vert, y, z])
            #     for x in np.linspace(x0 + vert, x0, num=45, endpoint=False):
            #         segment.append([x, 2*hori, z])   
            #     for y in np.linspace(2*hori, hori, num=10, endpoint=False):
            #         segment.append([x0, y, z])
            #     for x in np.linspace(x0, x0 + vert, num=45, endpoint=False):
            #         segment.append([x, hori, z])   
            #     for y in np.linspace(hori, 0, num=10, endpoint=False):
            #         segment.append([x0 + vert, y, z])
            #     for x in np.linspace(x0 + vert, x0, num=45):
            #         segment.append([x, 0, z])  

            # layer printing direction
            if flipped:
                if i % 2 == 0:
                    segments_list.extend(segment)
                else:
                    segments_list.extend(np.flipud(segment))
            else:
                segments_list.extend(segment)
        segments_list = [segments_list]
        request = PlanUndirectedPrintRequest()
        request.velocity = velocity
        request.serialized_segments = yaml.dump(segments_list)
        succeed = self.plan_undirected_printing.call(request)
        return succeed


    def send_concrete_print(self, id):
        if id == 1:
            self.print_modulating_wall()
        elif id == 2:
            self.print_hexagons_struct()
        elif id == 3:
            self.print_maze()
        elif id == 4:
            self.print_single_straight_walls()
        elif id == 5:
            self.print_x_direction()
        elif id == 6:
            self.print_box()
        elif id == 7:
            self.print_sin_wave()


def _create_sine_wall(R, r, z, alpha_min, alpha_max, wave=3, n=100):
    """ Create list of point which looks like a circular sine wall

    Args:
        R   (float) : Big circle's radius
        r   (float) : Wave's radius (about big circle's circumference)
        alpha_min (float)   : Starting angle of big circle
        alpha_max   (float) : Ending angle of big circle
        wave        (float) : Wave's phase goes from -wave * pi to wave * pi
    """
    # Discretized angle for big circle
    alphas = np.linspace(alpha_min, alpha_max, n)
    # Discreteize angle for the wave
    beta = np.linspace(- wave * np.pi, wave * np.pi, n)
    # Construct the points
    xs = R * np.cos(alphas) + r * np.cos(beta)
    ys = R * np.sin(alphas)
    zs = np.ones(len(alphas)) * z  # Z-height of the origin
    points = np.array([xs, ys, zs]).T
    return points


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Master of the puppets node')
    parser.add_argument('-d', '--debug', action='store_true',
                        help='If set, will show additional debugging messages')
    parser.add_argument('-p', '--printid', type=int, help='[1] for modulating wall print, '
                                              '[2] for hex wall, '
                                              '[3] for simple circle')
    # Init node
    args = parser.parse_args(rospy.myargv()[1:])
    node_name = 'printing_demo'
    log_level = rospy.DEBUG if args.debug else rospy.INFO
    rospy.init_node(node_name, log_level=log_level)
    rospy.loginfo('Starting [%s] node' % node_name)
    demo = PrintingDemo()
    demo.send_concrete_print(args.printid)
    rospy.loginfo("Planning finished, now printing can be executed by running"
                  "StartPrintingAction node.")