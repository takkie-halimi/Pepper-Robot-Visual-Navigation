#!/usr/bin/env python
from math import atan2, cos, radians, sin, sqrt, degrees, isnan

import rospy

from tf import TransformListener
# from laser_geometry import LaserProjection
from sensor_msgs.msg import LaserScan, PointCloud2
from message_filters import TimeSynchronizer, Subscriber
from sensor_msgs.point_cloud2 import read_points, create_cloud_xyz32
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Vector3, Quaternion
from ddynamic_reconfigure_python.ddynamic_reconfigure import DDynamicReconfigure as DDR

import sensor_msgs.point_cloud2 as pc2

import numpy as np
"""
Author: Sammy Pfeiffer <Sammy.Pfeiffer at student.uts.edu.au>
With LaserProject implementation borrowed from the laser_geometry package.
"""


class LaserProjection:
    """
    A class to Project Laser Scan
    This calls will project laser scans into point clouds. It caches
    unit vectors between runs (provided the angular resolution of
    your scanner is not changing) to avoid excess computation.
    By default all range values less thatn the scanner min_range,
    greater than the scanner max_range are removed from the generated
    point cloud, as these are assumed to be invalid.
    If it is important to preserve a mapping between the index of
    range values and points in the cloud, the recommended approach is to
    pre-filter your laser scan message to meet the requirement that all
    ranges are between min and max_range.
    The generate PointClouds have a number of channels which can be enabled
    through the use of ChannelOption.
    - ChannelOption.INTENSITY - Create a channel named "intensities" with the
    intensity of the return for each point.
    - ChannelOption.INDEX     - Create a channel named "index" containing the
    index from the original array for each point.
    - ChannelOption.DISTANCE  - Create a channel named "distance" containing
    the distance from the laser to each point.
    - ChannelOption.TIMESTAMP - Create a channel named "stamps" containing the
    specific timestamp at which each point was measured.
    """

    LASER_SCAN_INVALID = -1.0
    LASER_SCAN_MIN_RANGE = -2.0
    LASER_SCAN_MAX_RANGE = -3.0

    class ChannelOption:
        NONE = 0x00  # Enable no channels
        INTENSITY = 0x01  # Enable "intensities" channel
        INDEX = 0x02  # Enable "index"       channel
        DISTANCE = 0x04  # Enable "distances"   channel
        TIMESTAMP = 0x08  # Enable "stamps"      channel
        VIEWPOINT = 0x10  # Enable "viewpoint"   channel
        DEFAULT = (INTENSITY | INDEX)

    def __init__(self):
        self.__angle_min = 0.0
        self.__angle_max = 0.0

        self.__cos_sin_map = np.array([[]])

    def projectLaser(self, scan_in,
                     range_cutoff=-1.0, channel_options=ChannelOption.DEFAULT):
        """
        Project a sensor_msgs::LaserScan into a sensor_msgs::PointCloud2.
        Project a single laser scan from a linear array into a 3D
        point cloud. The generated cloud will be in the same frame
        as the original laser scan.
        Keyword arguments:
        scan_in -- The input laser scan.
        range_cutoff -- An additional range cutoff which can be
            applied which is more limiting than max_range in the scan
            (default -1.0).
        channel_options -- An OR'd set of channels to include.
        """
        return self.__projectLaser(scan_in, range_cutoff, channel_options)

    def __projectLaser(self, scan_in, range_cutoff, channel_options):
        N = len(scan_in.ranges)

        ranges = np.array(scan_in.ranges)

        if (self.__cos_sin_map.shape[1] != N or
            self.__angle_min != scan_in.angle_min or
                self.__angle_max != scan_in.angle_max):
            rospy.logdebug("No precomputed map given. Computing one.")

            self.__angle_min = scan_in.angle_min
            self.__angle_max = scan_in.angle_max

            angles = scan_in.angle_min + np.arange(N) * scan_in.angle_increment
            self.__cos_sin_map = np.array([np.cos(angles), np.sin(angles)])

        output = ranges * self.__cos_sin_map

        # Set the output cloud accordingly
        cloud_out = PointCloud2()

        fields = [pc2.PointField() for _ in range(3)]

        fields[0].name = "x"
        fields[0].offset = 0
        fields[0].datatype = pc2.PointField.FLOAT32
        fields[0].count = 1

        fields[1].name = "y"
        fields[1].offset = 4
        fields[1].datatype = pc2.PointField.FLOAT32
        fields[1].count = 1

        fields[2].name = "z"
        fields[2].offset = 8
        fields[2].datatype = pc2.PointField.FLOAT32
        fields[2].count = 1

        idx_intensity = idx_index = idx_distance = idx_timestamp = -1
        idx_vpx = idx_vpy = idx_vpz = -1

        offset = 12

        if (channel_options & self.ChannelOption.INTENSITY and
                len(scan_in.intensities) > 0):
            field_size = len(fields)
            fields.append(pc2.PointField())
            fields[field_size].name = "intensity"
            fields[field_size].datatype = pc2.PointField.FLOAT32
            fields[field_size].offset = offset
            fields[field_size].count = 1
            offset += 4
            idx_intensity = field_size

        if channel_options & self.ChannelOption.INDEX:
            field_size = len(fields)
            fields.append(pc2.PointField())
            fields[field_size].name = "index"
            fields[field_size].datatype = pc2.PointField.INT32
            fields[field_size].offset = offset
            fields[field_size].count = 1
            offset += 4
            idx_index = field_size

        if channel_options & self.ChannelOption.DISTANCE:
            field_size = len(fields)
            fields.append(pc2.PointField())
            fields[field_size].name = "distances"
            fields[field_size].datatype = pc2.PointField.FLOAT32
            fields[field_size].offset = offset
            fields[field_size].count = 1
            offset += 4
            idx_distance = field_size

        if channel_options & self.ChannelOption.TIMESTAMP:
            field_size = len(fields)
            fields.append(pc2.PointField())
            fields[field_size].name = "stamps"
            fields[field_size].datatype = pc2.PointField.FLOAT32
            fields[field_size].offset = offset
            fields[field_size].count = 1
            offset += 4
            idx_timestamp = field_size

        if channel_options & self.ChannelOption.VIEWPOINT:
            field_size = len(fields)
            fields.extend([pc2.PointField() for _ in range(3)])
            fields[field_size].name = "vp_x"
            fields[field_size].datatype = pc2.PointField.FLOAT32
            fields[field_size].offset = offset
            fields[field_size].count = 1
            offset += 4
            idx_vpx = field_size
            field_size += 1

            fields[field_size].name = "vp_y"
            fields[field_size].datatype = pc2.PointField.FLOAT32
            fields[field_size].offset = offset
            fields[field_size].count = 1
            offset += 4
            idx_vpy = field_size
            field_size += 1

            fields[field_size].name = "vp_z"
            fields[field_size].datatype = pc2.PointField.FLOAT32
            fields[field_size].offset = offset
            fields[field_size].count = 1
            offset += 4
            idx_vpz = field_size

        if range_cutoff < 0:
            range_cutoff = scan_in.range_max
        else:
            range_cutoff = min(range_cutoff, scan_in.range_max)

        points = []
        big_str = "\n"
        for i in range(N):
            ri = scan_in.ranges[i]
            # if ri < range_cutoff and ri >= scan_in.range_min:
            if True:
                point = output[:, i].tolist()
                point.append(0)
                p = point
                angle_increment = scan_in.angle_increment
                min_angle = scan_in.angle_min
                dist = ri
                idx = i

                if idx_intensity != -1:
                    point.append(scan_in.intensities[i])

                if idx_index != -1:
                    point.append(i)

                if idx_distance != -1:
                    point.append(scan_in.ranges[i])

                if idx_timestamp != -1:
                    point.append(i * scan_in.time_increment)

                if idx_vpx != -1 and idx_vpy != -1 and idx_vpz != -1:
                    point.extend([0 for _ in range(3)])

                points.append(point)

                big_str += "   " + str(idx).zfill(2) + ": x: " + str(round(p[0], 2)) + ", y: " + str(round(
                    p[1], 2)) + ", z: " + str(round(p[2], 2)) + " = " + str(round(dist, 2)) + "m (at " + str(round(degrees(idx * angle_increment + min_angle), 2)) + "deg)\n"

        rospy.loginfo("Projected cloud:")
        rospy.loginfo(big_str)
        cloud_out = pc2.create_cloud(scan_in.header, fields, points)

        return cloud_out


class LaserPublisher(object):
    def __init__(self):
        if not rospy.core.is_initialized():
            rospy.init_node('laser_test')
            rospy.loginfo("Initialised rospy node: laser_test")

        self.tl = TransformListener()
        self.lp = LaserProjection()

        # Publishers
        self.all_laser_pub = rospy.Publisher(
            '/pepper/laser_2', LaserScan, queue_size=1)
        self.pc_pub = rospy.Publisher('/cloud', PointCloud2, queue_size=1)
        self.pcl_pub = rospy.Publisher('/cloudl', PointCloud2, queue_size=1)
        self.pcr_pub = rospy.Publisher('/cloudr', PointCloud2, queue_size=1)
        self.pc_redone_pub = rospy.Publisher('/cloud_redone',
                                             PointCloud2, queue_size=1)
        self.pc_rereprojected_pub = rospy.Publisher('/cloud_rereprojected',
                                                    PointCloud2,
                                                    queue_size=1)

        # Subscribers
        left_sub = Subscriber('/pepper/scan_left', LaserScan)
        front_sub = Subscriber('/pepper/scan_front', LaserScan)
        right_sub = Subscriber('/pepper/scan_right', LaserScan)

        self.ts = TimeSynchronizer([left_sub, front_sub, right_sub],
                                   10)
        rospy.loginfo("Finished intialising")
        self.ddr = DDR('increment')
        default_increment = radians(120.0 * 2.0) / 61.0
        self.ddr.add_variable('angle_increment', '', default_increment,
                              0.05, 0.08)
        # 130.665
        self.ddr.add_variable('half_max_angle', '', 120., 115., 145.0)
        self.ddr.start(self.dyn_rec_callback)
        self.ts.registerCallback(self.scan_cb)
        rospy.loginfo("Ready to go.")

    def add_variables_to_self(self):
        var_names = self.ddr.get_variable_names()
        for var_name in var_names:
            self.__setattr__(var_name, None)

    def dyn_rec_callback(self, config, level):
        rospy.loginfo("Received reconf call: " + str(config))
        # Update all variables
        var_names = self.ddr.get_variable_names()
        for var_name in var_names:
            self.__dict__[var_name] = config[var_name]
        return config

    def scan_cb(self, left, front, right):
        rospy.loginfo("We got scan_cb")
        translated_points = []
        try:
            pc_left = self.lp.projectLaser(left, channel_options=0x00)
            pc_front = self.lp.projectLaser(front, channel_options=0x00)
            pc_right = self.lp.projectLaser(right, channel_options=0x00)
        except Exception as e:
            rospy.logerr("Failed to transform laser scan because: " + str(e))

        pc_left.header.stamp = rospy.Time.now()
        pc_left.header.frame_id = 'SurroundingLeftLaser_frame'
        self.pcl_pub.publish(pc_left)
        self.pcr_pub.publish(pc_right)

        transform_right_to_front = self.tl.lookupTransform(
            'base_footprint', 'SurroundingRightLaser_frame', rospy.Time.now())
        rospy.logdebug("Transform Right to Front:")
        rospy.logdebug(transform_right_to_front)
        ts = TransformStamped()
        ts.transform.translation = Vector3(*transform_right_to_front[0])
        ts.transform.rotation = Quaternion(*transform_right_to_front[1])
        ts.header.stamp = rospy.Time.now()
        transformed_cloud = do_transform_cloud(pc_right, ts)
        # right point cloud translation
        for p in read_points(transformed_cloud,
                             field_names=('x', 'y', 'z'),
                             skip_nans=False):
            translated_points.append(p)

        for i in range(8):
            translated_points.append(
                (float('nan'), float('nan'), float('nan')))

        transform_front_to_front = self.tl.lookupTransform(
            'base_footprint', 'SurroundingFrontLaser_frame', rospy.Time.now())
        rospy.logdebug("Transform Front to Front:")
        rospy.logdebug(transform_front_to_front)
        ts = TransformStamped()
        ts.transform.translation = Vector3(*transform_front_to_front[0])
        ts.transform.rotation = Quaternion(*transform_front_to_front[1])
        ts.header.stamp = rospy.Time.now()
        transformed_cloud_f = do_transform_cloud(pc_front, ts)

        # front point cloud
        for p in read_points(transformed_cloud_f,
                             field_names=('x', 'y', 'z'),
                             skip_nans=False):
            translated_points.append(p)

        transform_left_to_front = self.tl.lookupTransform(
            'base_footprint', 'SurroundingLeftLaser_frame', rospy.Time.now())
        rospy.logdebug("Transform Left to Front:")
        rospy.logdebug(transform_left_to_front)
        ts = TransformStamped()
        ts.transform.translation = Vector3(*transform_left_to_front[0])
        ts.transform.rotation = Quaternion(*transform_left_to_front[1])
        ts.header.stamp = rospy.Time.now()
        from copy import deepcopy
        transformed_cloud_l = do_transform_cloud(deepcopy(pc_left), ts)

        for i in range(8):
            translated_points.append(
                (float('nan'), float('nan'), float('nan')))

        # left pc translation
        for p in read_points(transformed_cloud_l,
                             field_names=('x', 'y', 'z'),
                             skip_nans=False):
            translated_points.append(p)

        # Create a point cloud from the combined points wrt the front
        # laser frame
        pc_front.header.frame_id = 'base_footprint'
        point_cloud = create_cloud_xyz32(pc_front.header, translated_points)
        self.pc_pub.publish(point_cloud)
        rospy.logdebug("pointcloud all together len: " +
                       str(point_cloud.width))

        # # double check we have the same thing
        # compare_str = "\n"
        # for idx, (tp, pcp) in enumerate(zip(translated_points, read_points(point_cloud))):
        #     compare_str += str(idx).zfill(2) + ":\n"
        #     compare_str += "  tp : " + str(tp)
        #     compare_str += "\n  pcp: " + str(pcp) + "\n"
        # rospy.loginfo(compare_str)
        # # OK we know they are the same
        # # translated_points and point_cloud contain virtually the same data

        # Convert combined point cloud into LaserScan
        all_laser_msg = LaserScan()
        laser_ranges, angle_min, angle_max, angle_increment = self.pc_to_laser(
            point_cloud)
        all_laser_msg.header.frame_id = 'base_footprint'
        all_laser_msg.header.stamp = rospy.Time.now()
        all_laser_msg.ranges = laser_ranges
        all_laser_msg.angle_min = angle_min
        all_laser_msg.angle_max = angle_max
        all_laser_msg.angle_increment = angle_increment
        all_laser_msg.range_min = 0.1
        all_laser_msg.range_max = 7.0
        all_laser_msg.intensities = []
        self.all_laser_pub.publish(all_laser_msg)

        rospy.logdebug("all_laser_msg len: " + str(len(all_laser_msg.ranges)))
        pc_redone = self.lp.projectLaser(all_laser_msg, channel_options=0x00)
        rospy.logdebug("all_laser pc_redone len: " + str(pc_redone.width))
        self.pc_redone_pub.publish(pc_redone)

        # compare what came in and what came out
        rospy.logdebug("point_cloud frame_id, pc_redone frame_id:")
        rospy.logdebug((point_cloud.header.frame_id,
                        pc_redone.header.frame_id))
        rospy.logdebug("point_cloud is correct, pc_redone is incorrect")
        compare_str = "\n"
        for idx, (point_in, point_out) in enumerate(zip(read_points(point_cloud), read_points(pc_redone))):
            point_out = [point_out[0], point_out[1], 0.0]
            point_in = [point_in[0], point_in[1], 0.0]
            compare_str += str(idx).zfill(2) + ":\n"
            compare_str += "  in : " + str(point_in)
            compare_str += "\n  out: " + str(point_out) + "\n"
            dist = np.linalg.norm(np.array(point_out) - np.array(point_in))
            compare_str += " dist: " + str(dist) + "\n"
            # angle
            angle1 = atan2(point_in[1], point_in[0])
            angle2 = atan2(point_out[1], point_out[0])
            angle_dif = angle2 - angle1
            compare_str += " angle dif: " + str(angle_dif) + "\n"

        rospy.logdebug(compare_str)

    def pc_to_laser(self, cloud):
        laser_points = []
        points_rereprojected = []
        multiply_num_rays = 8
        num_rays = 61 * multiply_num_rays
        laser_points2 = [float('nan')] * num_rays
        min_angle = -radians(self.half_max_angle)
        max_angle = radians(self.half_max_angle)
        # angle_increment = self.angle_increment
        angle_increment = (radians(self.half_max_angle)
                           * 2.0) / float(num_rays)
        big_str = "\n"
        for idx, p in enumerate(read_points(cloud, skip_nans=False)):
            #dist = self.get_dist(p[0], p[1])
            p = [p[0], p[1], 0.0]
            dist = np.linalg.norm(np.array((0., 0., 0.)) - np.array(p))
            # dist1 = self.get_dist(p[0], p[1])
            big_str += "   " + str(idx).zfill(2) + ": x: " + str(round(p[0], 2)) + ", y: " + str(round(
                p[1], 2)) + ", z: " + str(round(p[2], 2)) + " = " + str(round(dist, 2)) + "m (at " + str(round(degrees(idx * angle_increment + min_angle), 2)) + "deg)\n"

            laser_points.append(dist)
            # coords from dist
            x = dist * cos(idx * angle_increment + min_angle)
            y = dist * sin(idx * angle_increment + min_angle)
            print(" [ px, py, are the correct points ] ")
            print("dist, px, py: " + str(dist) +
                  " " + str(p[0])) + " " + str(p[1])
            print("dist, x, y:   " + str(dist) + " " + str(x) + " " + str(y))

            dist_from_rereproj = self.get_dist(x, y)
            print("dist rereproj: " + str(dist_from_rereproj))
            # print("dist1       : " + str(dist1))

            # what if a make a pointcloud based in the cos sin version
            points_rereprojected.append((x, y, 0.0))

            # angle from point
            angle = atan2(p[1], p[0])
            # angle2 = atan2(y, x)
            expected_angle = idx * self.angle_increment + min_angle
            if not isnan(angle):
                tmp_angle = angle - min_angle
                print("tmp_angle: " + str(degrees(tmp_angle))) + " deg"
                print("angle_increment: " + str(degrees(angle_increment)))
                closest_index = int(tmp_angle / angle_increment)
                print("closest index: " + str(closest_index))
                if closest_index >= len(laser_points2):
                    laser_points2[-1] = dist
                elif closest_index < 0:
                    laser_points2[0] = dist
                else:
                    laser_points2[closest_index] = dist
            else:
                print("nan, not adding anything to scan")

            # laser_points[]
            print("Angle from p : " + str(round(degrees(angle), 2)))
            # print("Angle from xy: " + str(round(degrees(angle2), 2)))
            print("Expected angle: " + str(round(degrees(expected_angle), 2)))

        rospy.logdebug("Lasered cloud")
        rospy.logdebug(big_str)

        laser_points = laser_points2
        print("Len of laser points after new technique: " + str(len(laser_points)))

        rereprojected_pc = PointCloud2()
        rereprojected_pc.header.frame_id = 'base_footprint'
        rereprojected_pc.header.stamp = rospy.Time.now()
        point_cloud_rere = create_cloud_xyz32(
            rereprojected_pc.header, points_rereprojected)
        self.pc_rereprojected_pub.publish(point_cloud_rere)

        return laser_points, min_angle, max_angle, angle_increment

    def get_dist(self, x0, y0, x1=0.0, y1=0.0):
        return sqrt((x1 - x0)**2 + (y1 - y0)**2)


if __name__ == "__main__":
    lp = LaserPublisher()
    rospy.spin()
