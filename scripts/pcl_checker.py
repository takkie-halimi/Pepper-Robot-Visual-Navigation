from math import atan2, sqrt
import rospy
from sensor_msgs.point_cloud2 import read_points
from sensor_msgs.msg import LaserScan, PointCloud2
from message_filters import TimeSynchronizer, Subscriber


class PCLChecker(object):
    def __init__(self):
        rospy.init_node('pcl_checker', anonymous=True, disable_signals=True)
        # Original laser scan
        lleft_sub = Subscriber('/pepper/scan_left', LaserScan)
        lfront_sub = Subscriber('/pepper/scan_front', LaserScan)
        lright_sub = Subscriber('/pepper/scan_right', LaserScan)

        # Original point cloud
        pcleft_sub = Subscriber('/pepper/pc_left', PointCloud2)
        pcfront_sub = Subscriber('/pepper/pc_front', PointCloud2)
        pcright_sub = Subscriber('/pepper/pc_right', PointCloud2)

        # Translated point cloud
        pcall_sub = Subscriber('/cloud_in', PointCloud2)

        self.ts = TimeSynchronizer(
            [lleft_sub, lfront_sub, lright_sub,
             pcleft_sub, pcfront_sub, pcright_sub, pcall_sub], 10)

        self.ts.registerCallback(self.check_values)

    def get_dist(self, x0, y0, x1=0, y1=0):
        return sqrt((x1 - x0)**2 + (y1 - y0)**2)

    def check_values(self, olleft, olfront, olright,
                     opc_right, opc_left, opc_front, allpc):
        # Front
        olfront_angles = []
        opc_dist = []
        opc_angles = []

        # Left
        olleft_angles = []
        opcleft_dist = []
        opcleft_angles = []

        # Right
        olright_angles = []
        opcright_dist = []
        opcright_angles = []

        # Combined
        alllaser_dist = []
        alllaser_angles = []
        allpc_dist = []
        allpc_angles = []

        # Right
        for i in range(len(olright.ranges)):
            alllaser_dist.append(olright.ranges[i])
            alllaser_angles.append(olright.angle_min + i *
                                   olright.angle_increment - 1.757)
            olright_angles.append(olright.angle_min + i *
                                  olright.angle_increment - 1.757)

        for p in read_points(opc_right, skip_nans=True):
            distance = self.get_dist(p[0], p[1])
            opcright_dist.append(distance)
            opcright_angles.append(atan2(p[1], p[0]) - 1.757)

        # Front
        for i in range(len(olfront.ranges)):
            alllaser_dist.append(olfront.ranges[i])
            alllaser_angles.append(olfront.angle_min +
                                   i * olfront.angle_increment)
            olfront_angles.append(olfront.angle_min +
                                  i * olfront.angle_increment)

        for p in read_points(opc_front, skip_nans=True):
            opc_dist.append(self.get_dist(p[0], p[1]))
            opc_angles.append(atan2(p[1], p[0]))

        # Left
        for i in range(len(olleft.ranges)):
            alllaser_dist.append(olleft.ranges[i])
            alllaser_angles.append(olleft.angle_min + i *
                                   olleft.angle_increment + 1.757)
            olleft_angles.append(olleft.angle_min + i *
                                 olleft.angle_increment + 1.757)

        for p in read_points(opc_left, skip_nans=True):
            distance = self.get_dist(p[0], p[1])
            opcleft_dist.append(distance)
            opcleft_angles.append(atan2(p[1], p[0]) + 1.757)

        # All combined
        for p in read_points(allpc, skip_nans=True):
            allpc_dist.append(self.get_dist(p[0], p[1]))
            allpc_angles.append(atan2(p[1], p[0]))

        rospy.loginfo("===============================================================")
        rospy.loginfo("original right laser")
        rospy.loginfo(olright.ranges)
        rospy.loginfo(olright_angles)
        rospy.loginfo("===============================================================")
        rospy.loginfo("original front laser")
        rospy.loginfo(olfront.ranges)
        rospy.loginfo(olfront_angles)
        rospy.loginfo("===============================================================")
        rospy.loginfo("original left laser")
        rospy.loginfo(olleft.ranges)
        rospy.loginfo(olleft_angles)
        rospy.loginfo("===============================================================")
        rospy.loginfo("all combined laser")
        rospy.loginfo(alllaser_dist)
        rospy.loginfo(alllaser_angles)
        rospy.loginfo("==============================================================")
        rospy.loginfo("original right point cloud")
        rospy.loginfo(opcright_dist)
        rospy.loginfo(opcright_angles)
        rospy.loginfo("===============================================================")
        rospy.loginfo("original front point cloud")
        rospy.loginfo(opc_dist)
        rospy.loginfo(opc_angles)
        rospy.loginfo("===============================================================")
        rospy.loginfo("original left point cloud")
        rospy.loginfo(opcleft_dist)
        rospy.loginfo(opcleft_angles)
        rospy.loginfo("===============================================================")
        rospy.loginfo("all combined point cloud")
        rospy.loginfo(allpc_dist)
        rospy.loginfo(allpc_angles)
        rospy.loginfo("===============================================================")


if __name__ == "__main__":
    PCLC = PCLChecker()
    rospy.spin()
