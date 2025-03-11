import math
from wpimath.geometry import Pose2d
from utils.fieldTagLayout import FieldTagLayout
from utils.units import deg2Rad
from utils.signalLogging import addLog

class PickupIntelligence:
    def __init__(self, driveTrain):
        self.pickupTags = [1, 2, 12, 13]  # red first
        self.field = FieldTagLayout()
        self.base = driveTrain
        self.poseEst = self.base.poseEst
        self.curPoseEst = self.poseEst.getCurEstPose()
        # all in meters
        self.botLenX = 0.8382
        self.botLenY = 0.6604
        self.indivBumperWidth = 0.08573

    def decidePickupPose(self):
        bestTagLocation = self.field.lookup(self.decidePickupTag()).toPose2d()
        front_half_width = self.botLenX/2
        edge_to_center_d = front_half_width + self.indivBumperWidth
        safe_fudge_factor = 1.01
        fb_offset = edge_to_center_d*safe_fudge_factor
        newTargetPose = self.adjustLocationRobotRelative(bestTagLocation, fb_offset, 0)
        return newTargetPose

    def decidePickupTag(self):
        tagLocations = {}
        for tag in self.pickupTags:
            tagLocations[tag] = DriverAssist().calculateDistance(self.curPoseEst, self.field.lookup(tag).toPose2d())
        best_tag = min(tagLocations, key=tagLocations.get)
        return best_tag

    def adjustLocationRobotRelative(self, og_pose: Pose2d, fb_shift, lr_shift):
        # fun very important math with yav dawg!
        # fb + is towards the april tag
        # lr + is to the right of the april tag
        x = YPose(og_pose).x
        y = YPose(og_pose).y
        t = YPose(og_pose).t
        ang_rad = t
        # first we'll do f/b shift.
        h = abs(fb_shift)
        if h == 0:
            sign = 0
        else:
            sign = fb_shift/h
        t = (t + (2*math.pi)) % (2*math.pi)
        if deg2Rad(90) < t < deg2Rad(180) or deg2Rad(270) < t < deg2Rad(359.9):
            sign = sign * (-1)
        x_shift = sign * h * math.cos(deg2Rad(90)-ang_rad)
        y_shift = sign * h * math.sin(deg2Rad(90)-ang_rad)
        x += x_shift
        y += y_shift
        # next is the l/r shift
        h = abs(lr_shift)
        if h == 0:
            sign = 0
        else:
            sign = lr_shift/h
        if deg2Rad(90) < t < deg2Rad(180) or deg2Rad(270) < t < deg2Rad(359.9):
            sign = sign * (-1)
        x_shift = -1 * sign * h * math.cos(ang_rad)
        y_shift = sign * h * math.sin(ang_rad)
        x += x_shift
        y += y_shift
        return Pose2d(x, y, t)







class PlacementIntelligence():
    def __init__(self, driveTrain):
        self.placementTags = [6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22]  # red first
        self.field = FieldTagLayout()
        self.base = driveTrain
        self.poseEst = self.base.poseEst
        self.curPoseEst = self.poseEst.getCurEstPose()
        # all in meters
        self.botLenX = 0.8382
        self.botLenY = 0.6604
        self.indivBumperWidth = 0.08573
        self.ourTag = 0

        addLog("yvn_current_placeL4_tag", lambda: self.ourTag, "")

    def decidePlacementPose(self):
        if self.ourTag == 0:
            self.ourTag = self.decidePlacementTag()
        bestTagLocation = self.field.lookup(self.ourTag).toPose2d()
        front_half_width = self.botLenX/2
        edge_to_center_d = front_half_width + self.indivBumperWidth
        safe_fudge_factor = 1.01
        fb_offset = edge_to_center_d*safe_fudge_factor
        # THIS SENDS THE ROBOT TO THE LOCATION OF THE APRIL TAG, NOT WITH ANY OFFSETS!!!
        newTargetPose = self.adjustLocationRobotRelative(bestTagLocation, 0, 0)
        pos = YPose(newTargetPose)
        bestTagLocation = Pose2d(pos.x, pos.y, ((math.pi+pos.t) % (2*math.pi)))
        return bestTagLocation

    def decidePlacementTag(self):
        tagLocations = {}
        for tag in self.placementTags:
            tagLocations[tag] = DriverAssist().calculateDistance(self.curPoseEst, self.field.lookup(tag).toPose2d())
        best_tag = min(tagLocations, key=tagLocations.get)
        return best_tag

    def adjustLocationRobotRelative(self, og_pose: Pose2d, fb_shift, lr_shift):
        # fun very important math with yav dawg!
        # fb + is towards the april tag
        # lr + is to the right of the april tag
        x = YPose(og_pose).x
        y = YPose(og_pose).y
        t = YPose(og_pose).t
        ang_rad = t
        # first we'll do f/b shift.
        h = abs(fb_shift)
        if h == 0:
            sign = 0
        else:
            sign = fb_shift/h
        t = (t + (2*math.pi)) % (2*math.pi)
        if deg2Rad(90) < t < deg2Rad(180) or deg2Rad(270) < t < deg2Rad(359.9):
            sign = sign * (-1)
        x_shift = sign * h * math.cos(deg2Rad(90)-ang_rad)
        y_shift = sign * h * math.sin(deg2Rad(90)-ang_rad)
        x += x_shift
        y += y_shift
        # next is the l/r shift
        h = abs(lr_shift)
        if h == 0:
            sign = 0
        else:
            sign = lr_shift/h
        if deg2Rad(90) < t < deg2Rad(180) or deg2Rad(270) < t < deg2Rad(359.9):
            sign = sign * (-1)
        x_shift = -1 * sign * h * math.cos(ang_rad)
        y_shift = sign * h * math.sin(ang_rad)
        x += x_shift
        y += y_shift
        return Pose2d(x, y, t)















class DriverAssist:
    def __init__(self):
        pass

    def calculateDistance(self, pose_one: Pose2d, pose_two: Pose2d):
        yav_one = YPose(pose_one)
        one_x = yav_one.x
        one_y = yav_one.y
        yav_two = YPose(pose_two)
        two_x = yav_two.x
        two_y = yav_two.y
        distance = pow(pow(one_x-two_x, 2) + pow(one_y-two_y, 2), 0.5)
        return distance


class YPose():
    def __init__(self, position: Pose2d):
        self.x = position.X()
        self.y = position.Y()
        self.t = position.rotation().radians()