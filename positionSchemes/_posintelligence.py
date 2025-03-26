import math
from wpimath.geometry import Pose2d
from wpilib import Timer
from utils.fieldTagLayout import FieldTagLayout
from utils.units import deg2Rad, rad2Deg, in2m
from utils.signalLogging import addLog
from drivetrain.DrivetrainDependentConstants import drivetrainDepConstants


class PlacementIntelligence():
    def __init__(self, driveTrain):
        self.placementTags = [6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22]  # red first
        self.field = FieldTagLayout()
        self.base = driveTrain
        self.poseEst = self.base.poseEst
        self.curPoseEst = self.poseEst.getCurEstPose()
        # all in meters
        self.botLenX = in2m(drivetrainDepConstants['LENGTH']) # 0.8382
        self.botLenY = in2m(drivetrainDepConstants['WIDTH']) # 0.6604
        self.indivBumperWidth = in2m(3.375) # 0.08575
        self.currentTarget = 0
        self.shiftToNode_m = in2m(6.50)  # the distance from center to the right side of note of the center of robot's f

        addLog("yvn_current_placeL4_tag", lambda: self.currentTarget, "")

    def decidePlacementPose(self, sideOfReef, bumperToEdge_m, manualSetTarget=0):
        if self.currentTarget == 0:  # only sets if fresh tag is needed
            if manualSetTarget > 0:
                self.currentTarget = manualSetTarget
            else:
                self.currentTarget = self.decidePlacementTag()
        bestTagLocation = self.field.lookup(self.currentTarget).toPose2d()
        front_half_width = self.botLenX / 2
        edge_to_center_d = front_half_width + self.indivBumperWidth
        safe_fudge_factor = 1.003
        fb_offset = edge_to_center_d * safe_fudge_factor
        newTargetPose = self.adjustLocationRobotRelative(bestTagLocation, fb_offset + bumperToEdge_m,
                                                         self.shiftToNode_m * sideOfReef)
        pos = YPose(newTargetPose)
        bestTagLocation = Pose2d(pos.x, pos.y, ((math.pi + pos.t) % (2 * math.pi)))
        return bestTagLocation

    def decidePlacementTag(self):
        tagLocations = {}
        for tag in self.placementTags:
            tagLocations[tag] = DriverAssist().calculateDistance(self.curPoseEst, self.field.lookup(tag).toPose2d())
        best_tag = min(tagLocations, key=tagLocations.get)
        return best_tag

    # FOR PLACEMENT
    def adjustLocationRobotRelative(self, og_pose: Pose2d, fb_shift, lr_shift):
        # fun very important math with yav dawg!
        # fb + is towards the april tag
        # lr + is to the right of the april tag
        x = YPose(og_pose).x
        y = YPose(og_pose).y
        t_clean = YPose(og_pose).t
        t = t_clean  # if that's what the man says #((2*math.pi) - YPose(og_pose).t)

        ang_rad = ((t + (2 * math.pi) * 2) % (2 * math.pi))
        print(f"adjustLocationRobotRelative"
            f" time={Timer.getFPGATimestamp():.3f}s"
            f" t={rad2Deg(t):+7.1f}deg ang_rad.degrees()={rad2Deg(ang_rad):+7.1f}deg")

        # first we'll do f/b shift.
        h = abs(fb_shift)
        if h == 0:
            sign = 0
        else:
            sign = fb_shift / h
        h = h * sign
        t = ang_rad
        x_shift = 0
        y_shift = 0
        theta = 90 - 30
        # right: math.isclose(deg2Rad(180), t, abs_tol=0.05):
        if self.currentTarget == 10 or self.currentTarget == 18:
            x_shift = -1 * h
            y_shift = 0
        # left: math.isclose(deg2Rad(0), t, abs_tol=0.05):
        if self.currentTarget == 7 or self.currentTarget == 21:
            x_shift = h
            y_shift = 0
        # up-right:
        if self.currentTarget == 8 or self.currentTarget == 20:
            x_shift = 1 * h * math.cos(deg2Rad(theta))
            y_shift = 1 * h * math.sin(deg2Rad(theta))
        # up-left:
        if self.currentTarget == 9 or self.currentTarget == 19:
            x_shift = -1 * h * math.cos(deg2Rad(theta))
            y_shift = 1 * h * math.sin(deg2Rad(theta))
        # down-left:
        if self.currentTarget == 11 or self.currentTarget == 17:
            x_shift = -1 * h * math.cos(deg2Rad(theta))
            y_shift = -1 * h * math.sin(deg2Rad(theta))
        # down-right:
        if self.currentTarget == 6 or self.currentTarget == 22:
            x_shift = 1 * h * math.cos(deg2Rad(theta))
            y_shift = -1 * h * math.sin(deg2Rad(theta))
        x += x_shift
        y += y_shift
        # next is the l/r shift
        h = abs(lr_shift)
        if h == 0:
            sign = 0
        else:
            sign = lr_shift / h
        h = h * sign
        x_shift = 0
        y_shift = 0
        theta = 30
        # right: math.isclose(deg2Rad(180), t, abs_tol=0.05):
        if self.currentTarget == 10 or self.currentTarget == 18:
            x_shift = 0
            y_shift = -1 * h
        # left: math.isclose(deg2Rad(0), t, abs_tol=0.05):
        if self.currentTarget == 7 or self.currentTarget == 21:
            x_shift = 0
            y_shift = h
        # up-right:
        if self.currentTarget == 8 or self.currentTarget == 20:
            x_shift = -1 * h * math.cos(deg2Rad(theta))
            y_shift = 1 * h * math.sin(deg2Rad(theta))
        # up-left:
        if self.currentTarget == 9 or self.currentTarget == 19:
            x_shift = -1 * h * math.cos(deg2Rad(theta))
            y_shift = -1 * h * math.sin(deg2Rad(theta))
        # down-left:
        if self.currentTarget == 11 or self.currentTarget == 17:
            x_shift = 1 * h * math.cos(deg2Rad(theta))
            y_shift = -1 * h * math.sin(deg2Rad(theta))
        # down-right:
        if self.currentTarget == 6 or self.currentTarget == 22:
            x_shift = 1 * h * math.cos(deg2Rad(theta))
            y_shift = 1 * h * math.sin(deg2Rad(theta))
        x += x_shift
        y += y_shift
        return Pose2d(x, y, t_clean)


class PickupIntelligence:
    def __init__(self, driveTrain):
        self.pickupTags = [1, 2, 12, 13]  # red first
        self.field = FieldTagLayout()
        self.base = driveTrain
        self.poseEst = self.base.poseEst
        self.curPoseEst = self.poseEst.getCurEstPose()
        # all in meters
        self.botLenX = in2m(drivetrainDepConstants['LENGTH'])  # 0.8382
        self.botLenY = in2m(drivetrainDepConstants['WIDTH'])  # 0.6604
        self.indivBumperWidth = in2m(3.375)

    def decidePickupPose(self, extra_forward_m=0, manualSetTarget=0):
        targetTag = manualSetTarget
        if manualSetTarget <= 0:
            targetTag = self.decidePickupTag()
        bestTagLocation = self.field.lookup(targetTag).toPose2d()
        front_half_width = self.botLenX / 2
        edge_to_center_d = front_half_width + self.indivBumperWidth
        safe_fudge_factor = 1.01
        fb_offset = edge_to_center_d * safe_fudge_factor
        newTargetPose = self.adjustLocationRobotRelative(bestTagLocation, fb_offset+extra_forward_m, 0)
        return newTargetPose

    def decidePickupTag(self):
        tagLocations = {}
        for tag in self.pickupTags:
            tagLocations[tag] = DriverAssist().calculateDistance(self.curPoseEst, self.field.lookup(tag).toPose2d())
        best_tag = min(tagLocations, key=tagLocations.get)
        return best_tag

    def adjustLocationRobotRelative(self, og_pose: Pose2d, fb_shift, lr_shift):
        # fun very important math with yav dawg!
        # fb + is towards? the april tag
        # lr + is to the right of the april tag
        x = YPose(og_pose).x
        y = YPose(og_pose).y
        t = YPose(og_pose).t
        ang_rad = (t + (2 * math.pi) * 2) % (2 * math.pi)
        # first we'll do f/b shift.
        h = abs(fb_shift)
        if h == 0:
            sign = 0
        else:
            sign = fb_shift / h
        t = (t + (2 * math.pi) * 2) % (2 * math.pi)
        if deg2Rad(90) < t < deg2Rad(180) or deg2Rad(270) < t < deg2Rad(359.9):
            sign = sign * (-1)
        x_shift = sign * h * math.cos(deg2Rad(90) - ang_rad)
        y_shift = sign * h * math.sin(deg2Rad(90) - ang_rad)
        x += x_shift
        y += y_shift
        # next is the l/r shift
        h = abs(lr_shift)
        if h == 0:
            sign = 0
        else:
            sign = lr_shift / h
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
        distance = pow(pow(one_x - two_x, 2) + pow(one_y - two_y, 2), 0.5)
        return distance


class YPose():
    def __init__(self, position: Pose2d):
        self.x = position.X()
        self.y = position.Y()
        self.t = position.rotation().radians()
