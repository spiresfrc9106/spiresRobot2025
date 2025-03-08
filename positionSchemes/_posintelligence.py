
from wpimath.geometry import Pose2d
from utils.fieldTagLayout import FieldTagLayout

class PickupIntelligence:
    def __init__(self, driveTrain):
        self.pickupTags = [1, 2, 12, 13]  # red first
        self.field = FieldTagLayout()
        self.base = driveTrain
        self.poseEst = self.base.poseEst
        self.curPoseEst = self.poseEst.getCurEstPose()

    def decidePickupPose(self):
        self.bestTag = self.field.lookup(self.decidePickupTag())

        return self.bestTag

    def decidePickupTag(self):
        tagLocations = {}
        for tag in self.pickupTags:
            tagLocations[tag] = DriverAssist().calculateDistance(self.curPoseEst, self.field.lookup(tag).toPose2d())
        best_tag = min(tagLocations, key=tagLocations.get)
        return best_tag

class PlacementIntelligence():
    def __init__(self):
        pass


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