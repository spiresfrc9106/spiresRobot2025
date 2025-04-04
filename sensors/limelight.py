import math
from enum import Enum
import ntcore
from wpimath.geometry import Pose3d, Rotation3d, Translation3d, Rotation2d, Translation3d

class Team(Enum):
    RED = 0
    BLUE = 1
class LimelightPipeline:
    feducial = 0.0
    neural = 1.0
    retroreflective = 2.0


limelight_led_mode = {
    "pipline_default": 0,
    "force_off": 1,
    "force_blink": 2,
    "force_on": 3,
}

odometry_megatag2_max_angular_velocity_degrees_per_sec = 12

class Limelight:
    """
    A class for interfacing with the limelight camera.
    """

    def __init__(self, origin_offset: Translation3d, name: str = "limelight", mode_given=1.0):
        """

        :param origin_offset: The offset of the limelight from the robot's origin in meters

        :param name: The name of the limelight network table. This is used to differentiate between multiple limelights.
        If you have multiple limelights, you must give them different names in order for their values to be read
        correctly. If you only have one limelight, you can leave this as the default value.
        """

        self.nt = ntcore.NetworkTableInstance.getDefault()
        self.name = name
        self.table: ntcore.NetworkTable = self.nt.getTable(name)
        self.tx: float = 0
        self.ty: float = 0
        self.tv: float = 0
        self.ta: float = 0
        self.tid: float = -1.0
        self.origin_offset: Translation3d = origin_offset
        self.drive_cam = False
        self.pipeline: LimelightPipeline = LimelightPipeline.feducial
        self.t_class = None
        self.force_update = False
        self.botpose_blue: list[float] = [0,0,0,0,0,0,0,0,0,0,0]
        self.botpose_blue_l: float = 0.0
        self.botpose_red: list[float] = [0,0,0,0,0,0,0,0,0,0,0]
        self.botpose_red_l: float = 0.0
        self.botpose: list[float] = [0,0,0,0,0,0,0,0,0,0,0]
        self.botposemeta2: list[float] = [0,0,0,0,0,0,0,0,0,0,0]
        self.targetpose: Pose3d = Pose3d(Translation3d(0, 0, 0), Rotation3d(0, 0, 0))
        self.targetPoseByRobot: list[float] = [0, 0, 0, 0, 0, 0]
        self.cam_pos_moving: bool = False
        self.set_pipeline_mode(mode_given)
        #self.set_led_mode(limelight_led_mode["force_on"])
        #self.set_cam_pose(Pose3d(Translation3d(0,0,0), Rotation3d(Rotation2d(0)))) Coach mike suggestion, Yavin may be fatally wrong, if breaks it's his fault

    def init(self):
        pass

    def set_cam_pose(self, pose: Pose3d):
        """
        Sets the pose of the limelight relative to the robot's origin.

        :param pose: The pose of the limelight relative to the robot's origin
        """

        self.origin_offset = pose

        campose = [
            self.origin_offset.Y(),
            self.origin_offset.X(),
            self.origin_offset.Z(),
            math.degrees(self.origin_offset.rotation().X()),
            math.degrees(self.origin_offset.rotation().Y()),
            math.degrees(self.origin_offset.rotation().Z()),
        ]
        #self.table.putNumberArray("camerapose_robotspace_set", campose)
        
    def get_cam_pose(self):
        """
        Gets the pose of the limelight relative to the robot's origin.

        :return Pose3d: The pose of the limelight relative to the robot's origin
        """

        return self.origin_offset

    def set_cam_elevator_height(self, elevator_height_meters: float):
        """
        Sets the height of the limelight relative to the floor.

        :param height: The height of the elevator in meters
        """
        x = self.origin_offset.X()
        y = self.origin_offset.Y()
        z = self.origin_offset.Z()

        self.set_cam_pose(
            Pose3d(
                Translation3d(x, y, elevator_height_meters + z), self.origin_offset.rotation()
            )
        )

    def enable_force_update(self):
        """
        Forces the limelight to update its values. This is useful if you want to get the limelight's values multiple
        times in one loop.

        This is disabled by default.

        Force update can also be enabled on a method-by-method basis by passing force_update=True to the method.

        When disabled, the limelight will only update its values once per loop with the update() function.
        """
        self.force_update = True

    def disable_force_update(self):
        """
        Disables the force update. When this is disabled, the limelight will only update its values once per loop.
        """

        self.force_update = False

    def set_pipeline_mode(self, mode: LimelightPipeline) -> None:
        """
        Sets the pipeline mode of the limelight will be using (Feeducial, Retroreflective, Neural, etc.)
        These modes match the numbers used in the limelight web interface.

        :param mode: The pipeline int to set the limelight to

        """

        self.table.putNumber("pipeline", mode)
        self.pipeline = mode

    def get_pipeline_mode(self) -> LimelightPipeline:
        """
        Gets the pipeline mode of the limelight will be using (Feducial, Retroreflective, Neural, etc.) as an integer.

        :return LimelightPipeline: The pipeline mode the limelight is currently using
        """

        pipeline = self.table.getNumber("getpipe", 0.0)
        if self.pipeline != pipeline:
            self.pipeline = pipeline
        return self.pipeline

    def set_led_mode(self, mode: limelight_led_mode) -> None:
        """
        Changes the LED mode of the limelight.

        :param mode: The LED mode to set the limelight to
        """

        self.table.putNumber("ledMode", mode)

    def get_led_mode(self) -> limelight_led_mode:
        return self.table.getNumber("ledMode", 0)
    
    def get_target_id(self) -> float:
        return self.tid

    def set_cam_vision(self):
        """
        Sets the limelight to use the camera for vision processing.
        """

        self.table.putNumber("camMode", 0)
        self.drive_cam = False

    def set_cam_driver(self):
        """
        Sets the limelight to use the camera for driver vision.
        """

        self.table.putNumber("camMode", 1)
        self.drive_cam = True

    def get_cam_mode(self):
        """
        Gets the camera mode of the limelight (Vision or Driver)
        :return bool: True if the limelight is in driver mode, False if it is in vision mode
        """

        mode = self.table.getNumber("camMode", 0)
        if self.drive_cam != mode:
            self.drive_cam = mode
        return self.drive_cam

    def get_neural_classId(self, force_update: bool = False):
        """
        Gets the neural classId of the limelight.
        This id number matches the numbers used in the limelight web interface.

        :return neural_classId: The neural classId the limelight is currently using
        """

        if force_update or self.force_update:
            self.update()
        if self.pipeline != LimelightPipeline.neural:
            return False
        if self.tv == 0:
            return None
        self.t_class = self.table.getString("tclass", "")
        return self.t_class

    def update(self):
        """
        Updates all values of the limelight Manually.
        For proper use, this should be called in the main event loop.
        """

        self.update_generic()
        
        self.get_pipeline_mode()
        self.get_neural_classId()
        # self.botpose_red = self.table.getEntry("botpose_wpired").getDoubleArray([0, 0, 0, 0, 0, 0])
        self.update_bot_pose()
        self.updateTargetPosition()
        
    def update_generic(self):
        '''
        Updates generic values from the limelight network table
        calling this in the main event loop will only update generic values
        '''
        self.tx = self.table.getNumber("tx", 0)
        self.ty = self.table.getNumber("ty", 0)
        self.tv = self.table.getNumber("tv", 0)
        self.ta = self.table.getNumber("ta", 0)
        self.tid = self.table.getNumber("tid", -1)

    def getTargetSize(self):
        return self.ta

    def updateTargetPosition(self):
        target_entry_name = "targetpose_robotspace"
        target_pose = self.table.getEntry(target_entry_name)
        target_formation = target_pose.getDoubleArray([0, 0, 0, 0, 0, 0])
        self.targetPoseByRobot = target_formation

    def update_bot_pose(self, megatag2:bool = False):
        '''
        Updates botpose values from the limelight network table
        calling this in the main event loop will only update botpose values
        '''
        botpose = 'botpose_wpiblue' #changed from botpose to botpose_wpiblue to get proper coordinates
        botpose_red = 'botpose_wpired'
        botpose_blue = 'botpose_wpiblue'
        if megatag2:
            botpose = 'botpose_orb'
            botpose_red = 'botpose_orb_wpired'
            botpose_blue = 'botpose_orb_wpiblue'

        botposemeta2 = 'botpose_orb_wpiblue' #changed from botpode to blue for proper numbers
        
        # self.botpose_red = self.table.getNumberArray(
        #     botpose_red, [0, 0, 0, 0, 0, 0, 0, 0, 0,0,0]
        # )
        
        botpose_red_entry = self.table.getEntry(botpose_red)
        
        self.botpose_red = botpose_red_entry.getDoubleArray(
            [0, 0, 0, 0, 0, 0, 0, 0, 0,0,0]
        )
        
        self.botpose_red_l = (botpose_red_entry.getLastChange() / 1000000.0) - (self.botpose_red[6]/1000)
        
        
        
        self.get_pipeline_mode()
        
        botpose_blue_entry = self.table.getEntry(botpose_blue)
        
        self.botpose_blue = botpose_blue_entry.getDoubleArray(
            [0, 0, 0, 0, 0, 0, 0, 0, 0,0,0]
        )
        
        self.botpose_blue_l = (botpose_blue_entry.getLastChange() / 1000000.0) - (self.botpose_blue[6]/1000)
        # self.botpose_blue = self.table.getNumberArray(
        #     botpose_blue, [0, 0, 0, 0, 0, 0, 0, 0, 0,0,0]
        # )
        # self.botpose = self.table.getEntry("botpose").getDoubleArray([0, 0, 0, 0, 0, 0])

        # See: https://docs.limelightvision.io/docs/docs-limelight/apis/complete-networktables-api
        # botpose
        # Robot transform in field-space. Translation (X,Y,Z) in meters Rotation(Roll,Pitch,Yaw) in degrees,
        # total latency (cl+tl), tag count, tag span, average tag distance from camera,
        # average tag area (percentage of image)
        #
        # or
        # botpose_orb
        # Robot transform in field-space (Megatag2). Translation (X,Y,Z) in meters Rotation(Roll,Pitch,Yaw) in degrees,
        # total latency (cl+tl), tag count, tag span, average tag distance from camera,
        # average tag area (percentage of image)

        self.botpose = self.table.getNumberArray(botpose, [0, 0, 0, 0, 0, 0, 0, 0, 0,0,0])
        self.botposemeta2 = self.table.getNumberArray(botposemeta2, [0, 0, 0, 0, 0, 0, 0, 0, 0,0,0])
        self.targetpose = self.table.getNumberArray("botpose_targetspace", [0, 0, 0, 0, 0, 0])


    def set_robot_orientation(self, yaw_degrees: float, yaw_rate_degrees_per_second: float,
                              pitch_degrees: float, pitch_rate_degrees_per_second: float,
                              roll_degrees: float, roll_rate_degrees_per_second: float):
        '''
        Sets robot gyro orientation
        
        NEEDED for Megatag2
        '''
        array = [yaw_degrees, yaw_rate_degrees_per_second, pitch_degrees, pitch_rate_degrees_per_second, roll_degrees, roll_rate_degrees_per_second]
        
        # self.table.putNumberArray("robot_orientation_set", array)
        
        latency = self.table.getEntry('tl').getDouble(0) + self.table.getEntry('cl').getDouble(0)
        
        l_t = ntcore._now() + int((latency / 1000))
        
        self.table.getEntry('robot_orientation_set').setDoubleArray(array)

    def target_exists(self, force_update: bool = False):
        """
        Checks if a target exists within the limelight's field of view.

        :param force_update: If True, the limelight variables be updated before checking if a target exists.
                             Defaults to False.

        :return bool: True if a target exists, False if not
        """

        self.tv = self.table.getNumber("tv", 0)
        return self.tv > 0.0

    def get_april_length(self):
        temporary_check = self.table.getNumberArray('rawfiducials', [])
        return len(temporary_check)/7

    def get_latency_total(self):
        latency = self.table.getEntry('tl').getDouble(0) + self.table.getEntry('cl').getDouble(0)
        return latency / 1000

    def april_tag_exists(self) -> bool:
        """
        Checks if an AprilTag exists within the limelight's field of view.

        :param force_update: If True, the limelight variables be updated before checking if a target exists.
                            Defaults to False.

        :return bool: True if an AprilTag exists, False if not
        """

        self.tid = self.table.getNumber("tid", -1)
        return self.tid > 0.0

    def get_target(self, force_update: bool = False):
        """
        Gets the tx, ty values of a target if it exists.

        :param force_update: If True, the limelight variables be updated before checking if a target exists.
                            Defaults to False.

        :return list: [tx, ty] if a target exists

        :return None: if no target exists
        """

        if self.force_update or force_update:
            self.update()
        if self.tv < 1:
            return None
        return (self.tx, self.ty, self.ta)

    def get_bot_pose(
        self,
        megatag2: bool = False,
        round_to: int = 4,
    ):
        """
        IF USING MEGATAG2 USE SET ORIENTATION FUNCTION EVERY CYCLE
        
        Gets the pose of the robot relative to the field using the feducial pipeline.
        This uses the botpose values from the limelight configuration, which are relative to the alliance wall.
        To call this properly, you must set the pipeline to feducial.
        :param team: The team color of the robot. This is used to get the botpose of the robot relative to the
                        alliance wall. can be None if you don't want to use it. 0 for red, 1 for blue.
        :param round_to: The number of decimal places to round the botpose to. Defaults to 4.
        :param force_update: If True, the limelight variables be updated before getting the botpose. Defaults to False.

        :return list: [x, y, z, pitch, yaw, roll] if a target exists

        :return None: if no targets exists
        :return False: if the pipeline is not set to feducial
        """

        
        
        if self.get_pipeline_mode() != LimelightPipeline.feducial:
            return False
        elif not self.april_tag_exists():
            return None

        self.update_bot_pose(megatag2)
        botpose: list = []
        latency:float = 0

        botpose = self.botpose
        #if config.active_team == config.Team.RED:
        #    botpose = self.botpose_red
        #    latency = self.botpose_red_l
        #elif config.active_team == config.Team.BLUE:
        #    botpose = self.botpose_blue
        #    latency = self.botpose_blue_l
        #else:
        #    botpose = self.botpose
        botpose = [round(i, round_to) for i in botpose]
        pose = Pose3d(
            Translation3d(botpose[0], botpose[1], botpose[2]),
            Rotation3d(botpose[3], botpose[4], math.radians(botpose[5])),
        )
        timestamp:float = latency
        tag_count:float = botpose[7]
        tag_span:float = botpose[8]
        ave_tag_dist:float = botpose[9]
        tag_area:float = botpose[10]
        tag_id:float = self.get_target_id()
        return pose, timestamp, tag_count, ave_tag_dist, tag_area, tag_id, megatag2
        
    def get_target_pose(self):
        
        if not self.target_exists():
            return None
        
        pose = Pose3d(
            Translation3d(self.targetpose[0], self.targetpose[1], self.targetpose[2]),
            Rotation3d(self.targetpose[3], self.targetpose[4], math.radians(self.targetpose[5])),
        )
        return pose
        
        
    def enable_moving(self):
        self.cam_pos_moving = True
        
    def disable_moving(self, new_pose: Pose3d):
        self.set_cam_pose(new_pose)
        self.cam_pos_moving = False

    def isConnected(self):
        return True

    def getName(self):
        return self.name


class LimelightController():
    def __init__(self, limelight_list: list[Limelight], gyro, mega_tag2: bool = True):
        super().__init__()
        self.limelights: list[Limelight] = limelight_list
        self.gyro = gyro
        self.mega_tag2 = mega_tag2
        
    def set_orientations(self):
        for limelight in self.limelights:
            if self.mega_tag2:

                def convert(radians):

                    degrees = math.degrees(radians)

                    #todo xyzzy in 2025 limelight is always blue oriented.
                    #if config.active_team == config.Team.RED:
                    #    degrees -= 180
                        
                    return degrees
                
                gyro_data = [
                    self.gyro.getYaw(),
                    # fin_rotation,
                    math.degrees(self.gyro.getRate()),
                    0,0,0,0
                ]
                
                limelight.set_robot_orientation(*gyro_data)   

    def get_estimated_robot_pose(self) -> list[tuple[Pose3d, float, float, float, float]] | None:
        poses = []
        use_megatag_2:bool = False
        self.set_orientations()
        for limelight in self.limelights:
            if self.mega_tag2:
                
                if abs(math.degrees(self.gyro.get_robot_heading_rate())) < odometry_megatag2_max_angular_velocity_degrees_per_sec:
                    use_megatag_2 = True


            if (
                limelight.april_tag_exists()
                and limelight.get_pipeline_mode() == LimelightPipeline.feducial
                and not limelight.cam_pos_moving
            ):
                # print(limelight.name+' Is sending bot pose')
                poses += [limelight.get_bot_pose(use_megatag_2)]
        if len(poses) > 0:
            return poses
        else:
            return None

    def get_detected_objects(self) -> list[tuple[float, float, float]] | None:
        objects = []
        for limelight in self.limelights:
            if (
                limelight.target_exists() is not None
                and limelight.get_pipeline_mode() == LimelightPipeline.neural
            ):
                objects += [limelight.get_target()]
            else:
                objects += [None]
        if len(objects) > 0:
            return objects
        else:
            return None

    def set_pipeline_mode(self, mode: LimelightPipeline) -> None:
        for limelight in self.limelights:
            limelight.set_pipeline_mode(mode)



