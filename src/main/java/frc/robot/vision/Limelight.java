package frc.robot.vision;

import java.util.HashMap;
import java.util.Optional;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.util.PoseUtils;
import frc.robot.vision.LimelightHelpers.RawFiducial;

public class Limelight extends SubsystemBase {

    /* CONSTANTS */
    public static final double coralStationTagHeightMeters = 1.35255; // make sure these two are correct
    // does it need to be to the center of the tag?
    public static final double reefTagHeightMeters = // 0.174625;
            0.3;
    public static final double reefOffsetFromCenterOfTag = 0;

    public static final int[] reefIDsRed = { 6, 7, 8, 9, 10, 11 };
    public static final int[] reefIDsBlue = { 17, 18, 19, 20, 21, 22 };
    public static final int[] reefIDs = { 6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22 };

    public static final int[] coralStationIDsRed = { 1, 2 };
    public static final int[] coralStationIDsBlue = { 12, 13 };
    public static final int[] coralStationIDs = { 1, 2, 12, 13 };
    public static HashMap<Integer, Rotation2d> tagRotationsMap = new HashMap<Integer, Rotation2d>();
    
    {
        tagRotationsMap.put(6, Rotation2d.fromDegrees(120));
        tagRotationsMap.put(7, Rotation2d.fromDegrees(180));
        tagRotationsMap.put(8, Rotation2d.fromDegrees(-120));
        tagRotationsMap.put(9, Rotation2d.fromDegrees(-60));
        tagRotationsMap.put(10, Rotation2d.fromDegrees(0));
        tagRotationsMap.put(11, Rotation2d.fromDegrees(60));

        // TODO: Should these be flipped?
        tagRotationsMap.put(17, Rotation2d.fromDegrees(60));
        tagRotationsMap.put(18, Rotation2d.fromDegrees(0));
        tagRotationsMap.put(19, Rotation2d.fromDegrees(-60));
        tagRotationsMap.put(20, Rotation2d.fromDegrees(-120));
        tagRotationsMap.put(21, Rotation2d.fromDegrees(180));
        tagRotationsMap.put(22, Rotation2d.fromDegrees(120));
    }

    public static final HashMap<Integer, Pose2d> aprilTagPositions = new HashMap<Integer, Pose2d>();

    {
        //for red alliance
        aprilTagPositions.put(1, new Pose2d(657.37, 25.80, Rotation2d.fromDegrees(126)));
        aprilTagPositions.put(2, new Pose2d(657.37, 291.2, Rotation2d.fromDegrees(234)));
        aprilTagPositions.put(3, new Pose2d(455.15, 317.15, Rotation2d.fromDegrees(270)));
        aprilTagPositions.put(4, new Pose2d(365.2, 241.64, Rotation2d.fromDegrees(0)));
        aprilTagPositions.put(5, new Pose2d(365.2, 75.390, Rotation2d.fromDegrees(6)));
        aprilTagPositions.put(6, new Pose2d(530.49, 130.17, Rotation2d.fromDegrees(300)));
        aprilTagPositions.put(7, new Pose2d(546.87, 158.5, Rotation2d.fromDegrees(0)));
        aprilTagPositions.put(8, new Pose2d(530.49, 186.83, Rotation2d.fromDegrees(60)));
        aprilTagPositions.put(9, new Pose2d(497.77, 186.83, Rotation2d.fromDegrees(120)));
        aprilTagPositions.put(10, new Pose2d(481.39, 158.5, Rotation2d.fromDegrees(180)));
        aprilTagPositions.put(11, new Pose2d(497.77, 130.17, Rotation2d.fromDegrees(240)));
        //for blue alliance
        aprilTagPositions.put(12, new Pose2d(33.51, 25.80, Rotation2d.fromDegrees(54)));
        aprilTagPositions.put(13, new Pose2d(33.51, 291.20, Rotation2d.fromDegrees(306)));
        aprilTagPositions.put(14, new Pose2d(325.68, 241.64, Rotation2d.fromDegrees(180)));
        aprilTagPositions.put(15, new Pose2d(325.68, 75.39, Rotation2d.fromDegrees(180)));
        aprilTagPositions.put(16, new Pose2d(235.73, -0.15, Rotation2d.fromDegrees(90)));
        aprilTagPositions.put(17, new Pose2d(160.39, 130.17, Rotation2d.fromDegrees(240)));
        aprilTagPositions.put(18, new Pose2d(144.00, 158.50, Rotation2d.fromDegrees(180)));
        aprilTagPositions.put(19, new Pose2d(160.39, 186.83, Rotation2d.fromDegrees(120)));
        aprilTagPositions.put(20, new Pose2d(193.30, 186.83, Rotation2d.fromDegrees(60)));
        aprilTagPositions.put(21, new Pose2d(209.49, 158.50, Rotation2d.fromDegrees(0)));
        aprilTagPositions.put(22, new Pose2d(193.10, 130.17, Rotation2d.fromDegrees(300)));
    }
    public static final double TARGET_DEBOUNCE_TIME = 0.2;

    /* INSTANCE VARIABLES */
    private int tagCount;
    private int[] validIDs = {}; // TODO: set these
    public String cameraName;
    private double tx;
    private double ty;
    private Debouncer targetDebouncer = new Debouncer(TARGET_DEBOUNCE_TIME, DebounceType.kFalling);

    public static final double angleVelocityTolerance = 360 * Math.PI / 180; // in radians per sec

    private double cameraHeightMeters;
    public double cameraAngle;
    public double cameraOffsetX; // right is positive
    public double cameraOffsetY; // forward is positive
    private double angleMult;

    private boolean hasTipped;

    private DoublePublisher yDistPub;
    private DoublePublisher xDistPub;
    private DoublePublisher horizontalDistPub;

    private SwerveDrivePoseEstimator m_PoseEstimator = new SwerveDrivePoseEstimator(null, getClosestTagAngle(), null, null);
    private Pose2d pose = new Pose2d();

    // TODO setup camera IPs?
    // https://docs.limelightvision.io/docs/docs-limelight/getting-started/FRC/best-practices
    public Limelight(String cameraName, double cameraHeightMeters, double cameraAngle, double cameraOffsetX,
            double cameraOffsetY, boolean cameraUpsideDown) {
        this.cameraName = cameraName;
        this.cameraHeightMeters = cameraHeightMeters;
        this.cameraAngle = cameraAngle;
        this.cameraOffsetX = cameraOffsetX;
        this.cameraOffsetY = cameraOffsetY;
        LimelightHelpers.SetFiducialIDFiltersOverride(cameraName, validIDs);
        if (cameraUpsideDown) {
            angleMult = -1;
        } else {
            angleMult = 1;
        }

        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable lightTable = inst.getTable(cameraName);

        yDistPub = lightTable.getDoubleTopic("Y Distance").publish();
        xDistPub = lightTable.getDoubleTopic("X Distance").publish();
        horizontalDistPub = lightTable.getDoubleTopic("Horizontal Distance").publish();
    }

    // might not be needed
    public static boolean isCorrectID(int ID, int... IDs) {
        for (int n : IDs) {
            if (n == ID)
                return true;
        }
        return false;
    }

    // from last years robot
    public double getTimestampSeconds() {
        double latency = (LimelightHelpers.getLimelightNTDouble(cameraName, "cl")
                + LimelightHelpers.getLimelightNTDouble(cameraName, "tl"))
                / 1000.0;

        return Timer.getFPGATimestamp() - latency;
    }

    // from last years robot as well
    public boolean hasValidTarget() {
        boolean hasMatch = (LimelightHelpers.getLimelightNTDouble(cameraName, "tv") == 1.0);
        return targetDebouncer.calculate(hasMatch);
    }

    public void disable() {
        // https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-robot-localization-megatag2#using-limelight-4s-built-in-imu-with-imumode_set--setimumode
        // https://docs.limelightvision.io/docs/docs-limelight/software-change-log#limelight-os-20251-final-release---22425-test-release---21825
        LimelightHelpers.SetIMUMode(cameraName, 1); // If not moving reset internal IMU
        // LimelightHelpers.setLimelightNTDouble(cameraName, "throttle_set", 200); //
        // manage thermals
    }

    public void setGyroMode(int mode) {
        LimelightHelpers.SetIMUMode(cameraName, mode);
    }

    public void enable() {
        LimelightHelpers.SetIMUMode(cameraName, 1); // if moving use builtin, maybe change to 4
        // LimelightHelpers.setLimelightNTDouble(cameraName, "throttle_set", 0); //TODO
        // check needs to be 1? // manage thermals
    }

    public RawFiducial getClosestTag() {
        RawFiducial[] tags = LimelightHelpers.getRawFiducials(cameraName);
        if (tags.length == 0) {
            return null;
        }
        RawFiducial largest = tags[0];
        for (RawFiducial tag : tags) {
            if (tag.distToRobot > largest.distToRobot) {
                largest = tag;
            }
        }
        return largest;
    }

    public Rotation2d getClosestTagAngle() {
        int closestId = getClosestTag().id;
        return tagRotationsMap.get(closestId);
    }


    // TODO: Do we need these / check if the trig is right

    public double getDistanceToTag(double tagHeightMeters) {
        if (hasValidTarget()) {
            double distance = getStraightDistanceToTag(tagHeightMeters) - cameraOffsetY;
            return distance / Math.cos((Math.PI / 180.0) * getTX());
        }
        return 0;
    }

    public double getStraightDistanceToTag(double tagHeightMeters) {
        if (hasValidTarget()) {
            double distance = (tagHeightMeters - cameraHeightMeters)
                    / Math.tan(
                            (Math.PI / 180.0)
                                    * (cameraAngle + getTY()));
            return distance + cameraOffsetY;
        }
        return 0;
    }

    public double getHorizontalDistanceToTag(double tagHeightMeters) {
        if (hasValidTarget()) {
            double distance = getStraightDistanceToTag(tagHeightMeters) - cameraOffsetY;

            distance = distance * Math.tan(getTX() * (Math.PI / 180.0));
            return distance + cameraOffsetX;
        }
        return 0;
    }

    public double getDistanceToCoralStation() {
        return getDistanceToTag(coralStationTagHeightMeters);
    }

    public double getStraightDistanceToCoralStation() {
        return getStraightDistanceToTag(coralStationTagHeightMeters);

    }

    public double getHorizontalDistanceToCoralStation() {
        return getHorizontalDistanceToTag(coralStationTagHeightMeters);
    }

    // ISN'T OFFSET FOR THE CENTER OF THE ROBOT!!!!!!!
    public double getDistanceToReef() {
        return getDistanceToTag(reefTagHeightMeters);
    }

    // TODO: Do we need these / check if the trig is right
    public double getStraightDistanceToReef() {
        return getStraightDistanceToTag(reefTagHeightMeters);
    }

    public double getHorizontalDistanceToReef() {
        return getHorizontalDistanceToTag(reefTagHeightMeters);
    }

    @AutoLogOutput
    public double getTX() {
        return tx * angleMult;
    }

    @AutoLogOutput
    public double getTY() {
        return ty * -angleMult;
    }

    public DoubleSupplier tySupplier() {
        return () -> getTY();
    }

    public DoubleSupplier txSupplier() {
        return () -> getTX();
    }

    // TODO: Do we need these / check if the trig is right
    // public double getStraightDistanceToTag() {
    // if (hasValidTarget())
    // return goalHeightReef / (Math.tan(Math.toRadians(getTY() +
    // limelightOffsetAngleVertical)));
    // return 0;
    // }

    // TODO: Do we need these / check if the trig is right
    public double getStrafeDistanceToReef() {
        if (isCorrectID(getTagID(), reefIDs)) {
            return (Math.tan(Math.toRadians(getTX()))) * getStraightDistanceToReef();
        }
        return 0;
    }

    public int getTagID() {
        return (int) LimelightHelpers.getFiducialID(cameraName);
    }

    public void periodic() {

        // tagID = (int) Limetable.getEntry("tid").getDouble(-1);
        // TODO if you get a pose estimate in the frame before this is applied it may
        // not work
        tx = LimelightHelpers.getTX(cameraName);
        ty = LimelightHelpers.getTY(cameraName);
        RawFiducial[] allTags = LimelightHelpers.getRawFiducials(cameraName);
        int numValidTags = 0;
        for (LimelightHelpers.RawFiducial t : allTags) {
            if (t.distToCamera < 4.0) {
                numValidTags++;
            }
        }

        int[] validTags = new int[numValidTags];
        int counter = 0;
        for (RawFiducial t : allTags) {
            if (t.distToCamera < 4.0) {
                validTags[counter] = t.id;
                counter++;
            }
        }
        // LimelightHelpers.SetFiducialIDFiltersOverride(cameraName, validTags);
        xDistPub.set(getHorizontalDistanceToReef());
        yDistPub.set(getStraightDistanceToReef());
        horizontalDistPub.set(getDistanceToReef());

        double[] poseArr = LimelightHelpers.getBotPose_TargetSpace(cameraName);
        Pose2d botPose = new Pose2d();
        if (poseArr.length >= 6) {
            botPose = new Pose2d(poseArr[0], poseArr[2], Rotation2d.fromDegrees(poseArr[4]));
        }
        Logger.recordOutput(cameraName + "/IMUYaw",
                LimelightHelpers.getIMUData(cameraName).robotYaw * (Math.PI / 180.0)); // TODO should be yaw?
        Logger.recordOutput(cameraName + "/BotPoseTargetSpace", botPose);
        Logger.recordOutput(cameraName + "/BotPose3dTargetSpace",
                LimelightHelpers.getBotPose3d_TargetSpace(cameraName));

        var entry = LimelightHelpers.getLimelightNTTableEntry(cameraName, "tcornxy");
        if (entry != null) {
            var tcornxy = entry.getDoubleArray(new double[0]);
            if (tcornxy != null && tcornxy.length > 0) {
                Logger.recordOutput(cameraName + "/tcornxy", tcornxy);
            }
        }
    }

    public void updateOdometry(){
        Optional<Alliance>ally = DriverStation.getAlliance();
        LimelightHelpers.PoseEstimate mt1 = null;
        boolean doRejectUpdate = false;
        

        if(ally.isPresent()){
            if (ally.get() == Alliance.Red){
                mt1 = LimelightHelpers.getBotPoseEstimate_wpiRed("limelight");
            }
            if (ally.get() == Alliance.Blue){
                mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
            }
        }
        else {
            
        }

        if (mt1.tagCount == 1 && mt1.rawFiducials.length == 1){
            if (mt1.rawFiducials[0].ambiguity > 0.7){
                doRejectUpdate = true;
            }
            if (mt1.rawFiducials[0].distToCamera > 3){
                doRejectUpdate = true;
            }
        }

        if (mt1.tagCount == 0){
            doRejectUpdate = true;
        }

        if (!doRejectUpdate){
            m_PoseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.5, 0.5, 9999999));
            m_PoseEstimator.addVisionMeasurement(mt1.pose, mt1.timestampSeconds);
        }

        //pose = m_PoseEstimator.getEstimatedPosition();

        //pose = 
    }

    public Pose2d getPose(){
        return pose;
    }
    




    public Command flashLEDs() {
        return Commands.sequence(
            Commands.runOnce(() -> LimelightHelpers.setLEDMode_ForceBlink(cameraName)),
            Commands.waitSeconds(0.6),
            Commands.runOnce(() -> LimelightHelpers.setLEDMode_ForceOff(cameraName))
        );
    }



    public Command ifHasTarget(Command cmd) {
        return cmd.onlyWhile(this::hasValidTarget);
    }

}
