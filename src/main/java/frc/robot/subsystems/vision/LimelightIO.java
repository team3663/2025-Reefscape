package frc.robot.subsystems.vision;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;

@Logged
public class LimelightIO implements VisionIO {
    private static final int LIMELIGHT_IMU_EXTERNAL = 0;
    private static final int LIMELIGHT_IMU_FUSED = 1;
    private static final int LIMELIGHT_IMU_INTERNAL = 2;

    private final String cameraName;
    private boolean isIgnoredNet;
    private VisionInputs visionInputs;
    private final String objectDetectionCamera = Constants.FRONT_LEFT_CAMERA_NAME;

    public LimelightIO(String name, Transform3d transform, boolean isIgnored) {
        this.cameraName = name;
        this.isIgnoredNet= isIgnored;


        // Initially the Limelight IMU should be in FUSED mode, it will change when robot is enabled.
        LimelightHelpers.SetIMUMode(cameraName, LIMELIGHT_IMU_EXTERNAL);

        // Tell the limelight were on the robot it is located.
        Rotation3d rotation = transform.getRotation();
        LimelightHelpers.setCameraPose_RobotSpace(name,
                transform.getX(),
                -transform.getY(),
                transform.getZ(),
                Units.radiansToDegrees(rotation.getX()),
                Units.radiansToDegrees(rotation.getY()),
                Units.radiansToDegrees(rotation.getZ()));
    }
    @Override
    public boolean isIgnoredIfNotNet(){
        return isIgnoredNet;
    }

    public void updateInputs(VisionInputs visionInputs, double currentYaw) {
        this.visionInputs = visionInputs;
        // Assume pose will not be updated.
        visionInputs.poseUpdated = false;

        // Give the Limelight our current robot yaw as provided by the Pigeon.
        double orientationStart = System.currentTimeMillis();
        LimelightHelpers.SetRobotOrientation(cameraName, Units.radiansToDegrees(currentYaw), 0, 0, 0, 0, 0);
        double orientationEnd = System.currentTimeMillis();
        visionInputs.orientationDuration = orientationEnd - orientationStart;

        // reading the yaw from the limelights internal IMU
        double imuStart = System.currentTimeMillis();
        LimelightHelpers.IMUData imuData = LimelightHelpers.getIMUData(cameraName);
        visionInputs.IMUYaw = imuData.Yaw;
        double imuEnd = System.currentTimeMillis();
        visionInputs.imuDataDuration = imuEnd - imuStart;

        // Get a new pose estimate
        double poseStart = System.currentTimeMillis();
        LimelightHelpers.PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(cameraName);
        double poseEnd = System.currentTimeMillis();
        visionInputs.poseEstimateDuration = poseEnd - poseStart;


        // processed if valid estimate.
        double filterStart = System.currentTimeMillis();
        if (estimate != null && estimate.tagCount != 0) {
            visionInputs.estimatedPose = estimate.pose;
            visionInputs.timestampSeconds = estimate.timestampSeconds;

            // Extract list of AprilTag Ids see in this pose estimate.
            int[] targetIds = new int[estimate.rawFiducials.length];
            int index = 0;
            for (LimelightHelpers.RawFiducial tag : estimate.rawFiducials) {
                targetIds[index++] = tag.id;
            }
            visionInputs.targetIds = targetIds;
            visionInputs.poseUpdated = true;
        }
        double filterEnd = System.currentTimeMillis();
        visionInputs.filterDuration = filterEnd - filterStart;

        LimelightHelpers.RawDetection[] detections = LimelightHelpers.getRawDetections(objectDetectionCamera);
        for (LimelightHelpers.RawDetection detection : detections){
            int classId = detection.classId;
            double txnc = detection.txnc;
            double tync = detection.tync;
            double ta = detection.ta;
            // add corners here if needed
        }

    }

    Pose2d coralFieldPose = new Pose2d();

    public Pose2d getCoralPose() {
        Translation2d coralTranslationFromRobot;
        Pose2d nextCoralFieldPose = coralFieldPose;
        if (coralInVision()) {
            Translation2d coralTranslationFromCamera = new Translation2d(
                    //TODO - I don't think this gives me what I want, add the math aspect
                    LimelightHelpers.getTXNC(objectDetectionCamera),
                    -LimelightHelpers.getTYNC(objectDetectionCamera)
            );

            coralTranslationFromRobot = coralTranslationFromCamera
                    .plus(new Translation2d(Constants.FRONT_LEFT_CAMERA_X, Constants.FRONT_LEFT_CAMERA_Y))
                    .rotateBy(new Rotation3d(Constants.FRONT_LEFT_CAMERA_ROLL, Constants.FRONT_LEFT_CAMERA_PITCH, Constants.FRONT_LEFT_CAMERA_YAW).toRotation2d());

            nextCoralFieldPose = visionInputs.estimatedPose.plus(new Transform2d(coralTranslationFromRobot, new Rotation2d()));
        }

//        Rotation2d slopeAngle = new Rotation2d(
//                coralTranslationFromRobot.getX(),
//                coralTranslationFromRobot.getY())
//                .plus(Rotation2d.fromDegrees(180.0));

        // TODO - Write the Code that add's an offset from point to robot "intake"
        return nextCoralFieldPose;
    }

    public Boolean coralInVision(){
//        return (inputs.validResult
//                && inputs.targetTxs.length > 0
//                && inputs.limelightTA > 0.5
//                && inputs.targetTxs[0] != 0
//                && inputs.targetTys[0] != 0);
        // TODO - Understand this and rewrite it to fit our code
    }

    public double estimateObjectDistance(){
        // TODO - figure out what ty is and if I need to give something to access this
        double targetOffsetAngle_Vertical = ty.getDouble(0.0);

        // how many degrees back is your limelight rotated from perfectly vertical?
        double limelightMountAngleDegrees = Constants.FRONT_LEFT_CAMERA_PITCH;

        // distance from the center of the Limelight lens to the floor
        double limelightLensHeightInches = Constants.FRONT_LEFT_CAMERA_Y;

        // distance from the target to the floor
        double goalHeightInches = 0.0;

        double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
        double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

        //calculate distance
        return (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);
    }

    public void robotStateChanged() {
        // When the robot is disabled then seed the limelight's IMU with data from the Pigeon but once
        // the robot is enabled then switch to the Limelight's internal IMU.
//        if (RobotState.isDisabled()) {
//            LimelightHelpers.SetIMUMode(cameraName, LIMELIGHT_IMU_FUSED);
//        } else {
//            LimelightHelpers.SetIMUMode(cameraName, LIMELIGHT_IMU_INTERNAL);
//        }
    }
}
