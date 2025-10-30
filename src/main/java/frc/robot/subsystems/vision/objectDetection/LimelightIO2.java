package frc.robot.subsystems.vision.objectDetection;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.vision.VisionIO;

@Logged
public class LimelightIO2 implements VisionIO {
    private static final int LIMELIGHT_IMU_EXTERNAL = 0;
    private static final int LIMELIGHT_IMU_FUSED = 1;
    private static final int LIMELIGHT_IMU_INTERNAL = 2;

    private final String cameraName;

    public LimelightIO2(String name, Transform3d transform) {
        this.cameraName = name;


        // Initially the Limelight IMU should be in FUSED mode, it will change when robot is enabled.
        // NOPE THE ABOVE IS NOT HAPPENING AT ALL
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

    public void updateInputs(VisionInputs2 visionInputs, double currentYaw) {
        // Assume pose will not be updated.
        visionInputs.poseUpdated = new boolean[]{};

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

        // Get raw neural detector results
        LimelightHelpers.RawDetection[] detections = LimelightHelpers.getRawDetections(cameraName);

        // processed if valid estimate.
        double filterStart = System.currentTimeMillis();
        if (detections.length > 0) {
            Translation2d[] poses = new Translation2d[detections.length];
            int[] ids = new int[detections.length];
            boolean[] updated = new boolean[detections.length];
            for (int i = 0; i < detections.length; i++) {
                LimelightHelpers.RawDetection detection = detections[i];

                double rotation = imuData.robotYaw + Units.degreesToRadians(detection.txnc);
                Pose3d cameraPose = LimelightHelpers.getCameraPose3d_RobotSpace(cameraName);
                double r = cameraPose.getZ() * Math.tan(Units.degreesToRadians(detection.tync) + cameraPose.getRotation().getX());
                double x = r * Math.cos(rotation);
                double y = r * Math.sin(rotation);
                Translation2d pose = new Translation2d(x, y);

                poses[i] = pose;
                ids[i] = detection.classId;
                updated[i] = true;
            }
            visionInputs.poses = poses;
            visionInputs.ids = ids;
            visionInputs.poseUpdated = updated;
        }
        double filterEnd = System.currentTimeMillis();
        visionInputs.filterDuration = filterEnd - filterStart;
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