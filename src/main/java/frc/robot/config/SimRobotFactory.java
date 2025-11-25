package frc.robot.config;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.Constants;
import frc.robot.subsystems.arm.ArmIO;
import frc.robot.subsystems.arm.SimArmIO;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.SimElevatorIO;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.objectDetection.LimelightIO2;
import frc.robot.subsystems.vision.objectDetection.VisionIO2;

public class SimRobotFactory extends C2025RobotFactory {
    @Override
    public ArmIO createArmIo() {
        return new SimArmIO();
    }

    @Override
    public ElevatorIO createElevatorIo() {
        return new SimElevatorIO();
    }

    @Override
    public VisionIO[] createVisionIo() {
        return new VisionIO[0];
    }

    @Override
    public VisionIO2 createVisionIo2() {
        Rotation3d backRotation = new Rotation3d(Constants.FRONT_LEFT_CAMERA_ROLL, Constants.FRONT_LEFT_CAMERA_PITCH, Constants.FRONT_LEFT_CAMERA_YAW);
        Transform3d backTransform = new Transform3d(Constants.FRONT_LEFT_CAMERA_X, Constants.FRONT_LEFT_CAMERA_Y, Constants.FRONT_LEFT_CAMERA_Z, backRotation);

        return new LimelightIO2(Constants.FRONT_LEFT_CAMERA_NAME, backTransform);
    }
}
