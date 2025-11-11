package frc.robot.config;

import frc.robot.subsystems.arm.ArmIO;
import frc.robot.subsystems.arm.SimArmIO;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.SimElevatorIO;
import frc.robot.subsystems.vision.VisionIO;
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
    public VisionIO2[] createVisionIo2() {
        return new VisionIO2[0];
    }
}
