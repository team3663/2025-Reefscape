package frc.robot.config;

import frc.robot.subsystems.arm.ArmIO;
import frc.robot.subsystems.arm.SimArmIO;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.SimElevatorIO;

public class SimRobotFactory extends C2025RobotFactory {
    @Override
    public ArmIO createArmIo() {
        return new SimArmIO();
    }

    @Override
    public ElevatorIO createElevatorIo() {
        return new SimElevatorIO();
    }
}
