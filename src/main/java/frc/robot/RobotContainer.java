// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.P2025ArmIO;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.P2025ElevatorIO;

@Logged
public class RobotContainer {
    private final Elevator elevator = new Elevator(new P2025ElevatorIO(
            new TalonFX(0), new TalonFX(1), 0));
    private final Arm arm = new Arm(new P2025ArmIO(new TalonFX(3)));

    private final SuperStructure superStructure = new SuperStructure(elevator, arm);

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
