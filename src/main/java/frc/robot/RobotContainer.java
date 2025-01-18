// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.P2025ArmIO;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.C2025ElevatorIO;
import frc.robot.utility.ControllerHelper;

@Logged
public class RobotContainer {
    private final Elevator elevator = new Elevator(new C2025ElevatorIO(
            new TalonFX(0), new TalonFX(1), 0));
    private final Arm arm = new Arm(new P2025ArmIO(new TalonFX(3)));

    private final SuperStructure superStructure = new SuperStructure(elevator, arm);

    private final CommandXboxController controller = new CommandXboxController(0);

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        elevator.setDefaultCommand(elevator.followPosition(
                () -> ControllerHelper.modifyAxis(controller.getLeftY(), Units.feetToMeters(2.0))));
        arm.setDefaultCommand(arm.followPosition(
                () -> ControllerHelper.modifyAxis(controller.getRightY(), Units.feetToMeters(2.0))));

        controller.a().onTrue(superStructure.stop());
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}