// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.CANdi;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.config.RobotFactory;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.C2025ArmIO;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.elevator.C2025ElevatorIO;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.utility.ControllerHelper;
import frc.robot.subsystems.grabber.Grabber;
import frc.robot.subsystems.grabber.C2025GrabberIO;

@Logged
public class RobotContainer {
    private final Drivetrain drivetrain;
    private final Elevator elevator = new Elevator(new C2025ElevatorIO(
            new TalonFX(0), new TalonFX(1), 0));
    private final Arm arm = new Arm(new C2025ArmIO(new TalonFX(3), new TalonFX(4)));
    private final Grabber grabber = new Grabber(new C2025GrabberIO(new TalonFX(0), new CANdi(0)));

    private final SuperStructure superStructure = new SuperStructure(elevator, arm);

    private final CommandXboxController driverController = new CommandXboxController(0);

    public RobotContainer(RobotFactory robotFactory) {
        drivetrain = new Drivetrain(robotFactory.createDrivetrainIo());

        configureBindings();

        drivetrain.setDefaultCommand(
                drivetrain.drive(this::getDrivetrainXVelocity, this::getDrivetrainYVelocity, this::getDrivetrainAngularVelocity)
        );
    }

    private void configureBindings() {
        driverController.a().onTrue(arm.stop());
        driverController.b().onTrue(arm.goToPositions(Math.PI, -Math.PI));
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }

    private double getDrivetrainXVelocity() {
        return -ControllerHelper.modifyAxis(driverController.getLeftY(), 1.0);
    }

    private double getDrivetrainYVelocity() {
        return -ControllerHelper.modifyAxis(driverController.getLeftX(), 1.0);
    }

    private double getDrivetrainAngularVelocity() {
        return -ControllerHelper.modifyAxis(driverController.getRightX(), 1.0);
    }
}
