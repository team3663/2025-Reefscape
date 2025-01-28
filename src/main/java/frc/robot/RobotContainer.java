// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.config.RobotFactory;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.grabber.Grabber;
import frc.robot.subsystems.led.Led;
import frc.robot.utility.ControllerHelper;

import java.util.HashMap;
import java.util.Map;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

@Logged
public class RobotContainer {
    private final Drivetrain drivetrain;
    private final Elevator elevator;
    private final Arm arm;
    private final Grabber grabber;
    private final Climber climber;
    private final Led led;
    private final SuperStructure superStructure;

    private final CommandFactory commandFactory;

    @NotLogged
    private final CommandXboxController driverController = new CommandXboxController(0);
    @NotLogged
    private final CommandXboxController operatorController = new CommandXboxController(1);

    private RobotMode robotMode = RobotMode.CORAL_LEVEL_1;

    public RobotContainer(RobotFactory robotFactory) {
        drivetrain = new Drivetrain(robotFactory.createDrivetrainIo());
        elevator = new Elevator(robotFactory.createElevatorIo());
        arm = new Arm(robotFactory.createArmIo());
        grabber = new Grabber(robotFactory.createGrabberIo());
        climber = new Climber(robotFactory.createClimberIo());
        led = new Led(robotFactory.createLedIo());
        superStructure = new SuperStructure(elevator, arm);

        commandFactory = new CommandFactory(drivetrain, elevator, arm, grabber, climber, led, superStructure);

        configureBindings();

        drivetrain.setDefaultCommand(
                drivetrain.drive(this::getDrivetrainXVelocity, this::getDrivetrainYVelocity, this::getDrivetrainAngularVelocity)
        );
        led.setDefaultCommand(led.signalCommand(() -> robotMode));
    }

    private void configureBindings() {
        // Make a Robot Command map where each item in the RobotMode Enum is mapped to a command to go to the corresponding position
        Map<RobotMode, Command> robotModeCommandMap = new HashMap<>();
        robotModeCommandMap.put(RobotMode.CORAL_LEVEL_1, commandFactory.goToL1());
        robotModeCommandMap.put(RobotMode.CORAL_LEVEL_2, commandFactory.goToL2());
        robotModeCommandMap.put(RobotMode.CORAL_LEVEL_3, commandFactory.goToL3());
        robotModeCommandMap.put(RobotMode.CORAL_LEVEL_4, commandFactory.goToL4());
        robotModeCommandMap.put(RobotMode.ALGAE_NET, commandFactory.goToNet());
        robotModeCommandMap.put(RobotMode.ALGAE_PROCESSOR, commandFactory.goToProcessor());
        robotModeCommandMap.put(RobotMode.ALGAE_REMOVE_UPPER, commandFactory.goToRemoveUpper());
        robotModeCommandMap.put(RobotMode.ALGAE_REMOVE_LOWER, commandFactory.goToRemoveLower());

        driverController.rightBumper().onTrue(Commands.select(robotModeCommandMap, () -> this.robotMode));

        driverController.x().onTrue(
                Commands.parallel(
                        elevator.goToPosition(1.0),
                        arm.goToPositions(Units.degreesToRadians(120.0), Units.degreesToRadians(20.0))
                ));
        driverController.b().onTrue(
                Commands.parallel(
                        elevator.goToPosition(0.0),
                        arm.goToPositions(0.0, 0.0)
                ));
        driverController.back().onTrue(drivetrain.resetFieldOriented());
    }

    private Command setRobotMode(RobotMode robotMode) {
        return runOnce(() -> this.robotMode = robotMode);
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }

    private double getDrivetrainXVelocity() {
        return -ControllerHelper.modifyAxis(driverController.getLeftY(), drivetrain.getConstants().maxLinearVelocity());
    }

    private double getDrivetrainYVelocity() {
        return -ControllerHelper.modifyAxis(driverController.getLeftX(), drivetrain.getConstants().maxLinearVelocity());
    }

    private double getDrivetrainAngularVelocity() {
        return -ControllerHelper.modifyAxis(driverController.getRightX(), drivetrain.getConstants().maxAngularVelocity());
    }
}
