// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.config.RobotFactory;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.grabber.Grabber;
import frc.robot.subsystems.led.Led;
import frc.robot.subsystems.vision.Vision;
import frc.robot.utility.ControllerHelper;

import java.util.*;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

@Logged
public class RobotContainer {
    private final Drivetrain drivetrain;
    private final Elevator elevator;
    private final Arm arm;
    private final Grabber grabber;
    private final Climber climber;
    private final Led led;
    private final Vision vision;
    private final SuperStructure superStructure;
    private final AutoFactory autoFactory;
    private final AutoChooser autoChooser;

    private final CommandFactory commandFactory;
    private final AutoPaths autoPaths;

    @NotLogged
    private final CommandXboxController driverController = new CommandXboxController(0);
    @NotLogged
    private final CommandXboxController operatorController = new CommandXboxController(1);

    private RobotMode robotMode = RobotMode.CORAL_LEVEL_1;
    private boolean haveAlgae;

    public RobotContainer(RobotFactory robotFactory) {
        drivetrain = new Drivetrain(robotFactory.createDrivetrainIo());
        elevator = new Elevator(robotFactory.createElevatorIo());
        arm = new Arm(robotFactory.createArmIo());
        grabber = new Grabber(robotFactory.createGrabberIo());
        climber = new Climber(robotFactory.createClimberIo());
        led = new Led(robotFactory.createLedIo());
        vision = new Vision(AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded), robotFactory.createVisionIo());
        superStructure = new SuperStructure(elevator, arm, () -> haveAlgae);

        commandFactory = new CommandFactory(drivetrain, elevator, arm, grabber, climber, led, superStructure);
        autoPaths = new AutoPaths(drivetrain, superStructure, drivetrain.getAutoFactory());

        vision.setDefaultCommand(vision.consumeVisionMeasurements(drivetrain::addVisionMeasurements, drivetrain::getYaw).ignoringDisable(true));

        configureBindings();

        drivetrain.setDefaultCommand(
                drivetrain.drive(this::getDrivetrainXVelocity, this::getDrivetrainYVelocity, this::getDrivetrainAngularVelocity)
        );
        led.setDefaultCommand(led.signalCommand(() -> robotMode));
//        superStructure.setDefaultCommand(superStructure.goToDefaultPositions());

        // Creates Auto Chooser
        autoChooser = new AutoChooser();

        // Add options to the shooter
        autoChooser.addRoutine("FacePlantG", autoPaths::facePlantG);
        autoChooser.addRoutine("FacePlantH", autoPaths::facePlantH);

        autoChooser.addRoutine("TwoCoralDC", autoPaths::twoCoralDC);
        autoChooser.addRoutine("TwoCoralFE", autoPaths::twoCoralFE);
        autoChooser.addRoutine("TwoCoralIJ", autoPaths::twoCoralIJ);
        autoChooser.addRoutine("TwoCoralKL", autoPaths::twoCoralKL);

        autoChooser.addRoutine("BehindTheBackAB", autoPaths::behindTheBackAB);
        autoChooser.addRoutine("BehindTheBackFlippedBA", autoPaths::behindTheBackFlippedBA);

        autoChooser.addRoutine("ThreeCoralEDC", autoPaths::threeCoralEDC);
        autoChooser.addRoutine("ThreeCoralGFE", autoPaths::threeCoralGFE);
        autoChooser.addRoutine("ThreeCoralJKL", autoPaths::threeCoralJKL);

        autoChooser.addRoutine("FourCoralFCDE", autoPaths::fourCoralFCDE);
        autoChooser.addRoutine("FourCoralFlippedILKJ", autoPaths::fourCoralFlippedILKJ);

        autoChooser.addRoutine("FiveCoralFBCDE", autoPaths::fiveCoralFBCDE);
        autoChooser.addRoutine("FiveCoralFlippedIALKJ", autoPaths::fiveCoralFlippedIALKJ);

        // Getting the auto factory
        autoFactory = drivetrain.getAutoFactory();

        // Puts auto chooser on the dashboard
        Shuffleboard.getTab("Driver")
                .add("Auto Chooser", autoChooser)
                .withPosition(0, 0)
                .withSize(3, 1)
                .withWidget(BuiltInWidgets.kComboBoxChooser);

        // Schedule the selected auto during the autonomous period
        RobotModeTriggers.autonomous().whileTrue(
                Commands.sequence(
                        autoChooser.selectedCommandScheduler()
                ));
    }

    private void configureBindings() {
        driverController.rightBumper().whileTrue(commandFactory.alignToReef(() -> robotMode));
        driverController.rightTrigger().and(driverController.rightBumper())
                .and(superStructure::atTargetPositions)
                .whileTrue(commandFactory.releaseGamePiece(() -> robotMode));

        driverController.leftBumper().whileTrue(commandFactory.alignToCoralStation());
        driverController.back().onTrue(drivetrain.resetFieldOriented());
        driverController.start().onTrue(superStructure.zero().alongWith(climber.zero()));

        operatorController.leftBumper().onTrue(climber.deploy());
        operatorController.rightBumper().onTrue(climber.climb());

        new Trigger(grabber::getGamePieceDetected).debounce(Constants.DEBOUNCE_TIME).onTrue(led.intakeFlash());

        new Trigger(() -> grabber.getGamePieceDetected() && robotMode.isAlgaeMode()).debounce(Constants.DEBOUNCE_TIME).onTrue(runOnce(() -> haveAlgae = true));
        new Trigger(() -> grabber.getGamePieceNotDetected()).debounce(Constants.DEBOUNCE_TIME).onTrue(runOnce(() -> haveAlgae = false));

        // Operator Controller Robot Mode
        operatorController.a().onTrue(setRobotMode(RobotMode.ALGAE_PROCESSOR));
        operatorController.y().onTrue(setRobotMode(RobotMode.ALGAE_NET));
        operatorController.x().onTrue(setRobotMode(RobotMode.ALGAE_REMOVE_UPPER));
        operatorController.b().onTrue(setRobotMode(RobotMode.ALGAE_REMOVE_LOWER));

        operatorController.povUp().onTrue(setRobotMode(RobotMode.CORAL_LEVEL_4));
        operatorController.povLeft().onTrue(setRobotMode(RobotMode.CORAL_LEVEL_3));
        operatorController.povRight().onTrue(setRobotMode(RobotMode.CORAL_LEVEL_2));
        operatorController.povDown().onTrue(setRobotMode(RobotMode.CORAL_LEVEL_1));

//         Driver Controller Robot Mode
        driverController.a().onTrue(setRobotMode(RobotMode.ALGAE_PROCESSOR));
        driverController.y().onTrue(setRobotMode(RobotMode.ALGAE_NET));
        driverController.x().onTrue(setRobotMode(RobotMode.ALGAE_REMOVE_UPPER));
        driverController.b().onTrue(setRobotMode(RobotMode.ALGAE_REMOVE_LOWER));

//        driverController.a().whileTrue(arm.sysIdQuasistaticShoulder(SysIdRoutine.Direction.kForward));
//        driverController.b().whileTrue(arm.sysIdQuasistaticShoulder(SysIdRoutine.Direction.kReverse));
//        driverController.x().whileTrue(arm.sysIdDynamicShoulder(SysIdRoutine.Direction.kForward));
//        driverController.y().whileTrue(arm.sysIdDynamicShoulder(SysIdRoutine.Direction.kReverse));

        driverController.povUp().onTrue(setRobotMode(RobotMode.CORAL_LEVEL_4));
        driverController.povLeft().onTrue(setRobotMode(RobotMode.CORAL_LEVEL_3));
        driverController.povRight().onTrue(setRobotMode(RobotMode.CORAL_LEVEL_2));
        driverController.povDown().onTrue(setRobotMode(RobotMode.CORAL_LEVEL_1));
    }

    private Command setRobotMode(RobotMode robotMode) {
        return runOnce(() -> this.robotMode = robotMode);
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
