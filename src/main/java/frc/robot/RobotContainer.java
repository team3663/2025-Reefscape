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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.config.RobotFactory;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.grabber.Grabber;
import frc.robot.subsystems.led.Led;
import frc.robot.subsystems.vision.Vision;
import frc.robot.utility.ControllerHelper;

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
        vision = new Vision(AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark), robotFactory.createVisionIo());
        superStructure = new SuperStructure(elevator, arm, () -> haveAlgae);

        commandFactory = new CommandFactory(drivetrain, elevator, arm, grabber, climber, led, superStructure);
        autoPaths = new AutoPaths(drivetrain, grabber, superStructure, drivetrain.getAutoFactory(),arm, commandFactory);

        vision.setDefaultCommand(vision.consumeVisionMeasurements(drivetrain::addVisionMeasurements, drivetrain::getYaw).ignoringDisable(true));

        configureBindings();

        drivetrain.setDefaultCommand(
                drivetrain.drive(this::getDrivetrainXVelocity, this::getDrivetrainYVelocity, this::getDrivetrainAngularVelocity)
        );
        led.setDefaultCommand(led.signalCommand(() -> robotMode));
        superStructure.setDefaultCommand(superStructure.goToDefaultPositions());

        // Creates Auto Chooser
        autoChooser = new AutoChooser();

        // Add options to the shooter
        autoChooser.addRoutine("FacePlantG", autoPaths::facePlantG);
        autoChooser.addRoutine("FacePlantH", autoPaths::facePlantH);

        autoChooser.addRoutine("TwoCoralDC", autoPaths::twoCoralDC);
        autoChooser.addRoutine("TwoCoralKL", autoPaths::twoCoralKL);

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

        SmartDashboard.putBoolean("Auto Reef", true);
        SmartDashboard.putBoolean("Auto Coral Station", true);
    }

    private void configureBindings() {
        driverController.rightBumper().whileTrue(commandFactory.alignToReef(() -> robotMode,
                this::getDrivetrainXVelocity, this::getDrivetrainYVelocity, this::getDrivetrainAngularVelocity));
        driverController.rightTrigger().and(driverController.rightBumper())
                .and(superStructure::atTargetPositions)
                .whileTrue(commandFactory.releaseGamePiece(() -> robotMode));

        driverController.leftTrigger().whileTrue(
                Commands.either(Commands.idle(), commandFactory.alignToCoralStation(), grabber::isGamePieceDetected));
        driverController.back().onTrue(drivetrain.resetFieldOriented());
        driverController.start().onTrue(superStructure.zero().alongWith(climber.zero()));

        driverController.a().whileTrue(grabber.withVoltage(-6.0));

        operatorController.leftBumper().onTrue(climber.deploy());
        operatorController.rightBumper().onTrue(climber.climb());

        new Trigger(grabber::isGamePieceDetected).debounce(Constants.DEBOUNCE_TIME).onTrue(led.intakeFlash());

        new Trigger(() -> grabber.getGamePieceDetected()).and(robotMode::isAlgaeMode).debounce(Constants.DEBOUNCE_TIME).onTrue(runOnce(() -> haveAlgae = true));
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

        // Test code for SysID
//        driverController.leftStick().onTrue(Commands.runOnce(SignalLogger::start));
//        driverController.rightStick().onTrue(Commands.runOnce(SignalLogger::stop));
//        driverController.a().whileTrue(arm.sysIdQuasistaticShoulder(SysIdRoutine.Direction.kForward));
//        driverController.b().whileTrue(arm.sysIdQuasistaticShoulder(SysIdRoutine.Direction.kReverse));
//        driverController.x().whileTrue(arm.sysIdDynamicShoulder(SysIdRoutine.Direction.kForward));
//        driverController.y().whileTrue(arm.sysIdDynamicShoulder(SysIdRoutine.Direction.kReverse));
    }

    private Command setRobotMode(RobotMode robotMode) {
        return runOnce(() -> this.robotMode = robotMode);
    }

    public void updateDashboard() {
        SmartDashboard.putBoolean("Coral Level 1", robotMode == RobotMode.CORAL_LEVEL_1);
        SmartDashboard.putBoolean("Coral Level 2", robotMode == RobotMode.CORAL_LEVEL_2);
        SmartDashboard.putBoolean("Coral Level 3", robotMode == RobotMode.CORAL_LEVEL_3);
        SmartDashboard.putBoolean("Coral Level 4", robotMode == RobotMode.CORAL_LEVEL_4);
        SmartDashboard.putBoolean("Algae in Processor", robotMode == RobotMode.ALGAE_PROCESSOR);
        SmartDashboard.putBoolean("Remove a Lower Algae", robotMode == RobotMode.ALGAE_REMOVE_LOWER);
        SmartDashboard.putBoolean("Remove an Upper Algae", robotMode == RobotMode.ALGAE_REMOVE_UPPER);
        SmartDashboard.putBoolean("Algae in Net", robotMode == RobotMode.ALGAE_NET);
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
