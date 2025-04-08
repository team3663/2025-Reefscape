// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import choreo.auto.AutoChooser;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
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

    private RobotMode robotModeReef = RobotMode.CORAL_LEVEL_1;
    private boolean isCSWithCoral = false;

    public RobotContainer(RobotFactory robotFactory) {
        drivetrain = new Drivetrain(robotFactory.createDrivetrainIo());
        elevator = new Elevator(robotFactory.createElevatorIo());
        arm = new Arm(robotFactory.createArmIo());
        grabber = new Grabber(robotFactory.createGrabberIo());
        climber = new Climber(robotFactory.createClimberIo());
        led = new Led(robotFactory.createLedIo());
        vision = new Vision(AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark), robotFactory.createVisionIo());
        superStructure = new SuperStructure(elevator, arm, grabber::hasAlgae);

        commandFactory = new CommandFactory(drivetrain, elevator, arm, grabber, climber, led, superStructure);
        autoPaths = new AutoPaths(drivetrain, grabber, superStructure, drivetrain.getAutoFactory(), arm, elevator);

        vision.setDefaultCommand(vision.consumeVisionMeasurements(drivetrain::addVisionMeasurements, drivetrain::getYaw,()-> robotModeReef).ignoringDisable(true));

        configureBindings();

        drivetrain.setDefaultCommand(
                drivetrain.drive(this::getDrivetrainXVelocity, this::getDrivetrainYVelocity, this::getDrivetrainAngularVelocity)
        );
        led.setDefaultCommand(led.signalCommand(() -> robotModeReef));
        superStructure.setDefaultCommand(superStructure.goToDefaultPositions());

        // Creates Auto Chooser
        autoChooser = new AutoChooser();

        // Add options to the shooter
        autoChooser.addRoutine("FacePlant:D1(L4)", autoPaths::facePlantD1L4);
        autoChooser.addRoutine("FacePlant:D2(L4)", autoPaths::facePlantD2L4);

        autoChooser.addRoutine("TwoCoral:C1(L4) B1(L4)", autoPaths::twoCoralC1L4B1L4);
        autoChooser.addRoutine("TwoCoral:E2(L4) F2(L4)", autoPaths::twoCoralE2L4F2L4);

        autoChooser.addRoutine("ThreeCoral:C1(L4) B1(L4) B2(L4)", autoPaths::threeCoralC1L4B1L4B2L4);
        autoChooser.addRoutine("ThreeCoral:E2(L4) F2(L4) F1(L4)", autoPaths::threeCoralE2L4F2L4F1L4);

        autoChooser.addRoutine("FourCoral:C1(L4) B1(L4) B2(L4) A2(L2)", autoPaths::fourCoralC1L4B1L4B2L4A2L2);
        autoChooser.addRoutine("FourCoral:E2(L4) F2(L4) F1(L4) A1(L2)", autoPaths::fourCoralE2L4F2L4F1L4A1L2);

        autoChooser.addRoutine("FourCoral:C1(L4) B1(L4) B2(L4) C1(L2)", autoPaths::fourCoralC1L4B1L4B2L4C1L2);
        autoChooser.addRoutine("FourCoral:E2(L4) F2(L4) F1(L4) E2(L2)", autoPaths::fourCoralE2L4F2L4F1L4E2L2);

        autoChooser.addRoutine("FourCoral:C1(L4) B1(L4) B1(L3) B2(L3)", autoPaths::fourCoralC1L4B1L4B1L3B2L3);
        autoChooser.addRoutine("FourCoral:E2(L4) F2(L4) F2(L3) F1(L3)", autoPaths::fourCoralE2L4F2L4F2L3F1L3);

//        autoChooser.addRoutine("FourCoral:C2(L2) B1(L4) B2(L4) C1(L2)", autoPaths::fourCoralC2L2B1L4B2L4C1L2);
//        autoChooser.addRoutine("FourCoral:E1(L2) F2(L4) F1(L4) E2(L2)", autoPaths::fourCoralE1L2F2L4F1L4E2L2);

        // Puts auto chooser on the dashboard
        SmartDashboard.putData("Auto Select", autoChooser);



        // Schedule the selected auto during the autonomous period
        RobotModeTriggers.autonomous().whileTrue(
                Commands.sequence(
                        autoChooser.selectedCommandScheduler()
                ));
        RobotModeTriggers.autonomous().or(RobotModeTriggers.teleop()).onTrue(climber.stow());

        SmartDashboard.putBoolean("Auto Reef", true);
        SmartDashboard.putBoolean("Auto Coral Station", false);



        new Trigger(() -> robotModeReef == RobotMode.CORAL_LEVEL_1)
                .whileTrue(led.setLedColor(Color.kYellow).andThen(Commands.idle(led)));
    }

    private void configureBindings() {
        driverController.rightBumper().whileTrue(commandFactory.alignToReef(() -> robotModeReef, driverController.rightTrigger(),
                driverController.leftBumper(),this::getDrivetrainXVelocity, this::getDrivetrainYVelocity, this::getDrivetrainAngularVelocity));

        driverController.leftTrigger().whileTrue(
                Commands.either(Commands.idle(), commandFactory.alignToCoralStation(() -> isCSWithCoral),
                        grabber::isGamePieceDetected));
        driverController.back().onTrue(drivetrain.resetFieldOriented());
        driverController.start().onTrue(superStructure.zero().alongWith(climber.zero()));

        driverController.b().whileTrue(grabber.eject());

        operatorController.leftBumper().whileTrue(climber.arm()
                .alongWith(superStructure.goToPositions(Constants.ArmPositions.ELEVATOR_CLIMB_POSITION,
                        Constants.ArmPositions.SHOULDER_CLIMB_ANGLE, Constants.ArmPositions.WRIST_CLIMB_ANGLE)));
        operatorController.leftBumper().onFalse(climber.stow());
        driverController.a().onFalse(climber.stow());
        driverController.y().and(operatorController.leftBumper())
                .onTrue(climber.climb()
                        .alongWith(
                                superStructure.followPositions(() -> Constants.ArmPositions.ELEVATOR_CLIMB_POSITION, () -> Constants.ArmPositions.SHOULDER_CLIMB_ANGLE, () -> Constants.ArmPositions.WRIST_CLIMB_ANGLE)
                        )
                        .until(driverController.a()).withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming));

        new Trigger(grabber::isGamePieceDetected).debounce(Constants.DEBOUNCE_TIME).onTrue(led.intakeFlash());

        // Operator Controller Robot Mode
        operatorController.a().onTrue(setRobotMode(RobotMode.ALGAE_PROCESSOR).ignoringDisable(true));
        operatorController.y().onTrue(setRobotMode(RobotMode.ALGAE_NET).ignoringDisable(true));
        operatorController.x().onTrue(setRobotMode(RobotMode.ALGAE_REMOVE_UPPER));
        operatorController.b().onTrue(setRobotMode(RobotMode.ALGAE_REMOVE_LOWER));
        operatorController.rightTrigger().onTrue(setRobotMode(RobotMode.ALGAE_PICKUP_GROUND));

        operatorController.povUp().onTrue(setRobotMode(RobotMode.CORAL_LEVEL_4));
        operatorController.povLeft().onTrue(setRobotMode(RobotMode.CORAL_LEVEL_3));
        operatorController.povRight().onTrue(setRobotMode(RobotMode.CORAL_LEVEL_2));
        operatorController.povDown().onTrue(setRobotMode(RobotMode.CORAL_LEVEL_1));

        operatorController.leftTrigger().onTrue(setCSWithCoral(true));
        operatorController.leftTrigger().onFalse(setCSWithCoral(false));

        // Test code for SysID
//        driverController.leftStick().onTrue(Commands.runOnce(SignalLogger::start));
//        driverController.rightStick().onTrue(Commands.runOnce(SignalLogger::stop));
//        driverController.a().whileTrue(arm.sysIdQuasistaticShoulder(SysIdRoutine.Direction.kForward));
//        driverController.b().whileTrue(arm.sysIdQuasistaticShoulder(SysIdRoutine.Direction.kReverse));
//        driverController.x().whileTrue(arm.sysIdDynamicShoulder(SysIdRoutine.Direction.kForward));
//        driverController.y().whileTrue(arm.sysIdDynamicShoulder(SysIdRoutine.Direction.kReverse));
    }

    private Command setRobotMode(RobotMode robotMode) {
        return runOnce(() -> this.robotModeReef = robotMode);
    }

    public Command setCSWithCoral(boolean isCSWithCoral) {
        return runOnce(() -> this.isCSWithCoral = isCSWithCoral);
    }

    public void updateDashboard() {
        SmartDashboard.putBoolean("Coral Level 1", robotModeReef == RobotMode.CORAL_LEVEL_1);
        SmartDashboard.putBoolean("Coral Level 2", robotModeReef == RobotMode.CORAL_LEVEL_2);
        SmartDashboard.putBoolean("Coral Level 3", robotModeReef == RobotMode.CORAL_LEVEL_3);
        SmartDashboard.putBoolean("Coral Level 4", robotModeReef == RobotMode.CORAL_LEVEL_4);
        SmartDashboard.putBoolean("Algae in Processor", robotModeReef == RobotMode.ALGAE_PROCESSOR);
        SmartDashboard.putBoolean("Remove a Lower Algae", robotModeReef == RobotMode.ALGAE_REMOVE_LOWER);
        SmartDashboard.putBoolean("Remove an Upper Algae", robotModeReef == RobotMode.ALGAE_REMOVE_UPPER);
        SmartDashboard.putBoolean("Algae in Net", robotModeReef == RobotMode.ALGAE_NET);
        SmartDashboard.putBoolean("Algae from Ground", robotModeReef == RobotMode.ALGAE_PICKUP_GROUND);
        SmartDashboard.putBoolean("CS With Coral", isCSWithCoral);
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
