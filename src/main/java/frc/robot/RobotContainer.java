// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
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
    //private final Vision vision;
    private final SuperStructure superStructure;
    private final AutoFactory autoFactory;
    private final AutoChooser autoChooser;

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
        //vision = new Vision(AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape), robotFactory.createVisionIo());
        superStructure = new SuperStructure(elevator, arm);

        commandFactory = new CommandFactory(drivetrain, elevator, arm, grabber, climber, led, superStructure);

        //vision.setDefaultCommand(vision.consumeVisionMeasurements(drivetrain::addVisionMeasurements).ignoringDisable(true));

        configureBindings();

        drivetrain.setDefaultCommand(
                drivetrain.drive(this::getDrivetrainXVelocity, this::getDrivetrainYVelocity, this::getDrivetrainAngularVelocity)
        );
        led.setDefaultCommand(led.signalCommand(() -> robotMode));
        superStructure.setDefaultCommand(superStructure.goToDefaultPositions());

        // Creates Auto Chooser
        autoChooser = new AutoChooser();

        // Add options to the shooter
        autoChooser.addRoutine("FacePlantG", this::facePlantG);
        autoChooser.addRoutine("FacePlantH", this::facePlantH);

        autoChooser.addRoutine("TwoCoralDC", this::twoCoralDC);
        autoChooser.addRoutine("TwoCoralFE", this::twoCoralFE);
        autoChooser.addRoutine("TwoCoralIJ", this::twoCoralIJ);
        autoChooser.addRoutine("TwoCoralKL", this::twoCoralKL);

        autoChooser.addRoutine("BehindTheBackAB", this::behindTheBackAB);
        autoChooser.addRoutine("BehindTheBackFlippedBA", this::behindTheBackFlippedBA);

        autoChooser.addRoutine("ThreeCoralEDC", this::threeCoralEDC);
        autoChooser.addRoutine("ThreeCoralGFE", this::threeCoralGFE);
        autoChooser.addRoutine("ThreeCoralJKL", this::threeCoralJKL);

        autoChooser.addRoutine("FourCoralFCDE", this::fourCoralFCDE);
        autoChooser.addRoutine("FourCoralFlippedILKJ", this::fourCoralFlippedILKJ);

        autoChooser.addRoutine("FiveCoralFBCDE", this::fiveCoralFBCDE);
        autoChooser.addRoutine("FiveCoralFlippedIALKJ", this::fiveCoralFlippedIALKJ);

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

    private AutoRoutine facePlantG() {
        AutoRoutine routine = autoFactory.newRoutine("FacePlantG");

        AutoTrajectory facePlantGTraj = routine.trajectory("FacePlantG");

        routine.active().onTrue(
                Commands.sequence(
                        facePlantGTraj.resetOdometry(),
                        Commands.waitSeconds(2).andThen(
                                Commands.parallel(
                                        facePlantGTraj.cmd(),
                                        superStructure.followPositions(() -> RobotMode.CORAL_LEVEL_4))
                        )
                )
        );
        return routine;
    }

    private AutoRoutine facePlantH() {
        AutoRoutine routine = autoFactory.newRoutine("FacePlantH");

        AutoTrajectory facePlantHTraj = routine.trajectory("FacePlantH");

        routine.active().onTrue(
                Commands.sequence(
                        facePlantHTraj.resetOdometry(),
                        Commands.waitSeconds(2).andThen(Commands.parallel(
                                facePlantHTraj.cmd(), superStructure.followPositions(() -> RobotMode.CORAL_LEVEL_4))
                        )
                )
        );

        return routine;
    }

    private AutoRoutine twoCoralDC() {
        AutoRoutine routine = autoFactory.newRoutine("TwoCoralDC");

        AutoTrajectory Start = routine.trajectory("PStart-D");
        AutoTrajectory DWCS = routine.trajectory("D-WCS");
        AutoTrajectory WCSC = routine.trajectory("WCS-C");

        routine.active().onTrue(
                Commands.sequence(
                        Start.resetOdometry(),
                        superStructure.followPositions(() -> RobotMode.CORAL_LEVEL_4),
                        Start.cmd()
                )
        );

        Start.done().onTrue(Commands.parallel(DWCS.cmd(), superStructure.followPositions(() -> RobotMode.CORAL_STATION)));
        DWCS.done().onTrue(Commands.parallel(WCSC.cmd(), superStructure.followPositions(() -> RobotMode.CORAL_LEVEL_4)));

        return routine;
    }

    private AutoRoutine twoCoralFE() {
        AutoRoutine routine = autoFactory.newRoutine("TwoCoralFE");

        AutoTrajectory Start = routine.trajectory("PStart-F");
        AutoTrajectory FWCS = routine.trajectory("F-WCS");
        AutoTrajectory WCSE = routine.trajectory("WCS-E");

        routine.active().onTrue(
                Commands.sequence(
                        Start.resetOdometry(),
                        Commands.parallel(
                                Start.cmd(),
                                superStructure.followPositions(() -> RobotMode.CORAL_LEVEL_4))
                )
        );

        Start.done().onTrue(Commands.parallel(FWCS.cmd(), superStructure.followPositions(()-> RobotMode.CORAL_STATION)));
        FWCS.done().onTrue(Commands.parallel(WCSE.cmd(), superStructure.followPositions(()-> RobotMode.CORAL_LEVEL_4)));

        return routine;
    }

    private AutoRoutine twoCoralIJ() {
        AutoRoutine routine = autoFactory.newRoutine("TwoCoralIJ");

        AutoTrajectory Start = routine.trajectory("LStart-I");
        AutoTrajectory ILWCS = routine.trajectory("I-LWCS");
        AutoTrajectory LWCSJ = routine.trajectory("LWCS-J");

        routine.active().onTrue(
                Commands.sequence(
                        Start.resetOdometry(),
                        superStructure.followPositions(() -> RobotMode.CORAL_LEVEL_4),
                        Start.cmd()
                )
        );

        Start.done().onTrue(Commands.parallel(ILWCS.cmd(), superStructure.followPositions(() -> RobotMode.CORAL_STATION)));
        ILWCS.done().onTrue(Commands.parallel(LWCSJ.cmd(), superStructure.followPositions(() -> RobotMode.CORAL_LEVEL_4)));

        return routine;
    }

    private AutoRoutine twoCoralKL() {
        AutoRoutine routine = autoFactory.newRoutine("TwoCoralKL");

        AutoTrajectory Start = routine.trajectory("LStart-K");
        AutoTrajectory KLWCS = routine.trajectory("K-LWCS");
        AutoTrajectory LWCSL = routine.trajectory("LWCS-L");

        routine.active().onTrue(
                Commands.sequence(
                        Start.resetOdometry(),
                        superStructure.followPositions(() -> RobotMode.CORAL_LEVEL_4),
                        Start.cmd()
                )
        );

        Start.done().onTrue(Commands.parallel(KLWCS.cmd(), superStructure.followPositions(() -> RobotMode.CORAL_STATION)));
        KLWCS.done().onTrue(Commands.parallel(LWCSL.cmd(), superStructure.followPositions(() -> RobotMode.CORAL_LEVEL_4)));

        return routine;
    }

    private AutoRoutine behindTheBackAB() {
        AutoRoutine routine = autoFactory.newRoutine("BehindTheBackAB");

        AutoTrajectory Start = routine.trajectory("PStart-A");
        AutoTrajectory ADCS = routine.trajectory("A-DCS");
        AutoTrajectory DCSB = routine.trajectory("DCS-B");

        routine.active().onTrue(
                Commands.sequence(
                        Start.resetOdometry(),
                        Commands.parallel(
                                Start.cmd(),
                                superStructure.followPositions(() -> RobotMode.CORAL_LEVEL_4))
                )
        );

        Start.done().onTrue(Commands.parallel(ADCS.cmd(), superStructure.followPositions(() -> RobotMode.CORAL_STATION)));
        ADCS.done().onTrue(Commands.parallel(DCSB.cmd(), superStructure.followPositions(() -> RobotMode.CORAL_LEVEL_4)));

        return routine;
    }

    private AutoRoutine behindTheBackFlippedBA() {
        AutoRoutine routine = autoFactory.newRoutine("BehindTheBackFlippedBA");

        AutoTrajectory Start = routine.trajectory("LStart-B");
        AutoTrajectory BLDCS = routine.trajectory("B-LDCS");
        AutoTrajectory LDCSA = routine.trajectory("LDCS-A");

        routine.active().onTrue(
                Commands.sequence(
                        Start.resetOdometry(),
                        superStructure.followPositions(() -> RobotMode.CORAL_LEVEL_4),
                        Start.cmd()
                )
        );

        Start.done().onTrue(Commands.parallel(BLDCS.cmd(), superStructure.followPositions(() -> RobotMode.CORAL_STATION)));
        BLDCS.done().onTrue(Commands.parallel(LDCSA.cmd(), superStructure.followPositions(() -> RobotMode.CORAL_LEVEL_4)));

        return routine;
    }

    private AutoRoutine threeCoralEDC() {
        AutoRoutine routine = autoFactory.newRoutine("ThreeCoralEDC");

        AutoTrajectory Start = routine.trajectory("PStart-E");
        AutoTrajectory EWCS = routine.trajectory("E-WCS");
        AutoTrajectory WCSD = routine.trajectory("WCS-D");
        AutoTrajectory DWCS = routine.trajectory("D-WCS");
        AutoTrajectory WCSC = routine.trajectory("WCS-C");

        routine.active().onTrue(
                Commands.sequence(
                        Start.resetOdometry(),
                        Commands.parallel(
                                Start.cmd(),
                                superStructure.followPositions(() -> RobotMode.CORAL_LEVEL_4))
                )
        );

        Start.done().onTrue(Commands.parallel(EWCS.cmd(), superStructure.followPositions(() -> RobotMode.CORAL_STATION)));
        EWCS.done().onTrue(Commands.parallel(WCSD.cmd(), superStructure.followPositions(() -> RobotMode.CORAL_LEVEL_4)));
        WCSD.done().onTrue(Commands.parallel(DWCS.cmd(), superStructure.followPositions(() -> RobotMode.CORAL_STATION)));
        DWCS.done().onTrue(Commands.parallel(WCSC.cmd(), superStructure.followPositions(() -> RobotMode.CORAL_LEVEL_4)));

        return routine;
    }

    private AutoRoutine threeCoralGFE() {
        AutoRoutine routine = autoFactory.newRoutine("ThreeCoralGFE");

        AutoTrajectory Start = routine.trajectory("FacePlantG");
        AutoTrajectory GWCS = routine.trajectory("G-WCS");
        AutoTrajectory WCSF = routine.trajectory("WCS-F");
        AutoTrajectory FWCS = routine.trajectory("F-WCS");
        AutoTrajectory WCSE = routine.trajectory("WCS-E");

        routine.active().onTrue(
                Commands.sequence(
                        Start.resetOdometry(),
                        superStructure.followPositions(() -> RobotMode.CORAL_LEVEL_4),
                        Start.cmd()
                )
        );

        Start.done().onTrue(Commands.parallel(GWCS.cmd(), superStructure.followPositions(() -> RobotMode.CORAL_STATION)));
        GWCS.done().onTrue(Commands.parallel(WCSF.cmd(), superStructure.followPositions(() -> RobotMode.CORAL_LEVEL_4)));
        WCSF.done().onTrue(Commands.parallel(FWCS.cmd(), superStructure.followPositions(() -> RobotMode.CORAL_STATION)));
        FWCS.done().onTrue(Commands.parallel(WCSE.cmd(), superStructure.followPositions(() -> RobotMode.CORAL_LEVEL_4)));

        return routine;
    }

    private AutoRoutine threeCoralJKL() {
        AutoRoutine routine = autoFactory.newRoutine("ThreeCoralJKL");

        AutoTrajectory Start = routine.trajectory("LStart-J");
        AutoTrajectory JLWCS = routine.trajectory("J-LWCS");
        AutoTrajectory LWCSK = routine.trajectory("LWCS-K");
        AutoTrajectory KLWCS = routine.trajectory("K-LWCS");
        AutoTrajectory LWCSL = routine.trajectory("LWCS-L");

        routine.active().onTrue(
                Commands.sequence(
                        Start.resetOdometry(),
                        Commands.parallel(
                                Start.cmd(),
                                superStructure.followPositions(() -> RobotMode.CORAL_LEVEL_4))
                )
        );

        Start.done().onTrue(Commands.parallel(JLWCS.cmd(), superStructure.followPositions(() -> RobotMode.CORAL_STATION)));
        JLWCS.done().onTrue(Commands.parallel(LWCSK.cmd(), superStructure.followPositions(() -> RobotMode.CORAL_LEVEL_4)));
        LWCSK.done().onTrue(Commands.parallel(KLWCS.cmd(), superStructure.followPositions(() -> RobotMode.CORAL_STATION)));
        KLWCS.done().onTrue(Commands.parallel(LWCSL.cmd(), superStructure.followPositions(() -> RobotMode.CORAL_LEVEL_4)));

        return routine;
    }

    private AutoRoutine fourCoralFCDE() {
        AutoRoutine routine = autoFactory.newRoutine("FourCoralFCDE");

        AutoTrajectory Start = routine.trajectory("POStart-F");
        AutoTrajectory FWCS = routine.trajectory("F-WCS");
        AutoTrajectory WCSC = routine.trajectory("WCS-C");
        AutoTrajectory CWCS = routine.trajectory("C-WCS");
        AutoTrajectory WCSD = routine.trajectory("WCS-D");
        AutoTrajectory DWCS = routine.trajectory("D-WCS");
        AutoTrajectory WCSE = routine.trajectory("WCS-E");

        routine.active().onTrue(
                Commands.sequence(
                        Start.resetOdometry(),
                        superStructure.followPositions(() -> RobotMode.CORAL_LEVEL_4),
                        Start.cmd()
                )
        );

        Start.done().onTrue(Commands.parallel(FWCS.cmd(), superStructure.followPositions(() -> RobotMode.CORAL_STATION)));
        FWCS.done().onTrue(Commands.parallel(WCSC.cmd(), superStructure.followPositions(() -> RobotMode.CORAL_LEVEL_4)));
        WCSC.done().onTrue(Commands.parallel(CWCS.cmd(), superStructure.followPositions(() -> RobotMode.CORAL_STATION)));
        CWCS.done().onTrue(Commands.parallel(WCSD.cmd(), superStructure.followPositions(() -> RobotMode.CORAL_LEVEL_4)));
        WCSD.done().onTrue(Commands.parallel(DWCS.cmd(), superStructure.followPositions(() -> RobotMode.CORAL_STATION)));
        DWCS.done().onTrue(Commands.parallel(WCSE.cmd(), superStructure.followPositions(() -> RobotMode.CORAL_LEVEL_4)));

        return routine;
    }

    private AutoRoutine fourCoralFlippedILKJ() {
        AutoRoutine routine = autoFactory.newRoutine("FourCoralFlippedILKJ");

        AutoTrajectory Start = routine.trajectory("LOStart-I");
        AutoTrajectory ILWCS = routine.trajectory("I-LWCS");
        AutoTrajectory LWCSL = routine.trajectory("LWCS-L");
        AutoTrajectory LLWCS = routine.trajectory("L-LWCS");
        AutoTrajectory LWCSK = routine.trajectory("LWCS-K");
        AutoTrajectory KLWCS = routine.trajectory("K-LWCS");
        AutoTrajectory LWCSJ = routine.trajectory("LWCS-J");

        routine.active().onTrue(
                Commands.sequence(
                        Start.resetOdometry(),
                        superStructure.followPositions(() -> RobotMode.CORAL_LEVEL_4),
                        Start.cmd()
                )
        );

        Start.done().onTrue(Commands.parallel(ILWCS.cmd(), superStructure.followPositions(() -> RobotMode.CORAL_STATION)));
        ILWCS.done().onTrue(Commands.parallel(LWCSL.cmd(), superStructure.followPositions(() -> RobotMode.CORAL_LEVEL_4)));
        LWCSL.done().onTrue(Commands.parallel(LLWCS.cmd(), superStructure.followPositions(() -> RobotMode.CORAL_STATION)));
        LLWCS.done().onTrue(Commands.parallel(LWCSK.cmd(), superStructure.followPositions(() -> RobotMode.CORAL_LEVEL_4)));
        LWCSK.done().onTrue(Commands.parallel(KLWCS.cmd(), superStructure.followPositions(() -> RobotMode.CORAL_STATION)));
        KLWCS.done().onTrue(Commands.parallel(LWCSJ.cmd(), superStructure.followPositions(() -> RobotMode.CORAL_LEVEL_4)));

        return routine;
    }

    private AutoRoutine fiveCoralFBCDE() {
        AutoRoutine routine = autoFactory.newRoutine("FiveCoralFBCDE");

        AutoTrajectory Start = routine.trajectory("POStart-F");
        AutoTrajectory FWCS = routine.trajectory("F-WCS");
        AutoTrajectory WCSB = routine.trajectory("WCS-B");
        AutoTrajectory BWCS = routine.trajectory("B-WCS");
        AutoTrajectory WCSC = routine.trajectory("WCS-C");
        AutoTrajectory CWCS = routine.trajectory("C-WCS");
        AutoTrajectory WCSD = routine.trajectory("WCS-D");
        AutoTrajectory DWCS = routine.trajectory("D-WCS");
        AutoTrajectory WCSE = routine.trajectory("WCS-E");

        routine.active().onTrue(
                Commands.sequence(
                        Start.resetOdometry(),
                        superStructure.followPositions(() -> RobotMode.CORAL_LEVEL_4),
                        Start.cmd()
                )
        );

        Start.done().onTrue(Commands.parallel(FWCS.cmd(), superStructure.followPositions(() -> RobotMode.CORAL_STATION)));
        FWCS.done().onTrue(Commands.parallel(WCSB.cmd(), superStructure.followPositions(() -> RobotMode.CORAL_LEVEL_4)));
        WCSB.done().onTrue(Commands.parallel(BWCS.cmd(), superStructure.followPositions(() -> RobotMode.CORAL_STATION)));
        BWCS.done().onTrue(Commands.parallel(WCSC.cmd(), superStructure.followPositions(() -> RobotMode.CORAL_LEVEL_4)));
        WCSC.done().onTrue(Commands.parallel(CWCS.cmd(), superStructure.followPositions(() -> RobotMode.CORAL_STATION)));
        CWCS.done().onTrue(Commands.parallel(WCSD.cmd(), superStructure.followPositions(() -> RobotMode.CORAL_LEVEL_4)));
        WCSD.done().onTrue(Commands.parallel(DWCS.cmd(), superStructure.followPositions(() -> RobotMode.CORAL_STATION)));
        DWCS.done().onTrue(Commands.parallel(WCSE.cmd(), superStructure.followPositions(() -> RobotMode.CORAL_LEVEL_4)));

        return routine;
    }

    private AutoRoutine fiveCoralFlippedIALKJ() {
        AutoRoutine routine = autoFactory.newRoutine("FiveCoralFlippedIALKJ");

        AutoTrajectory Start = routine.trajectory("LOStart-I");
        AutoTrajectory ILWCS = routine.trajectory("I-LWCS");
        AutoTrajectory LWCSA = routine.trajectory("LWCS-A");
        AutoTrajectory ALWCS = routine.trajectory("A-LWCS");
        AutoTrajectory LWCSL = routine.trajectory("LWCS-L");
        AutoTrajectory LLWCS = routine.trajectory("L-LWCS");
        AutoTrajectory LWCSK = routine.trajectory("LWCS-K");
        AutoTrajectory KLWCS = routine.trajectory("K-LWCS");
        AutoTrajectory LWCSJ = routine.trajectory("LWCS-J");

        setRobotMode(RobotMode.CORAL_LEVEL_4);

        routine.active().onTrue(
                Commands.sequence(
                        Start.resetOdometry(),
                        superStructure.followPositions(() -> RobotMode.CORAL_LEVEL_4),
                        Start.cmd()
                )
        );

        Start.done().onTrue(Commands.parallel(ILWCS.cmd(), superStructure.followPositions(() -> RobotMode.CORAL_STATION)));
        ILWCS.done().onTrue(Commands.parallel(LWCSA.cmd(), superStructure.followPositions(() -> RobotMode.CORAL_LEVEL_4)));
        LWCSA.done().onTrue(Commands.parallel(ALWCS.cmd(), superStructure.followPositions(() -> RobotMode.CORAL_STATION)));
        ALWCS.done().onTrue(Commands.parallel(LWCSL.cmd(), superStructure.followPositions(() -> RobotMode.CORAL_LEVEL_4)));
        LWCSL.done().onTrue(Commands.parallel(LLWCS.cmd(), superStructure.followPositions(() -> RobotMode.CORAL_STATION)));
        LLWCS.done().onTrue(Commands.parallel(LWCSK.cmd(), superStructure.followPositions(() -> RobotMode.CORAL_LEVEL_4)));
        LWCSK.done().onTrue(Commands.parallel(KLWCS.cmd(), superStructure.followPositions(() -> RobotMode.CORAL_STATION)));
        KLWCS.done().onTrue(Commands.parallel(LWCSJ.cmd(), superStructure.followPositions(() -> RobotMode.CORAL_LEVEL_4)));

        return routine;
    }

    private void configureBindings() {
        driverController.rightBumper().whileTrue(superStructure.followPositions(() -> robotMode));
        driverController.rightTrigger().and(driverController.rightBumper())
                .and(superStructure::atTargetPositions)
                .whileTrue(commandFactory.releaseGamePiece());

        driverController.leftBumper().whileTrue(commandFactory.goToCoralStationAndIntake());
        driverController.back().onTrue(drivetrain.resetFieldOriented());
        driverController.start().onTrue(Commands.parallel(arm.zeroWrist(), elevator.zero(), climber.zero()));

        operatorController.leftBumper().onTrue(climber.deploy());
        operatorController.rightBumper().onTrue(climber.climb());

        new Trigger(grabber::getGamePieceDetected).debounce(0.5).onTrue(led.intakeFlash());

        // Operator Controller Robot Mode
        operatorController.a().onTrue(setRobotMode(RobotMode.ALGAE_PROCESSOR));
        operatorController.y().onTrue(setRobotMode(RobotMode.ALGAE_NET));
        operatorController.x().onTrue(setRobotMode(RobotMode.ALGAE_REMOVE_UPPER));
        operatorController.b().onTrue(setRobotMode(RobotMode.ALGAE_REMOVE_LOWER));

        operatorController.povUp().onTrue(setRobotMode(RobotMode.CORAL_LEVEL_4));
        operatorController.povLeft().onTrue(setRobotMode(RobotMode.CORAL_LEVEL_3));
        operatorController.povRight().onTrue(setRobotMode(RobotMode.CORAL_LEVEL_2));
        operatorController.povDown().onTrue(setRobotMode(RobotMode.CORAL_LEVEL_1));

        // Driver Controller Robot Mode
        driverController.a().onTrue(setRobotMode(RobotMode.ALGAE_PROCESSOR));
        driverController.y().onTrue(setRobotMode(RobotMode.ALGAE_NET));
        driverController.x().onTrue(setRobotMode(RobotMode.ALGAE_REMOVE_UPPER));
        driverController.b().onTrue(setRobotMode(RobotMode.ALGAE_REMOVE_LOWER));

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
