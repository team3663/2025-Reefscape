// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.config.RobotFactory;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.grabber.Grabber;
import frc.robot.subsystems.led.Led;
import frc.robot.utility.ControllerHelper;

import java.util.EnumMap;
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
        superStructure = new SuperStructure(elevator, arm);

        commandFactory = new CommandFactory(drivetrain, elevator, arm, grabber, climber, led, superStructure);

        configureBindings();

        drivetrain.setDefaultCommand(
                drivetrain.drive(this::getDrivetrainXVelocity, this::getDrivetrainYVelocity, this::getDrivetrainAngularVelocity)
        );
        led.setDefaultCommand(led.signalCommand(() -> robotMode));

        // Creates Auto Chooser
        autoChooser = new AutoChooser();

        // Add options to the shooter
        autoChooser.addRoutine("FacePlantG", this::facePlantG);
        autoChooser.addRoutine("FacePlantH", this::facePlantH);
        autoChooser.addRoutine("FourCoral", this::fourCoral);
        autoChooser.addRoutine("BehindTheBack", this::behindTheBack);
        autoChooser.addRoutine("FlippedBehindTheBack", this::flippedBehindTheBack);
        autoChooser.addRoutine("Flipped4Coral", this::flipped4Coral);
        autoChooser.addRoutine("FiveCoral", this::fiveCoral);
        autoChooser.addRoutine("FlippedFiveCoral", this::flippedFiveCoral);
        autoChooser.addRoutine("TwoCoralFE", this::twoCoralFE);
        autoChooser.addRoutine("TwoCoralDC", this::twoCoralDC);
        autoChooser.addRoutine("ThreeCoralEDC", this::threeCoralEDC);
        autoChooser.addRoutine("TwoCoralIJ", this::twoCoralIJ);
        autoChooser.addRoutine("TwoCoralKL", this::twoCoralKL);
        autoChooser.addRoutine("ThreeCoralJKL", this::threeCoralJKL);

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
                        facePlantGTraj.cmd()
                        )
                )
        );
        return routine;
    }

    private AutoRoutine behindTheBack(){
        AutoRoutine routine = autoFactory.newRoutine("BehindTheBack");

        AutoTrajectory Start = routine.trajectory("PStart-A");
        AutoTrajectory ADCS = routine.trajectory("A-DCS");
        AutoTrajectory DCSB = routine.trajectory("DCS-B");

        routine.active().onTrue(
                Commands.sequence(
                        Start.resetOdometry(),
                        Start.cmd()
                )
        );

        Start.done().onTrue(ADCS.cmd());
        ADCS.done().onTrue(DCSB.cmd());

        return routine;
    }

    private AutoRoutine threeCoralEDC(){
        AutoRoutine routine = autoFactory.newRoutine("threeCoralEDC");

        AutoTrajectory Start = routine.trajectory("PStart-E");
        AutoTrajectory EWCS = routine.trajectory("E-WCS");
        AutoTrajectory WCSD = routine.trajectory("WCS-D");
        AutoTrajectory DWCS = routine.trajectory("D-WCS");
        AutoTrajectory WCSC = routine.trajectory("WCS-C");

        routine.active().onTrue(
                Commands.sequence(
                        Start.resetOdometry(),
                        Start.cmd()
                )
        );

        Start.done().onTrue(EWCS.cmd());
        EWCS.done().onTrue(WCSD.cmd());
        WCSD.done().onTrue(DWCS.cmd());
        DWCS.done().onTrue(WCSC.cmd());

        return routine;
    }
    private AutoRoutine threeCoralJKL(){
        AutoRoutine routine = autoFactory.newRoutine("threeCoralEDC");

        AutoTrajectory Start = routine.trajectory("LStart-J");
        AutoTrajectory JLWCS = routine.trajectory("J-LWCS");
        AutoTrajectory LWCSK = routine.trajectory("LWCS-K");
        AutoTrajectory KLWCS = routine.trajectory("K-LWCS");
        AutoTrajectory LWCSL = routine.trajectory("LWCS-L");

        routine.active().onTrue(
                Commands.sequence(
                        Start.resetOdometry(),
                        Start.cmd()
                )
        );

        Start.done().onTrue(JLWCS.cmd());
        JLWCS.done().onTrue(LWCSK.cmd());
        LWCSK.done().onTrue(KLWCS.cmd());
        KLWCS.done().onTrue(LWCSL.cmd());

        return routine;
    }


    private AutoRoutine twoCoralFE(){
        AutoRoutine routine = autoFactory.newRoutine("twoCoralEF");

        AutoTrajectory Start = routine.trajectory("PStart-F");
        AutoTrajectory FWCS = routine.trajectory("F-WCS");
        AutoTrajectory WCSE = routine.trajectory("WCS-E");

        routine.active().onTrue(
                Commands.sequence(
                        Start.resetOdometry(),
                        Start.cmd()
                )
        );

        Start.done().onTrue(FWCS.cmd());
        FWCS.done().onTrue(WCSE.cmd());

        return routine;
    }
    private AutoRoutine twoCoralKL(){
        AutoRoutine routine = autoFactory.newRoutine("TwoCoralKL");

        AutoTrajectory Start = routine.trajectory("LStart-K");
        AutoTrajectory KLWCS = routine.trajectory("K-LWCS");
        AutoTrajectory LWCSL = routine.trajectory("LWCS-L");

        routine.active().onTrue(
                Commands.sequence(
                        Start.resetOdometry(),
                        Start.cmd()
                )
        );

        Start.done().onTrue(KLWCS.cmd());
        KLWCS.done().onTrue(LWCSL.cmd());

        return routine;
    }

    private AutoRoutine twoCoralIJ(){
        AutoRoutine routine = autoFactory.newRoutine("TwoCoralIJ");

        AutoTrajectory Start = routine.trajectory("LStart-I");
        AutoTrajectory ILWCS = routine.trajectory("I-LWCS");
        AutoTrajectory LWCSJ = routine.trajectory("LWCS-J");

        routine.active().onTrue(
                Commands.sequence(
                        Start.resetOdometry(),
                        Start.cmd()
                )
        );

        Start.done().onTrue(ILWCS.cmd());
        ILWCS.done().onTrue(LWCSJ.cmd());

        return routine;
    }

    private AutoRoutine twoCoralDC(){
        AutoRoutine routine = autoFactory.newRoutine("TwoCoralDC");

        AutoTrajectory Start = routine.trajectory("PStart-D");
        AutoTrajectory DWCS = routine.trajectory("D-WCS");
        AutoTrajectory WCSC = routine.trajectory("WCS-C");

        routine.active().onTrue(
                Commands.sequence(
                        Start.resetOdometry(),
                        Start.cmd()
                )
        );

        Start.done().onTrue(DWCS.cmd());
        DWCS.done().onTrue(WCSC.cmd());

        return routine;
    }

    private AutoRoutine flippedBehindTheBack(){
        AutoRoutine routine = autoFactory.newRoutine("FlippedBehindTheBack");

        AutoTrajectory Start = routine.trajectory("LStart-B");
        AutoTrajectory BLDCS = routine.trajectory("B-LDCS");
        AutoTrajectory LDCSA = routine.trajectory("LDCS-A");

        routine.active().onTrue(
                Commands.sequence(
                        Start.resetOdometry(),
                        Start.cmd()
                )
        );

        Start.done().onTrue(BLDCS.cmd());
        BLDCS.done().onTrue(LDCSA.cmd());

        return routine;
    }

    private AutoRoutine facePlantH(){
        AutoRoutine routine = autoFactory.newRoutine("FacePlantH");

        AutoTrajectory facePlantHTraj = routine.trajectory("FacePlantH" );

        routine.active().onTrue(
                Commands.sequence(
                        facePlantHTraj.resetOdometry(),
                        Commands.waitSeconds(2).andThen(
                                facePlantHTraj.cmd()
                        )
                )
        );
        return routine;
    }

    private AutoRoutine fourCoral(){
        AutoRoutine routine = autoFactory.newRoutine("ActualAuto");

        AutoTrajectory Start = routine.trajectory("PStart-F");
        AutoTrajectory FWCS = routine.trajectory("F-WCS");
        AutoTrajectory WCSC = routine.trajectory("WCS-C");
        AutoTrajectory CWCS = routine.trajectory("C-WCS");
        AutoTrajectory WCSD = routine.trajectory("WCS-D");
        AutoTrajectory DWCS = routine.trajectory("D-WCS");
        AutoTrajectory WCSE = routine.trajectory("WCS-E");

        routine.active().onTrue(
                Commands.sequence(
                        Start.resetOdometry(),
                        Start.cmd()
                )
        );
        Start.done().onTrue(FWCS.cmd());
        FWCS.done().onTrue(WCSC.cmd());
        WCSC.done().onTrue(CWCS.cmd());
        CWCS.done().onTrue(WCSD.cmd());
        WCSD.done().onTrue(DWCS.cmd());
        DWCS.done().onTrue(WCSE.cmd());

        return routine;
    }

    private AutoRoutine flipped4Coral(){
        AutoRoutine routine = autoFactory.newRoutine("Flipped4Coral");

        AutoTrajectory Start = routine.trajectory("LStart-I");
        AutoTrajectory ILWCS = routine.trajectory("I-LWCS");
        AutoTrajectory LWCSL = routine.trajectory("LWCS-L");
        AutoTrajectory LLWCS = routine.trajectory("L-LWCS");
        AutoTrajectory LWCSK = routine.trajectory("LWCS-K");
        AutoTrajectory KLWCS = routine.trajectory("K-LWCS");
        AutoTrajectory LWCSJ = routine.trajectory("LWCS-J");

        routine.active().onTrue(
                Commands.sequence(
                        Start.resetOdometry(),
                        Start.cmd()
                )
        );

        Start.done().onTrue(ILWCS.cmd());
        ILWCS.done().onTrue(LWCSL.cmd());
        LWCSL.done().onTrue(LLWCS.cmd());
        LLWCS.done().onTrue(LWCSK.cmd());
        LWCSK.done().onTrue(KLWCS.cmd());
        KLWCS.done().onTrue(LWCSJ.cmd());

        return routine;
    }

    private AutoRoutine fiveCoral(){
        AutoRoutine routine = autoFactory.newRoutine("FiveCoral");

        AutoTrajectory Start = routine.trajectory("PStart-F");
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
                        Start.cmd()
                )
        );
        Start.done().onTrue(FWCS.cmd());
        FWCS.done().onTrue(WCSB.cmd());
        WCSB.done().onTrue(BWCS.cmd());
        BWCS.done().onTrue(WCSC.cmd());
        WCSC.done().onTrue(CWCS.cmd());
        CWCS.done().onTrue(WCSD.cmd());
        WCSD.done().onTrue(DWCS.cmd());
        DWCS.done().onTrue(WCSE.cmd());

        return routine;
    }

    private AutoRoutine flippedFiveCoral(){
        AutoRoutine routine = autoFactory.newRoutine("FlippedFiveCoral");

        AutoTrajectory Start = routine.trajectory("LStart-I");
        AutoTrajectory ILWCS = routine.trajectory("I-LWCS");
        AutoTrajectory LWCSA = routine.trajectory("LWCS-A");
        AutoTrajectory ALWCS = routine.trajectory("A-LWCS");
        AutoTrajectory LWCSL = routine.trajectory("LWCS-L");
        AutoTrajectory LLWCS = routine.trajectory("L-LWCS");
        AutoTrajectory LWCSK = routine.trajectory("LWCS-K");
        AutoTrajectory KLWCS = routine.trajectory("K-LWCS");
        AutoTrajectory LWCSJ = routine.trajectory("LWCS-J");

        routine.active().onTrue(
                Commands.sequence(
                        Start.resetOdometry(),
                        Start.cmd()
                )
        );

        Start.done().onTrue(ILWCS.cmd());
        ILWCS.done().onTrue(LWCSA.cmd());
        LWCSA.done().onTrue(ALWCS.cmd());
        ALWCS.done().onTrue(LWCSL.cmd());
        LWCSL.done().onTrue(LLWCS.cmd());
        LLWCS.done().onTrue(LWCSK.cmd());
        LWCSK.done().onTrue(KLWCS.cmd());
        KLWCS.done().onTrue(LWCSJ.cmd());

        return routine;
    }


    private void configureBindings() {
        // Make a Robot Command map where each item in the RobotMode Enum is mapped to a command to go to the corresponding position
        Map<RobotMode, Command> robotModeCommandMap = new EnumMap<>(RobotMode.class);
        robotModeCommandMap.put(RobotMode.CORAL_LEVEL_1, commandFactory.goToL1());
        robotModeCommandMap.put(RobotMode.CORAL_LEVEL_2, commandFactory.goToL2());
        robotModeCommandMap.put(RobotMode.CORAL_LEVEL_3, commandFactory.goToL3());
        robotModeCommandMap.put(RobotMode.CORAL_LEVEL_4, commandFactory.goToL4());
        robotModeCommandMap.put(RobotMode.ALGAE_NET, commandFactory.goToNet());
        robotModeCommandMap.put(RobotMode.ALGAE_PROCESSOR, commandFactory.goToProcessor());
        robotModeCommandMap.put(RobotMode.ALGAE_REMOVE_UPPER, commandFactory.goToRemoveUpper());
        robotModeCommandMap.put(RobotMode.ALGAE_REMOVE_LOWER, commandFactory.goToRemoveLower());

        driverController.rightBumper().whileTrue(Commands.select(robotModeCommandMap, () -> this.robotMode));
        driverController.rightTrigger().and(driverController.rightBumper())
                .and(superStructure::atTargetPositions)
                .onTrue(commandFactory.releaseGamePiece());

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

        operatorController.a().onTrue(setRobotMode(RobotMode.ALGAE_PROCESSOR));
        operatorController.y().onTrue(setRobotMode(RobotMode.ALGAE_NET));
        operatorController.x().onTrue(setRobotMode(RobotMode.ALGAE_REMOVE_UPPER));
        operatorController.b().onTrue(setRobotMode(RobotMode.ALGAE_REMOVE_LOWER));

        operatorController.povUp().onTrue(setRobotMode(RobotMode.CORAL_LEVEL_4));
        operatorController.povLeft().onTrue(setRobotMode(RobotMode.CORAL_LEVEL_3));
        operatorController.povRight().onTrue(setRobotMode(RobotMode.CORAL_LEVEL_2));
        operatorController.povDown().onTrue(setRobotMode(RobotMode.CORAL_LEVEL_1));
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
