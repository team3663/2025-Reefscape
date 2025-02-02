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
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.config.RobotFactory;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.grabber.Grabber;
import frc.robot.subsystems.led.Led;
import frc.robot.utility.ControllerHelper;

@Logged
public class RobotContainer {
    private final Drivetrain drivetrain;
    private final Elevator elevator;
    private final Arm arm;
    private final Grabber grabber;
    private final Led led;
    private final SuperStructure superStructure;
    private final AutoFactory autoFactory;
    private final AutoChooser autoChooser;

    @NotLogged
    private final CommandXboxController driverController = new CommandXboxController(0);

    public RobotContainer(RobotFactory robotFactory) {
        drivetrain = new Drivetrain(robotFactory.createDrivetrainIo());
        elevator = new Elevator(robotFactory.createElevatorIo());
        arm = new Arm(robotFactory.createArmIo());
        grabber = new Grabber(robotFactory.createGrabberIo());
        led = new Led(robotFactory.createLedIo());
        superStructure = new SuperStructure(elevator, arm);

        configureBindings();

        drivetrain.setDefaultCommand(
                drivetrain.drive(this::getDrivetrainXVelocity, this::getDrivetrainYVelocity, this::getDrivetrainAngularVelocity)
        );

        // Creates Auto Chooser
        autoChooser = new AutoChooser();

        // Add options to the shooter
        autoChooser.addRoutine("FacePlantG", this::facePlantG);
        autoChooser.addRoutine("FacePlantH", this::facePlantH);
        autoChooser.addRoutine("4Coral", this::fourCoral);
        autoChooser.addRoutine("BehindTheBack", this::behindTheBack);
        autoChooser.addRoutine("FlippedBehindTheBack", this::flippedBehindTheBack);
        autoChooser.addRoutine("Flipped4Coral", this::flipped4Coral);

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

        // Load the routine's trajectories
        AutoTrajectory facePlantGTraj = routine.trajectory("FacePlantG");

        // When the routine begins, reset odometry and start the first trajectory
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


    private void configureBindings() {

        //Joystick Y = quasistatic forward
        driverController.y().whileTrue(drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        // Joystick A = quasistatic reverse
        driverController.a().whileTrue(drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        // Joystick B = dynamic forward
        driverController.b().whileTrue(drivetrain.sysIdDynamic(SysIdRoutine.Direction.kForward));
        //Joystick X = dynamic reverse
        driverController.x().whileTrue(drivetrain.sysIdDynamic(SysIdRoutine.Direction.kReverse));

        driverController.povUp().onTrue(Commands.run(SignalLogger::start));

        driverController.povDown().onTrue(Commands.run(SignalLogger::stop));

//        driverController.a().onTrue(superStructure.stop());
//        driverController.x().onTrue(grabber.withVoltageUntilDetected(12));
//        driverController.b().onTrue(grabber.stop());
        driverController.back().onTrue(drivetrain.resetFieldOriented());
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
