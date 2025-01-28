// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.CANdi;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
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
import frc.robot.subsystems.led.LedColor;
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
        autoChooser.addCmd("FacePlantG", this::facePlantG);

        // Getting the auto facto
        //
        // ry
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
                        Commands.print("START"),
                        autoChooser.selectedCommandScheduler(),
                        Commands.print("END")
                ));
    }

    private Command facePlantG() {
        return Commands.sequence(autoFactory.resetOdometry("FacePlantG").andThen(
                autoFactory.trajectoryCmd("FacePlantG")
        ));
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
