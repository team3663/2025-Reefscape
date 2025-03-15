package frc.robot;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drivetrain.Drivetrain;
import choreo.auto.AutoFactory;
import frc.robot.subsystems.grabber.Grabber;
import frc.robot.utility.Gamepiece;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;


public class AutoPaths {
    private final Drivetrain drivetrain;
    private final Grabber grabber;
    private final Arm arm;
    private final SuperStructure superStructure;
    private final AutoFactory autoFactory;
    private final CommandFactory commandFactory;
    private final double INTERMEDIATE_LIMIT = 0.2;


    public AutoPaths(
            Drivetrain drivetrain, Grabber grabber,
            SuperStructure superStructure, AutoFactory autoFactory, Arm arm, CommandFactory commandFactory) {
        this.drivetrain = drivetrain;
        this.grabber = grabber;
        this.superStructure = superStructure;
        this.autoFactory = autoFactory;
        this.commandFactory = commandFactory;

        this.arm = arm;
    }


    public Command pickupFromCoralStation(RobotMode robotMode,
                                          AutoTrajectory path) {
        return Commands.parallel(
                path.cmd().andThen(drivetrain.stop()),
                superStructure.followPositions(
                                () -> Math.min(robotMode.getElevatorHeight(), Constants.ArmPositions.ELEVATOR_MAX_MOVING_HEIGHT),
                                () -> MathUtil.clamp(robotMode.getShoulderAngle(),
                                        Units.degreesToRadians(90.0) - Constants.ArmPositions.SHOULDER_MAX_MOVING_OFFSET,
                                        Units.degreesToRadians(90.0) + Constants.ArmPositions.SHOULDER_MAX_MOVING_OFFSET),
                                robotMode::getWristAngle)
                        .until(path.atTimeBeforeEnd(INTERMEDIATE_LIMIT))
                        .andThen(superStructure.goToPositions(RobotMode.CORAL_STATION)
                                .alongWith(grabber.grabCoral())));
    }

    public Command placeOnReef(RobotMode robotMode,
                               AutoTrajectory path, boolean shouldZero) {
        return Commands.parallel(
                        path.cmd().andThen(drivetrain.stop()),
                        Commands.sequence(
                                shouldZero ? superStructure.zero() : Commands.none(),
                                superStructure.followPositions(
                                                () -> Math.min(robotMode.getElevatorHeight(), Constants.ArmPositions.ELEVATOR_MAX_MOVING_HEIGHT),
                                                () -> MathUtil.clamp(robotMode.getShoulderAngle(),
                                                        Units.degreesToRadians(90.0) - Constants.ArmPositions.SHOULDER_MAX_MOVING_OFFSET,
                                                        Units.degreesToRadians(90.0) + Constants.ArmPositions.SHOULDER_MAX_MOVING_OFFSET),
                                                robotMode::getWristAngle)
                                        .until(path.atTimeBeforeEnd(INTERMEDIATE_LIMIT)),
                                superStructure.goToPositions(RobotMode.CORAL_LEVEL_4)
                        ))
                .andThen(grabber.placeCoralL4().withDeadline(Commands.waitUntil(grabber::isGamePieceDetected).andThen(Commands.waitSeconds(0.25))));
    }

    public AutoRoutine facePlantD1() {
        AutoRoutine routine = autoFactory.newRoutine("FacePlant:D1");

        AutoTrajectory facePlantD1Traj = routine.trajectory("FacePlantD1");

        routine.active().onTrue(
                Commands.sequence(
                        facePlantD1Traj.resetOdometry(),
                        superStructure.zero(),
                        facePlantD1Traj.cmd()

                ));
        facePlantD1Traj.done().onTrue(superStructure.goToPositions(RobotMode.CORAL_LEVEL_4).andThen(commandFactory.placeCoral()));
        return routine;
    }

    public AutoRoutine facePlantD2() {
        AutoRoutine routine = autoFactory.newRoutine("FacePlant:D2");

        AutoTrajectory facePlantD2Traj = routine.trajectory("FacePlantD2");

        routine.active().onTrue(
                Commands.sequence(
                        facePlantD2Traj.resetOdometry(),
                        superStructure.zero(),
                        facePlantD2Traj.cmd()

                ));
        facePlantD2Traj.done().onTrue(superStructure.goToPositions(RobotMode.CORAL_LEVEL_4).andThen(commandFactory.placeCoral()));
        return routine;
    }

    public AutoRoutine twoCoralB2B1() {
        AutoRoutine routine = autoFactory.newRoutine("TwoCoral:B2-B1");

        AutoTrajectory start = routine.trajectory("RStart-B2");
        AutoTrajectory b2rs = routine.trajectory("B2-RS");
        AutoTrajectory rsb1 = routine.trajectory("RS-B1");
        AutoTrajectory b1rs = routine.trajectory("B1-RS");
        routine.active().onTrue(
                Commands.sequence(
                        start.resetOdometry(),
                        placeOnReef(RobotMode.CORAL_LEVEL_4, start, true),
                        pickupFromCoralStation(RobotMode.CORAL_LEVEL_4, b2rs),
                        placeOnReef(RobotMode.CORAL_LEVEL_4, rsb1, false),
                        pickupFromCoralStation(RobotMode.CORAL_LEVEL_4, b1rs)
                ));

        return routine;
    }

    public AutoRoutine twoCoralF1F2() {
        AutoRoutine routine = autoFactory.newRoutine("TwoCoral:F1-F2");

        AutoTrajectory start = routine.trajectory("LStart-F1");
        AutoTrajectory f1ls = routine.trajectory("F1-LS");
        AutoTrajectory lsf2 = routine.trajectory("LS-F2");
        AutoTrajectory f2ls = routine.trajectory("F2-LS");

        routine.active().onTrue(
                Commands.sequence(
                        start.resetOdometry(),
                        placeOnReef(RobotMode.CORAL_LEVEL_4, start, true),
                        pickupFromCoralStation(RobotMode.CORAL_LEVEL_4, f1ls),
                        placeOnReef(RobotMode.CORAL_LEVEL_4, lsf2, false),
                        pickupFromCoralStation(RobotMode.CORAL_LEVEL_4, f2ls)
                )
        );

        return routine;
    }
}