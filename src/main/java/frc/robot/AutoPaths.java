package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.grabber.Grabber;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class AutoPaths {
    private final Drivetrain drivetrain;
    private final Grabber grabber;
    private final Arm arm;
    private final Elevator elevator;
    private final SuperStructure superStructure;
    private final AutoFactory autoFactory;
    private final CommandFactory commandFactory;


    public AutoPaths(
            Drivetrain drivetrain, Grabber grabber,
            SuperStructure superStructure, AutoFactory autoFactory, Arm arm, CommandFactory commandFactory, Elevator elevator) {
        this.drivetrain = drivetrain;
        this.grabber = grabber;
        this.superStructure = superStructure;
        this.autoFactory = autoFactory;
        this.commandFactory = commandFactory;
        this.elevator = elevator;
        this.arm = arm;
    }

    public Command limitedArm(RobotMode robotMode) {
        return superStructure.followPositions(
                () -> Math.min(robotMode.getElevatorHeight(), Constants.ArmPositions.ELEVATOR_MAX_MOVING_HEIGHT),
                () -> MathUtil.clamp(robotMode.getShoulderAngle(),
                        Units.degreesToRadians(90.0) - Constants.ArmPositions.SHOULDER_MAX_MOVING_OFFSET,
                        Units.degreesToRadians(90.0) + Constants.ArmPositions.SHOULDER_MAX_MOVING_OFFSET),
                () -> (robotMode.getWristAngle() + Constants.ArmPositions.WRIST_MOVING_OFFSET));
    }

    public Command pickupFromCoralStation(AutoTrajectory path) {
        return
                Commands.parallel(
                        path.cmd().andThen(drivetrain.stop()),
                        limitedArm(RobotMode.CORAL_STATION)
                                .until(path.atTimeBeforeEnd(1.0))
                                .andThen(superStructure.goToPositions(RobotMode.CORAL_STATION)
                                        .alongWith(grabber.grabCoral())));
    }

    public Command pickupFromCoralStation(Pose2d branch, Pose2d intermediatePose) {
        Pose2d[] intermediateHolder = new Pose2d[]{null};

        return drivetrain.goToPosition(() -> {
                    if (intermediateHolder[0] != null && drivetrain.getPose().getTranslation().getDistance(
                            intermediateHolder[0].getTranslation()) > Units.feetToMeters(5.0)) {
                        return intermediateHolder[0];
                    }
                    intermediateHolder[0] = null;
                    return branch;
                }, () -> false, () -> getMaxVelocity(elevator.getPosition()))
                .withDeadline(
                        Commands.sequence(
                                limitedArm(RobotMode.CORAL_STATION)
                                        .until(() -> drivetrain.getPose().getTranslation().getDistance(
                                                branch.getTranslation()
                                        ) < Units.feetToMeters(3.0)),
                                superStructure.goToPositions(RobotMode.CORAL_STATION)
                                        .alongWith(grabber.grabCoral())))
                .beforeStarting(() -> intermediateHolder[0] = intermediatePose);
    }

    public Command pickupFromCoralStation(Pose2d branch) {
        return pickupFromCoralStation(branch, null);
    }

    public Command placeOnReef(AutoTrajectory path, boolean shouldZero) {
        return Commands.parallel(
                        path.cmd().andThen(drivetrain.stop()),
                        Commands.sequence(
                                shouldZero ? superStructure.zero() : Commands.none(),
                                limitedArm(RobotMode.CORAL_LEVEL_4)
                                        .until(path.atTimeBeforeEnd(0.5)),
                                superStructure.goToPositions(RobotMode.CORAL_LEVEL_4)
                        ))
                .andThen(grabber.placeCoralL4().withDeadline(Commands.waitUntil(grabber::getGamePieceNotDetected)
                        .andThen(Commands.waitSeconds(0.05))));
    }

    public Command placeOnReef(Pose2d branch, boolean shouldZero) {
        return drivetrain.goToPosition(() -> branch, () -> false,
                        () -> getMaxVelocity(elevator.getPosition()))
                .withDeadline(
                        Commands.sequence(
                                shouldZero ? superStructure.zero() : Commands.none(),
                                limitedArm(RobotMode.CORAL_LEVEL_4)
                                        .until(() -> drivetrain.getPose().getTranslation().getDistance(
                                                branch.getTranslation()
                                        ) < Units.feetToMeters(3.0)),
                                superStructure.goToPositions(RobotMode.CORAL_LEVEL_4),
                                Commands.waitUntil(() -> drivetrain.getPose().getTranslation().getDistance(
                                        branch.getTranslation()) < Units.inchesToMeters(2.0))))
                .andThen(grabber.placeCoralL4().withDeadline(Commands.waitUntil(grabber::getGamePieceNotDetected)
//                        .andThen(Commands.waitSeconds(0.25))
                ));
    }

    public Command placeOnReefl2(AutoTrajectory path, boolean shouldZero) {
        return Commands.parallel(
                        path.cmd().andThen(drivetrain.stop()),
                        Commands.sequence(
                                shouldZero ? superStructure.zero() : Commands.none(),
                                limitedArm(RobotMode.CORAL_LEVEL_2)
                                        .until(path.atTimeBeforeEnd(0.6)),
                                superStructure.goToPositions(RobotMode.CORAL_LEVEL_2)
                        ))
                .andThen(grabber.placeCoralL4().withDeadline(Commands.waitUntil(grabber::getGamePieceNotDetected).andThen(Commands.waitSeconds(0.15))));
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
                        placeOnReef(start, true),
                        pickupFromCoralStation(b2rs),
                        placeOnReef(rsb1, false),
                        pickupFromCoralStation(b1rs)
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
                        placeOnReef(start, true),
                        pickupFromCoralStation(f1ls),
                        placeOnReef(lsf2, false),
                        pickupFromCoralStation(f2ls)
                )
        );

        return routine;
    }

    public AutoRoutine threeCoralC1B1B2() {
        AutoRoutine routine = autoFactory.newRoutine("ThreeCoral:C1-B1-B2");

        AutoTrajectory start = routine.trajectory("RStart-C1");
        AutoTrajectory c1rs = routine.trajectory("C1-RS");
        AutoTrajectory rsb1 = routine.trajectory("RS-B1");
        AutoTrajectory b1rs = routine.trajectory("B1-RS");
        AutoTrajectory rsb2 = routine.trajectory("RS-B2");
        AutoTrajectory b2rs = routine.trajectory("B2-RS");
        routine.active().onTrue(
                Commands.sequence(
                        start.resetOdometry(),
                        placeOnReef(start, true),
                        pickupFromCoralStation(c1rs),
                        placeOnReef(rsb1, false),
                        pickupFromCoralStation(b1rs),
                        placeOnReef(rsb2, false),
                        pickupFromCoralStation(b2rs)
                )
        );

        return routine;
    }

    public AutoRoutine threeCoralC17B1B2() {
        AutoRoutine routine = autoFactory.newRoutine("ThreeCoral:C17-B1-B2");

        AutoTrajectory start = routine.trajectory("R7Start-C1");
        AutoTrajectory c1rs = routine.trajectory("C1-RS");
        AutoTrajectory rsb1 = routine.trajectory("RS-B1");
        AutoTrajectory b1rs = routine.trajectory("B1-RS");
        AutoTrajectory rsb2 = routine.trajectory("RS-B2");
        AutoTrajectory b2rs = routine.trajectory("B2-RS");
        routine.active().onTrue(
                Commands.sequence(
                        start.resetOdometry(),
                        placeOnReef(start, true),
                        pickupFromCoralStation(c1rs),
                        placeOnReef(rsb1, false),
                        pickupFromCoralStation(b1rs),
                        placeOnReef(rsb2, false),
                        pickupFromCoralStation(b2rs)
                )
        );

        return routine;
    }

    public AutoRoutine threeCoralPIDC1B1B2() {
        AutoRoutine routine = autoFactory.newRoutine("ThreeCoralPID:C1-B1-B2");

        routine.active().onTrue(
                Commands.sequence(
                        drivetrain.resetOdometry(getAllianceRed() ? Constants.RED_AUTO_RIGHT_STARTING_POSITION_7FT : Constants.BLUE_AUTO_RIGHT_STARTING_POSITION_7FT),
                        placeOnReef(getAllianceRed() ? Constants.RED_BRANCH_C1 : Constants.BLUE_BRANCH_C1, true),
                        pickupFromCoralStation(getAllianceRed() ? Constants.RED_RIGHT_FAR_SIDE_CORAL_STATION : Constants.BLUE_RIGHT_FAR_SIDE_CORAL_STATION,
                                getAllianceRed() ? Constants.RED_RIGHT_INTERMEDIATE : Constants.BLUE_RIGHT_INTERMEDIATE),
                        placeOnReef(getAllianceRed() ? Constants.RED_BRANCH_B1 : Constants.BLUE_BRANCH_B1, false),
                        pickupFromCoralStation(getAllianceRed() ? Constants.RED_RIGHT_FAR_SIDE_CORAL_STATION : Constants.BLUE_RIGHT_FAR_SIDE_CORAL_STATION),
                        placeOnReef(getAllianceRed() ? Constants.RED_BRANCH_B2 : Constants.BLUE_BRANCH_B2, false),
                        pickupFromCoralStation(getAllianceRed() ? Constants.RED_RIGHT_FAR_SIDE_CORAL_STATION : Constants.BLUE_RIGHT_FAR_SIDE_CORAL_STATION)
                )
        );

        return routine;
    }

    public AutoRoutine threeCoralE2F2F1() {
        AutoRoutine routine = autoFactory.newRoutine("ThreeCoral:E2-F2-F1");

        AutoTrajectory start = routine.trajectory("LStart-E2");
        AutoTrajectory e2ls = routine.trajectory("E2-LS");
        AutoTrajectory lsf2 = routine.trajectory("LS-F2");
        AutoTrajectory f2ls = routine.trajectory("F2-LS");
        AutoTrajectory lsf1 = routine.trajectory("LS-F1");
        AutoTrajectory f1ls = routine.trajectory("F1-LS");
        routine.active().onTrue(
                Commands.sequence(
                        start.resetOdometry(),
                        placeOnReef(start, true),
                        pickupFromCoralStation(e2ls),
                        placeOnReef(lsf2, false),
                        pickupFromCoralStation(f2ls),
                        placeOnReef(lsf1, false),
                        pickupFromCoralStation(f1ls)
                ));

        return routine;
    }

    public AutoRoutine fourCoralE2F2F1A1() {
        AutoRoutine routine = autoFactory.newRoutine("ThreeCoral:E2-F2-F1");

        AutoTrajectory start = routine.trajectory("LStart-E2");
        AutoTrajectory e2ls = routine.trajectory("E2-LS");
        AutoTrajectory lsf2 = routine.trajectory("LS-F2");
        AutoTrajectory f2ls = routine.trajectory("F2-LS");
        AutoTrajectory lsf1 = routine.trajectory("LS-F1");
        AutoTrajectory f1ls = routine.trajectory("F1-LS");
        AutoTrajectory lsA1 = routine.trajectory("LS-A1");

        routine.active().onTrue(
                Commands.sequence(
                        start.resetOdometry(),
                        placeOnReef(start, true),
                        pickupFromCoralStation(e2ls),
                        placeOnReef(lsf2, false),
                        pickupFromCoralStation(f2ls),
                        placeOnReef(lsf1, false),
                        pickupFromCoralStation(f1ls),
                        placeOnReefl2(lsA1, false)
                ));

        return routine;
    }

    public AutoRoutine fourCoralC1B1B2A2() {
        AutoRoutine routine = autoFactory.newRoutine("ThreeCoral:C1-B1-B2");

        AutoTrajectory start = routine.trajectory("RStart-C1");
        AutoTrajectory c1rs = routine.trajectory("C1-RS");
        AutoTrajectory rsb1 = routine.trajectory("RS-B1");
        AutoTrajectory b1rs = routine.trajectory("B1-RS");
        AutoTrajectory rsb2 = routine.trajectory("RS-B2");
        AutoTrajectory b2rs = routine.trajectory("B2-RS");
        AutoTrajectory rsa2 = routine.trajectory("RS-A2");
        routine.active().onTrue(
                Commands.sequence(
                        start.resetOdometry(),
                        placeOnReef(start, true),
                        pickupFromCoralStation(c1rs),
                        placeOnReef(rsb1, false),
                        pickupFromCoralStation(b1rs),
                        placeOnReef(rsb2, false),
                        pickupFromCoralStation(b2rs),
                        placeOnReefl2(rsa2, false)
                )
        );

        return routine;
    }

    public boolean getAllianceRed() {
        return DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
    }

    public double getMaxVelocity(double elevatorHeight){
        final double UPPER_HEIGHT = Units.inchesToMeters(36.0);
        final double LOWER_HEIGHT = Units.inchesToMeters(24.0);

        final double SLOW_VELOCITY = 2.0;

        if (elevatorHeight > UPPER_HEIGHT)
            return SLOW_VELOCITY;
        else if (elevatorHeight < LOWER_HEIGHT)
            return drivetrain.getConstants().maxLinearVelocity();

        double t = MathUtil.inverseInterpolate(LOWER_HEIGHT, UPPER_HEIGHT, elevatorHeight);

        return MathUtil.interpolate(
                drivetrain.getConstants().maxLinearVelocity(),
                SLOW_VELOCITY,
                t);
    }
}