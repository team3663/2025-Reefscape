package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.grabber.Grabber;

import java.util.Set;
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

    /**
     * Returns the pose for the correct alliance.
     *
     * @param blueAlliancePose the pose to return when the robot is on the blue alliance.
     * @param redAlliancePose  the pose to return when the robot is on the red alliance.
     */
    private Pose2d alliancePose(Pose2d blueAlliancePose, Pose2d redAlliancePose) {
        return DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue ? blueAlliancePose : redAlliancePose;
    }

    private Command limitedArm(RobotMode robotMode) {
        return superStructure.followPositions(
                () -> Math.min(robotMode.getElevatorHeight(), Constants.ArmPositions.ELEVATOR_MAX_MOVING_HEIGHT),
                () -> MathUtil.clamp(robotMode.getShoulderAngle(),
                        Units.degreesToRadians(90.0) - Constants.ArmPositions.SHOULDER_MAX_MOVING_OFFSET,
                        Units.degreesToRadians(90.0) + Constants.ArmPositions.SHOULDER_MAX_MOVING_OFFSET),
                () -> (robotMode.getWristAngle() + Constants.ArmPositions.WRIST_MOVING_OFFSET));
    }

    private Command resetOdometry(Pose2d blueAlliancePose, Pose2d redAlliancePose) {
        return Commands.defer(() -> drivetrain.resetOdometry(alliancePose(blueAlliancePose, redAlliancePose)), Set.of(drivetrain));
    }

    private Command goToPosition(Pose2d targetPose, Supplier<Pose2d> intermediatePoseSupplier) {
        final double UPPER_HEIGHT = Units.inchesToMeters(36.0);
        final double LOWER_HEIGHT = Units.inchesToMeters(24.0);

        final double SLOW_VELOCITY = 2.0;

        Pose2d[] intermediateHolder = new Pose2d[]{null};

        return Commands.sequence(
                Commands.runOnce(() -> intermediateHolder[0] = intermediatePoseSupplier.get()),
                drivetrain.goToPosition(() -> {
                            if (intermediateHolder[0] != null && (intermediateHolder[0] = intermediatePoseSupplier.get()) != null)
                                return intermediateHolder[0];

                            return targetPose;
                        },
                        () -> false,
                        // Limit the drive speed based on the elevator position
                        () -> {
                            var elevatorHeight = elevator.getPosition();

                            if (elevatorHeight > UPPER_HEIGHT)
                                return SLOW_VELOCITY;
                            else if (elevatorHeight < LOWER_HEIGHT)
                                return drivetrain.getConstants().maxLinearVelocity();

                            double t = MathUtil.inverseInterpolate(LOWER_HEIGHT, UPPER_HEIGHT, elevatorHeight);

                            return MathUtil.interpolate(
                                    drivetrain.getConstants().maxLinearVelocity(),
                                    SLOW_VELOCITY,
                                    t);
                        }));
    }

    private Command goToPosition(Pose2d targetPose) {
        return goToPosition(targetPose, () -> null);
    }

    private Command pickup(Supplier<Pose2d> targetPoseSupplier, Supplier<Pose2d> intermediatePoseSupplier) {
        Pose2d[] targetPoseHolder = new Pose2d[]{null};

        return Commands.defer(() -> goToPosition(targetPoseHolder[0], intermediatePoseSupplier), Set.of(drivetrain))
                .withDeadline(Commands.sequence(
                        limitedArm(RobotMode.CORAL_STATION).until(() -> drivetrain.getPose().getTranslation().getDistance(targetPoseHolder[0].getTranslation()) < Units.feetToMeters(3.0)),
                        superStructure.goToPositions(RobotMode.CORAL_STATION).alongWith(grabber.grabCoral())
                ))
                .beforeStarting(() -> targetPoseHolder[0] = targetPoseSupplier.get());
    }

    private Command pickup(Supplier<Pose2d> targetPoseSupplier) {
        return pickup(targetPoseSupplier, () -> null);
    }

    private Command pickupAroundReef(Supplier<Pose2d> targetPoseSupplier, Supplier<Pose2d> intermediatePoseSupplier) {
        final double INTERMEDIATE_POSE_DISTANCE = Units.feetToMeters(5.0);

        Pose2d[] intermediatePoseHolder = new Pose2d[]{null};
        return pickup(targetPoseSupplier, () -> drivetrain.atPosition(intermediatePoseHolder[0].getTranslation(), INTERMEDIATE_POSE_DISTANCE) ? null : intermediatePoseHolder[0])
                .beforeStarting(() -> intermediatePoseHolder[0] = intermediatePoseSupplier.get());
    }

    private Command place(Supplier<Pose2d> targetPoseSupplier, RobotMode robotMode, boolean shouldZero, Supplier<Pose2d> intermediatePoseSupplier) {
        Pose2d[] targetPoseHolder = new Pose2d[]{null};

        return Commands.defer(() -> goToPosition(targetPoseHolder[0], intermediatePoseSupplier), Set.of(drivetrain))
                .withDeadline(Commands.sequence(
                        shouldZero ? superStructure.zero() : Commands.none(),
                        limitedArm(robotMode).until(() -> drivetrain.getPose().getTranslation().getDistance(targetPoseHolder[0].getTranslation()) < Units.feetToMeters(3.0)),
                        superStructure.goToPositions(robotMode),
                        Commands.waitUntil(() -> drivetrain.getPose().getTranslation().getDistance(targetPoseHolder[0].getTranslation()) < Units.inchesToMeters(2.0))))
                .andThen(Commands.either(grabber.placeCoralL4(), grabber.placeCoral(), () -> robotMode == RobotMode.CORAL_LEVEL_4)
                        .withDeadline(Commands.waitUntil(grabber::getGamePieceNotDetected)))
                .beforeStarting(() -> targetPoseHolder[0] = targetPoseSupplier.get());
    }

    private Command place(Supplier<Pose2d> targetPoseSupplier, RobotMode robotMode, boolean shouldZero) {
        return place(targetPoseSupplier, robotMode, shouldZero, () -> null);
    }

    private Command place(Supplier<Pose2d> targetPoseSupplier, RobotMode robotMode) {
        return place(targetPoseSupplier, robotMode, false, () -> null);
    }

    private Command placeAroundReef(Supplier<Pose2d> targetPoseSupplier, Supplier<Pose2d> intermediatePoseSupplier, RobotMode robotMode) {
        final double INTERMEDIATE_POSE_DISTANCE = Units.feetToMeters(2.0);

        Pose2d[] intermediatePoseHolder = new Pose2d[]{null};
        return place(targetPoseSupplier, robotMode, false, () -> drivetrain.atPosition(intermediatePoseHolder[0].getTranslation(), INTERMEDIATE_POSE_DISTANCE) ? null : intermediatePoseHolder[0])
                .beforeStarting(() -> intermediatePoseHolder[0] = intermediatePoseSupplier.get());
    }

    public AutoRoutine facePlantD1() {
        AutoRoutine routine = autoFactory.newRoutine("FacePlant:D1");

        routine.active().onTrue(Commands.sequence(
                resetOdometry(Constants.BLUE_AUTO_CENTER_STARTING_POSITION, Constants.RED_AUTO_CENTER_STARTING_POSITION),
                // Place first piece D1-L4
                place(() -> alliancePose(Constants.BLUE_BRANCH_D1, Constants.RED_BRANCH_D1), RobotMode.CORAL_LEVEL_4, true),
                superStructure.goToDefaultPositions()
        ));
        return routine;
    }

    public AutoRoutine facePlantD2() {
        AutoRoutine routine = autoFactory.newRoutine("FacePlant:D2");

        routine.active().onTrue(Commands.sequence(
                resetOdometry(Constants.BLUE_AUTO_CENTER_STARTING_POSITION, Constants.RED_AUTO_CENTER_STARTING_POSITION),
                // Place first piece D2-L4
                place(() -> alliancePose(Constants.BLUE_BRANCH_D2, Constants.RED_BRANCH_D2), RobotMode.CORAL_LEVEL_4, true),
                superStructure.goToDefaultPositions()
        ));
        return routine;
    }

    public AutoRoutine threeCoralC1B1B2() {
        AutoRoutine routine = autoFactory.newRoutine("ThreeCoral:C1-B1-B2");

        routine.active().onTrue(Commands.sequence(
                resetOdometry(Constants.BLUE_AUTO_RIGHT_STARTING_POSITION_7FT, Constants.RED_AUTO_RIGHT_STARTING_POSITION_7FT),
                // Place first piece C1-L4
                place(() -> alliancePose(Constants.BLUE_BRANCH_C1, Constants.RED_BRANCH_C1), RobotMode.CORAL_LEVEL_4, true),
                // Pickup second piece by driving around the reef
                pickupAroundReef(() -> alliancePose(Constants.BLUE_RIGHT_FAR_SIDE_CORAL_STATION, Constants.RED_RIGHT_FAR_SIDE_CORAL_STATION), () -> alliancePose(Constants.BLUE_RIGHT_PICKUP_AROUND_REEF_INTERMEDIATE, Constants.RED_RIGHT_PICKUP_AROUND_REEF_INTERMEDIATE)),
                // Place second piece B1-L4
                place(() -> alliancePose(Constants.BLUE_BRANCH_B1, Constants.RED_BRANCH_B1), RobotMode.CORAL_LEVEL_4),
                // Pickup third piece
                pickup(() -> alliancePose(Constants.BLUE_RIGHT_FAR_SIDE_CORAL_STATION, Constants.RED_RIGHT_FAR_SIDE_CORAL_STATION)),
                // Place third piece B2-L4
                place(() -> alliancePose(Constants.BLUE_BRANCH_B2, Constants.RED_BRANCH_B2), RobotMode.CORAL_LEVEL_4),
                // Pickup fourth piece
                pickup(() -> alliancePose(Constants.BLUE_RIGHT_FAR_SIDE_CORAL_STATION, Constants.RED_RIGHT_FAR_SIDE_CORAL_STATION)),
                // Go back to B2 to prep for teleop
                Commands.defer(() -> goToPosition(alliancePose(Constants.BLUE_BRANCH_B2, Constants.RED_BRANCH_B2)), Set.of(drivetrain)).alongWith(superStructure.goToDefaultPositions())
        ));

        return routine;
    }

    public AutoRoutine threeCoralE2F2F1() {
        AutoRoutine routine = autoFactory.newRoutine("ThreeCoral:E2-F2-F1");

        routine.active().onTrue(Commands.sequence(
                resetOdometry(Constants.BLUE_AUTO_LEFT_STARTING_POSITION_7FT, Constants.RED_AUTO_LEFT_STARTING_POSITION_7FT),
                // Place first piece E2-L4
                place(() -> alliancePose(Constants.BLUE_BRANCH_E2, Constants.RED_BRANCH_E2), RobotMode.CORAL_LEVEL_4, true),
                // Pickup second piece by driving around the reef
                pickupAroundReef(() -> alliancePose(Constants.BLUE_LEFT_FAR_SIDE_CORAL_STATION, Constants.RED_LEFT_FAR_SIDE_CORAL_STATION), () -> alliancePose(Constants.BLUE_LEFT_PICKUP_AROUND_REEF_INTERMEDIATE, Constants.RED_LEFT_PICKUP_AROUND_REEF_INTERMEDIATE)),
                // Place second piece F2-L4
                place(() -> alliancePose(Constants.BLUE_BRANCH_F2, Constants.RED_BRANCH_F2), RobotMode.CORAL_LEVEL_4),
                // Pickup third piece
                pickup(() -> alliancePose(Constants.BLUE_LEFT_FAR_SIDE_CORAL_STATION, Constants.RED_LEFT_FAR_SIDE_CORAL_STATION)),
                // Place third piece F1-L4
                place(() -> alliancePose(Constants.BLUE_BRANCH_F1, Constants.RED_BRANCH_F1), RobotMode.CORAL_LEVEL_4),
                // Pickup fourth piece
                pickup(() -> alliancePose(Constants.BLUE_LEFT_FAR_SIDE_CORAL_STATION, Constants.RED_LEFT_FAR_SIDE_CORAL_STATION)),
                // Go back to F1 to prep for teleop
                Commands.defer(() -> goToPosition(alliancePose(Constants.BLUE_BRANCH_F2, Constants.RED_BRANCH_F2)), Set.of(drivetrain))
        ));

        return routine;
    }

    public AutoRoutine fourCoralC2B1B2C1() {
        AutoRoutine routine = autoFactory.newRoutine("FourCoral:C2-B1-B2-C1");

        routine.active().onTrue(Commands.sequence(
                resetOdometry(Constants.BLUE_AUTO_RIGHT_STARTING_POSITION_7FT, Constants.RED_AUTO_RIGHT_STARTING_POSITION_7FT),
                // Place first piece C2-L2
                place(() -> alliancePose(Constants.BLUE_BRANCH_C2, Constants.RED_BRANCH_C2), RobotMode.CORAL_LEVEL_2, true),
                // Pickup second piece by driving around the reef
                pickupAroundReef(() -> alliancePose(Constants.BLUE_RIGHT_FAR_SIDE_CORAL_STATION, Constants.RED_RIGHT_FAR_SIDE_CORAL_STATION), () -> alliancePose(Constants.BLUE_RIGHT_PICKUP_AROUND_REEF_INTERMEDIATE, Constants.RED_RIGHT_PICKUP_AROUND_REEF_INTERMEDIATE)),
                // Place second piece B1-L4
                place(() -> alliancePose(Constants.BLUE_BRANCH_B1, Constants.RED_BRANCH_B1), RobotMode.CORAL_LEVEL_4),
                // Pickup third piece
                pickup(() -> alliancePose(Constants.BLUE_RIGHT_FAR_SIDE_CORAL_STATION, Constants.RED_RIGHT_FAR_SIDE_CORAL_STATION)),
                // Place third piece B2-L4
                place(() -> alliancePose(Constants.BLUE_BRANCH_B2, Constants.RED_BRANCH_B2), RobotMode.CORAL_LEVEL_4),
                // Pickup fourth piece
                pickup(() -> alliancePose(Constants.BLUE_RIGHT_FAR_SIDE_CORAL_STATION, Constants.RED_RIGHT_FAR_SIDE_CORAL_STATION)),
                // Place fourth piece C1-L2 by driving around the reef
                placeAroundReef(() -> alliancePose(Constants.BLUE_BRANCH_C1, Constants.RED_BRANCH_C1), () -> alliancePose(Constants.BLUE_RIGHT_PLACE_AROUND_REEF_INTERMEDIATE, Constants.RED_RIGHT_PLACE_AROUND_REEF_INTERMEDIATE), RobotMode.CORAL_LEVEL_2),
                superStructure.goToDefaultPositions()
        ));
        return routine;
    }

    public AutoRoutine fourCoralE1F2F1E2() {
        AutoRoutine routine = autoFactory.newRoutine("FourCoral:E1-F2-F1-E2");

        routine.active().onTrue(Commands.sequence(
                resetOdometry(Constants.BLUE_AUTO_LEFT_STARTING_POSITION_7FT, Constants.RED_AUTO_LEFT_STARTING_POSITION_7FT),
                // Place first piece E1-L2
                place(() -> alliancePose(Constants.BLUE_BRANCH_E1, Constants.RED_BRANCH_E1), RobotMode.CORAL_LEVEL_2, true),
                // Pickup second piece by driving around the reef
                pickupAroundReef(() -> alliancePose(Constants.BLUE_LEFT_FAR_SIDE_CORAL_STATION, Constants.RED_LEFT_FAR_SIDE_CORAL_STATION), () -> alliancePose(Constants.BLUE_LEFT_PICKUP_AROUND_REEF_INTERMEDIATE, Constants.RED_LEFT_PICKUP_AROUND_REEF_INTERMEDIATE)),
                // Place second piece F2-L4
                place(() -> alliancePose(Constants.BLUE_BRANCH_F2, Constants.RED_BRANCH_F2), RobotMode.CORAL_LEVEL_4),
                // Pickup third piece
                pickup(() -> alliancePose(Constants.BLUE_LEFT_FAR_SIDE_CORAL_STATION, Constants.RED_LEFT_FAR_SIDE_CORAL_STATION)),
                // Place third piece F1-L4
                place(() -> alliancePose(Constants.BLUE_BRANCH_F1, Constants.RED_BRANCH_F1), RobotMode.CORAL_LEVEL_4),
                // Pickup fourth piece
                pickup(() -> alliancePose(Constants.BLUE_LEFT_FAR_SIDE_CORAL_STATION, Constants.RED_LEFT_FAR_SIDE_CORAL_STATION)),
                // Place fourth piece E2-L2 by driving around the reef
                placeAroundReef(() -> alliancePose(Constants.BLUE_BRANCH_E2, Constants.RED_BRANCH_E2), () -> alliancePose(Constants.BLUE_LEFT_PLACE_AROUND_REEF_INTERMEDIATE, Constants.RED_LEFT_PLACE_AROUND_REEF_INTERMEDIATE), RobotMode.CORAL_LEVEL_2),
                superStructure.goToDefaultPositions()
        ));
        return routine;
    }
}