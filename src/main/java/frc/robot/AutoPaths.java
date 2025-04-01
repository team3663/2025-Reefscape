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
    /**
     * The maximum velocity of the robot when the elevator is extended.
     * <p>
     * This velocity cap is applied when the elevator is above {@link #ELEVATOR_EXTENDED_UPPER_HEIGHT}, and will
     * gradually be applied when it is between {@link #ELEVATOR_EXTENDED_LOWER_HEIGHT} and
     * {@link #ELEVATOR_EXTENDED_UPPER_HEIGHT}.
     */
    private static final double ELEVATOR_EXTENDED_MAX_VELOCITY = 2.0;
    private static final double ELEVATOR_EXTENDED_UPPER_HEIGHT = Units.inchesToMeters(36.0);
    private static final double ELEVATOR_EXTENDED_LOWER_HEIGHT = Units.inchesToMeters(24.0);

    /**
     * The distance away from the target pickup position at which the arm will no longer be limited in motion.
     * <p>
     * This is used to limit the extension of the superstructure at the beginning of a path.
     */
    private static final double PICKUP_LIMITED_ARM_DISTANCE = Units.feetToMeters(3.0);
    /**
     * The distance away from the target placement position at which the arm will no longer be limited in motion.
     * <p>
     * This is used to limit the extension of the superstructure at the beginning of a path.
     */
    private static final double PLACE_LIMITED_ARM_DISTANCE = Units.feetToMeters(3.0);
    /**
     * The distance away from the target placement position the robot can be in order to place the game piece.
     */
    private static final double PLACE_DISTANCE_THRESHOLD = Units.inchesToMeters(2.0);

    /**
     * The distance at which the robot will switch to the target pose when picking up around the reef.
     * Smaller distances will result in a sharper curve.
     */
    private static final double PICKUP_AROUND_REEF_INTERMEDIATE_POSE_DISTANCE = Units.feetToMeters(5.0);
    /**
     * The distance at which the robot will switch to the target pose when placing around the reef.
     * Smaller distances will result in a sharper curve.
     */
    private static final double PLACE_AROUND_REEF_INTERMEDIATE_POSE_DISTANCE = Units.feetToMeters(2.0);

    private final Drivetrain drivetrain;
    private final Grabber grabber;
    private final Arm arm;
    private final Elevator elevator;
    private final SuperStructure superStructure;
    private final AutoFactory autoFactory;

    public AutoPaths(
            Drivetrain drivetrain, Grabber grabber,
            SuperStructure superStructure, AutoFactory autoFactory, Arm arm, Elevator elevator) {
        this.drivetrain = drivetrain;
        this.grabber = grabber;
        this.superStructure = superStructure;
        this.autoFactory = autoFactory;
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

    /**
     * Move the superstructure to the desired robot mode while following the motion limits.
     * <p>
     * Used to prevent the superstructure from extending out early when driving or extending too high when not desired.
     * <p>
     * The superstructure will get as close as possible to the robot mode position while not violating the limits imposed by the
     * {@link Constants.ArmPositions#ELEVATOR_MAX_MOVING_HEIGHT}, {@link Constants.ArmPositions#SHOULDER_MAX_MOVING_OFFSET}, and {@link Constants.ArmPositions#WRIST_MOVING_OFFSET}.
     */
    private Command limitedArm(RobotMode robotMode) {
        return superStructure.followPositions(
                () -> Math.min(robotMode.getElevatorHeight(), Constants.ArmPositions.ELEVATOR_MAX_MOVING_HEIGHT),
                () -> MathUtil.clamp(robotMode.getShoulderAngle(),
                        Units.degreesToRadians(90.0) - Constants.ArmPositions.SHOULDER_MAX_MOVING_OFFSET,
                        Units.degreesToRadians(90.0) + Constants.ArmPositions.SHOULDER_MAX_MOVING_OFFSET),
                () -> (robotMode.getWristAngle() + Constants.ArmPositions.WRIST_MOVING_OFFSET));
    }

    /**
     * Resets the odometry of the drivetrain to an alliance-dependent position.
     * <p>
     * This command requires the {@link Drivetrain} and executes instantly.
     *
     * @param blueAlliancePose the pose to reset odometry to if the robot is on the blue alliance when the command executes
     * @param redAlliancePose  the pose to reset odometry to if the robot is on the red alliance when the command executes
     */
    private Command resetOdometry(Pose2d blueAlliancePose, Pose2d redAlliancePose) {
        return Commands.defer(() -> drivetrain.resetOdometry(alliancePose(blueAlliancePose, redAlliancePose)), Set.of(drivetrain));
    }

    /**
     * Drives the robot to the target position, travelling through intermediate positions if supplied.
     * <p>
     * This command requires the {@link Drivetrain} and does not end.
     * <p>
     * If intermediate positions are supplied, the robot will attempt to drive toward those instead of the target position.
     * If the intermediate position supplier has no more intermediate positions, or no longer wishes the for the robot to go to an intermediate position, it should return {@code null}.
     * Once a supplier has returned {@code null}, it will no longer be checked for intermediate positions and the robot will drive directly to the target position.
     * <p>
     * Intermediate positions allow for indirect travel.
     * See the implementations of {@link #placeAroundReef(Pose2d, Pose2d, Pose2d, Pose2d, RobotMode)} or {@link #pickupAroundReef(Pose2d, Pose2d, Pose2d, Pose2d)} for examples of its usage.
     *
     * @param targetPose               the target position of the robot
     * @param intermediatePoseSupplier a supplier of possibly {@code null} intermediate positions to travel towards
     */
    private Command goToPosition(Pose2d targetPose, Supplier<Pose2d> intermediatePoseSupplier) {
        Pose2d[] intermediateHolder = new Pose2d[]{null};

        return drivetrain.goToPosition(() -> {
                    if (intermediateHolder[0] != null && (intermediateHolder[0] = intermediatePoseSupplier.get()) != null)
                        return intermediateHolder[0];

                    return targetPose;
                },
                () -> false,
                // Limit the drive speed based on the elevator position
                () -> {
                    var elevatorHeight = elevator.getPosition();

                    if (elevatorHeight > ELEVATOR_EXTENDED_UPPER_HEIGHT) {
                        return ELEVATOR_EXTENDED_MAX_VELOCITY;
                    } else if (elevatorHeight < ELEVATOR_EXTENDED_LOWER_HEIGHT) {
                        return drivetrain.getConstants().maxLinearVelocity();
                    }

                    double t = MathUtil.inverseInterpolate(ELEVATOR_EXTENDED_LOWER_HEIGHT, ELEVATOR_EXTENDED_UPPER_HEIGHT, elevatorHeight);

                    return MathUtil.interpolate(drivetrain.getConstants().maxLinearVelocity(), ELEVATOR_EXTENDED_MAX_VELOCITY, t);
                }).beforeStarting(() -> intermediateHolder[0] = intermediatePoseSupplier.get());
    }

    /**
     * Drives the robot to the target position.
     * <p>
     * This command requires the {@link Drivetrain} and does not end.
     *
     * @param targetPose the target position of the robot
     */
    private Command goToPosition(Pose2d targetPose) {
        return goToPosition(targetPose, () -> null);
    }

    /**
     * Picks up a coral from a coral station while driving towards any intermediate positions.
     * <p>
     * Intermediate positions behave the same as {@link #goToPosition(Pose2d, Supplier)}.
     *
     * @param blueAllianceTargetPose   the position to go to when on the blue alliance
     * @param redAllianceTargetPose    the position to go to when on the red alliance
     * @param intermediatePoseSupplier a supplier of any intermediate positions the robot should travel towards
     */
    private Command pickup(Pose2d blueAllianceTargetPose, Pose2d redAllianceTargetPose, Supplier<Pose2d> intermediatePoseSupplier) {
        Pose2d[] targetPoseHolder = new Pose2d[]{null};

        return Commands.defer(() -> goToPosition(targetPoseHolder[0], intermediatePoseSupplier), Set.of(drivetrain))
                .withDeadline(Commands.sequence(
                        limitedArm(RobotMode.CORAL_STATION).until(() -> drivetrain.atPosition(targetPoseHolder[0].getTranslation(), PICKUP_LIMITED_ARM_DISTANCE)),
                        superStructure.goToPositions(RobotMode.CORAL_STATION).alongWith(grabber.grabCoral())))
                .beforeStarting(() -> targetPoseHolder[0] = alliancePose(blueAllianceTargetPose, redAllianceTargetPose));
    }

    /**
     * Picks up a coral by driving directly to the coral station.
     */
    private Command pickup(Pose2d blueAllianceTargetPose, Pose2d redAllianceTargetPose) {
        return pickup(blueAllianceTargetPose, redAllianceTargetPose, () -> null);
    }

    /**
     * Places a coral on the desired level while driving towards any intermediate positions and optionally zeroing the superstructure.
     * <p>
     * Intermediate positions behave the same as {@link #goToPosition(Pose2d, Supplier)}.
     * <p>
     * If the superstructure should be zeroed, the zeroing will occur while the is moving to the target position.
     * The robot will wait for the zeroing sequence to complete before moving the superstructure to the placement position.
     *
     * @param blueAllianceTargetPose   the position to go to when on the blue alliance
     * @param redAllianceTargetPose    the position to go to when on the red alliance
     * @param robotMode                the robot mode to use when placing the coral
     * @param intermediatePoseSupplier a supplier of any intermediate positions the robot should travel towards
     * @param shouldZero               whether the superstructure should zero while the robot is moving to the target position
     */
    private Command place(Pose2d blueAllianceTargetPose, Pose2d redAllianceTargetPose, RobotMode robotMode, Supplier<Pose2d> intermediatePoseSupplier, boolean shouldZero) {
        Pose2d[] targetPoseHolder = new Pose2d[]{null};

        return Commands.defer(() -> goToPosition(targetPoseHolder[0], intermediatePoseSupplier), Set.of(drivetrain))
                .withDeadline(Commands.sequence(
                        shouldZero ? superStructure.zero() : Commands.none(),
                        limitedArm(robotMode).until(() -> drivetrain.atPosition(targetPoseHolder[0].getTranslation(), PLACE_LIMITED_ARM_DISTANCE)),
                        superStructure.goToPositions(robotMode),
                        Commands.waitUntil(() -> drivetrain.atPosition(targetPoseHolder[0].getTranslation(), PLACE_DISTANCE_THRESHOLD))))
                .andThen(Commands.either(grabber.placeCoralL4(), grabber.placeCoral(), () -> robotMode == RobotMode.CORAL_LEVEL_4)
                        .withDeadline(Commands.waitUntil(grabber::getGamePieceNotDetected)))
                .beforeStarting(() -> targetPoseHolder[0] = alliancePose(blueAllianceTargetPose, redAllianceTargetPose));
    }

    /**
     * Places a coral on the desired level by driving straight to the target position.
     *
     * @see #zeroAndPlace(Pose2d, Pose2d, RobotMode) for a command that zeroes the superstructure while travelling to the target position.
     */
    private Command place(Pose2d blueAllianceTargetPose, Pose2d redAllianceTargetPose, RobotMode robotMode) {
        return place(blueAllianceTargetPose, redAllianceTargetPose, robotMode, () -> null, false);
    }

    /**
     * Zeroes the superstructure and places a coral in parallel. The superstructure is fully zeroed before the coral is placed.
     *
     * @see #place(Pose2d, Pose2d, RobotMode) for a non-zeroing alternative
     */
    private Command zeroAndPlace(Pose2d blueAllianceTargetPose, Pose2d redAllianceTargetPose, RobotMode robotMode) {
        return place(blueAllianceTargetPose, redAllianceTargetPose, robotMode, () -> null, true);
    }

    /**
     * Picks up a coral by travelling around the reef. Assumes the robot is near either the C or F faces.
     */
    private Command pickupAroundReef(Pose2d blueAllianceTargetPose, Pose2d redAllianceTargetPose, Pose2d blueAllianceIntermediatePose, Pose2d redAllianceIntermediatePose) {
        Pose2d[] intermediatePoseHolder = new Pose2d[]{null};
        return pickup(blueAllianceTargetPose, redAllianceTargetPose, () -> drivetrain.atPosition(intermediatePoseHolder[0].getTranslation(), PICKUP_AROUND_REEF_INTERMEDIATE_POSE_DISTANCE) ? null : intermediatePoseHolder[0])
                .beforeStarting(() -> intermediatePoseHolder[0] = alliancePose(blueAllianceIntermediatePose, redAllianceIntermediatePose));
    }

    /**
     * Places a coral by travelling around the reef. Assumes the robot is near the coral station and is placing on either the C or F faces.
     */
    private Command placeAroundReef(Pose2d blueAllianceTargetPose, Pose2d redAllianceTargetPose, Pose2d blueAllianceIntermediatePose, Pose2d redAllianceIntermediatePose, RobotMode robotMode) {
        Pose2d[] intermediatePoseHolder = new Pose2d[]{null};
        return place(blueAllianceTargetPose, redAllianceTargetPose, robotMode, () -> drivetrain.atPosition(intermediatePoseHolder[0].getTranslation(), PLACE_AROUND_REEF_INTERMEDIATE_POSE_DISTANCE) ? null : intermediatePoseHolder[0], false)
                .beforeStarting(() -> intermediatePoseHolder[0] = alliancePose(blueAllianceIntermediatePose, redAllianceIntermediatePose));
    }

    public AutoRoutine facePlantD1() {
        AutoRoutine routine = autoFactory.newRoutine("FacePlant:D1");

        routine.active().onTrue(Commands.sequence(
                resetOdometry(Constants.BLUE_AUTO_CENTER_STARTING_POSITION, Constants.RED_AUTO_CENTER_STARTING_POSITION),
                // Place first piece D1-L4
                zeroAndPlace(Constants.BLUE_BRANCH_D1, Constants.RED_BRANCH_D1, RobotMode.CORAL_LEVEL_4),
                superStructure.goToDefaultPositions()
        ));
        return routine;
    }

    public AutoRoutine facePlantD2() {
        AutoRoutine routine = autoFactory.newRoutine("FacePlant:D2");

        routine.active().onTrue(Commands.sequence(
                resetOdometry(Constants.BLUE_AUTO_CENTER_STARTING_POSITION, Constants.RED_AUTO_CENTER_STARTING_POSITION),
                // Place first piece D2-L4
                zeroAndPlace(Constants.BLUE_BRANCH_D2, Constants.RED_BRANCH_D2, RobotMode.CORAL_LEVEL_4),
                superStructure.goToDefaultPositions()
        ));
        return routine;
    }

    public AutoRoutine threeCoralC1B1B2() {
        AutoRoutine routine = autoFactory.newRoutine("ThreeCoral:C1-B1-B2");

        routine.active().onTrue(Commands.sequence(
                resetOdometry(Constants.BLUE_AUTO_RIGHT_STARTING_POSITION_7FT, Constants.RED_AUTO_RIGHT_STARTING_POSITION_7FT),
                // Place first piece C1-L4
                zeroAndPlace(Constants.BLUE_BRANCH_C1, Constants.RED_BRANCH_C1, RobotMode.CORAL_LEVEL_4),
                // Pickup second piece by driving around the reef
                pickupAroundReef(Constants.BLUE_RIGHT_FAR_SIDE_CORAL_STATION, Constants.RED_RIGHT_FAR_SIDE_CORAL_STATION, Constants.BLUE_RIGHT_PICKUP_AROUND_REEF_INTERMEDIATE, Constants.RED_RIGHT_PICKUP_AROUND_REEF_INTERMEDIATE),
                // Place second piece B1-L4
                place(Constants.BLUE_BRANCH_B1, Constants.RED_BRANCH_B1, RobotMode.CORAL_LEVEL_4),
                // Pickup third piece
                pickup(Constants.BLUE_RIGHT_FAR_SIDE_CORAL_STATION, Constants.RED_RIGHT_FAR_SIDE_CORAL_STATION),
                // Place third piece B2-L4
                place(Constants.BLUE_BRANCH_B2, Constants.RED_BRANCH_B2, RobotMode.CORAL_LEVEL_4),
                // Pickup fourth piece
                pickup(Constants.BLUE_RIGHT_FAR_SIDE_CORAL_STATION, Constants.RED_RIGHT_FAR_SIDE_CORAL_STATION),
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
                zeroAndPlace(Constants.BLUE_BRANCH_E2, Constants.RED_BRANCH_E2, RobotMode.CORAL_LEVEL_4),
                // Pickup second piece by driving around the reef
                pickupAroundReef(Constants.BLUE_LEFT_FAR_SIDE_CORAL_STATION, Constants.RED_LEFT_FAR_SIDE_CORAL_STATION, Constants.BLUE_LEFT_PICKUP_AROUND_REEF_INTERMEDIATE, Constants.RED_LEFT_PICKUP_AROUND_REEF_INTERMEDIATE),
                // Place second piece F2-L4
                place(Constants.BLUE_BRANCH_F2, Constants.RED_BRANCH_F2, RobotMode.CORAL_LEVEL_4),
                // Pickup third piece
                pickup(Constants.BLUE_LEFT_FAR_SIDE_CORAL_STATION, Constants.RED_LEFT_FAR_SIDE_CORAL_STATION),
                // Place third piece F1-L4
                place(Constants.BLUE_BRANCH_F1, Constants.RED_BRANCH_F1, RobotMode.CORAL_LEVEL_4),
                // Pickup fourth piece
                pickup(Constants.BLUE_LEFT_FAR_SIDE_CORAL_STATION, Constants.RED_LEFT_FAR_SIDE_CORAL_STATION),
                // Go back to F1 to prep for teleop
                Commands.defer(() -> goToPosition(alliancePose(Constants.BLUE_BRANCH_F2, Constants.RED_BRANCH_F2)), Set.of(drivetrain))
        ));

        return routine;
    }

    public AutoRoutine fourCoralC1B1B1B2() {
        AutoRoutine routine = autoFactory.newRoutine("FourCoral:C1-B1-B1-B2");

        routine.active().onTrue(Commands.sequence(
                resetOdometry(Constants.BLUE_AUTO_RIGHT_STARTING_POSITION_7FT, Constants.RED_AUTO_RIGHT_STARTING_POSITION_7FT),
                // Place first piece C1-L4
                zeroAndPlace(Constants.BLUE_BRANCH_C1, Constants.RED_BRANCH_C1, RobotMode.CORAL_LEVEL_4),
                // Pickup second piece by driving around the reef
                pickupAroundReef(Constants.BLUE_RIGHT_FAR_SIDE_CORAL_STATION, Constants.RED_RIGHT_FAR_SIDE_CORAL_STATION, Constants.BLUE_RIGHT_PICKUP_AROUND_REEF_INTERMEDIATE, Constants.RED_RIGHT_PICKUP_AROUND_REEF_INTERMEDIATE),
                // Place second piece B1-L4
                place(Constants.BLUE_BRANCH_B1, Constants.RED_BRANCH_B1, RobotMode.CORAL_LEVEL_4),
                // Pickup third piece
                pickup(Constants.BLUE_RIGHT_FAR_SIDE_CORAL_STATION, Constants.RED_RIGHT_FAR_SIDE_CORAL_STATION),
                // Place third piece B1-L3
                place(Constants.BLUE_BRANCH_B1, Constants.RED_BRANCH_B1, RobotMode.CORAL_LEVEL_3),
                // Pickup fourth piece
                pickup(Constants.BLUE_RIGHT_FAR_SIDE_CORAL_STATION, Constants.RED_RIGHT_FAR_SIDE_CORAL_STATION),
                // Place fourth piece B2-L3
                place(Constants.BLUE_BRANCH_B2, Constants.RED_BRANCH_B2, RobotMode.CORAL_LEVEL_3),
                // Pickup fifth piece (likely not happening)
                pickup(Constants.BLUE_RIGHT_FAR_SIDE_CORAL_STATION, Constants.RED_RIGHT_FAR_SIDE_CORAL_STATION)
        ));

        return routine;
    }

    public AutoRoutine fourCoralE2F2F2F1() {
        AutoRoutine routine = autoFactory.newRoutine("FourCoral:E2-F2-F2-F1");

        routine.active().onTrue(Commands.sequence(
                resetOdometry(Constants.BLUE_AUTO_LEFT_STARTING_POSITION_7FT, Constants.RED_AUTO_LEFT_STARTING_POSITION_7FT),
                // Place first piece E2-L4
                zeroAndPlace(Constants.BLUE_BRANCH_E2, Constants.RED_BRANCH_E2, RobotMode.CORAL_LEVEL_4),
                // Pickup second piece by driving around the reef
                pickupAroundReef(Constants.BLUE_LEFT_FAR_SIDE_CORAL_STATION, Constants.RED_LEFT_FAR_SIDE_CORAL_STATION, Constants.BLUE_LEFT_PICKUP_AROUND_REEF_INTERMEDIATE, Constants.RED_LEFT_PICKUP_AROUND_REEF_INTERMEDIATE),
                // Place second piece F2-L4
                place(Constants.BLUE_BRANCH_F2, Constants.RED_BRANCH_F2, RobotMode.CORAL_LEVEL_4),
                // Pickup third piece
                pickup(Constants.BLUE_LEFT_FAR_SIDE_CORAL_STATION, Constants.RED_LEFT_FAR_SIDE_CORAL_STATION),
                // Place third piece F2-L3
                place(Constants.BLUE_BRANCH_F2, Constants.RED_BRANCH_F2, RobotMode.CORAL_LEVEL_3),
                // Pickup fourth piece
                pickup(Constants.BLUE_LEFT_FAR_SIDE_CORAL_STATION, Constants.RED_LEFT_FAR_SIDE_CORAL_STATION),
                // Place fourth piece B1-L3
                place(Constants.BLUE_BRANCH_F1, Constants.RED_BRANCH_F1, RobotMode.CORAL_LEVEL_3),
                // Pickup fifth piece (likely not happening)
                pickup(Constants.BLUE_LEFT_FAR_SIDE_CORAL_STATION, Constants.RED_LEFT_FAR_SIDE_CORAL_STATION)
        ));

        return routine;
    }

    public AutoRoutine fourCoralC2B1B2C1() {
        AutoRoutine routine = autoFactory.newRoutine("FourCoral:C2-B1-B2-C1");

        routine.active().onTrue(Commands.sequence(
                resetOdometry(Constants.BLUE_AUTO_RIGHT_STARTING_POSITION_7FT, Constants.RED_AUTO_RIGHT_STARTING_POSITION_7FT),
                // Place first piece C2-L2
                zeroAndPlace(Constants.BLUE_BRANCH_C2, Constants.RED_BRANCH_C2, RobotMode.CORAL_LEVEL_2),
                // Pickup second piece by driving around the reef
                pickupAroundReef(Constants.BLUE_RIGHT_FAR_SIDE_CORAL_STATION, Constants.RED_RIGHT_FAR_SIDE_CORAL_STATION, Constants.BLUE_RIGHT_PICKUP_AROUND_REEF_INTERMEDIATE, Constants.RED_RIGHT_PICKUP_AROUND_REEF_INTERMEDIATE),
                // Place second piece B1-L4
                place(Constants.BLUE_BRANCH_B1, Constants.RED_BRANCH_B1, RobotMode.CORAL_LEVEL_4),
                // Pickup third piece
                pickup(Constants.BLUE_RIGHT_FAR_SIDE_CORAL_STATION, Constants.RED_RIGHT_FAR_SIDE_CORAL_STATION),
                // Place third piece B2-L4
                place(Constants.BLUE_BRANCH_B2, Constants.RED_BRANCH_B2, RobotMode.CORAL_LEVEL_4),
                // Pickup fourth piece
                pickup(Constants.BLUE_RIGHT_FAR_SIDE_CORAL_STATION, Constants.RED_RIGHT_FAR_SIDE_CORAL_STATION),
                // Place fourth piece C1-L2 by driving around the reef
                placeAroundReef(Constants.BLUE_BRANCH_C1, Constants.RED_BRANCH_C1, Constants.BLUE_RIGHT_PLACE_AROUND_REEF_INTERMEDIATE, Constants.RED_RIGHT_PLACE_AROUND_REEF_INTERMEDIATE, RobotMode.CORAL_LEVEL_2),
                superStructure.goToDefaultPositions()
        ));
        return routine;
    }

    public AutoRoutine fourCoralE1F2F1E2() {
        AutoRoutine routine = autoFactory.newRoutine("FourCoral:E1-F2-F1-E2");

        routine.active().onTrue(Commands.sequence(
                resetOdometry(Constants.BLUE_AUTO_LEFT_STARTING_POSITION_7FT, Constants.RED_AUTO_LEFT_STARTING_POSITION_7FT),
                // Place first piece E1-L2
                zeroAndPlace(Constants.BLUE_BRANCH_E1, Constants.RED_BRANCH_E1, RobotMode.CORAL_LEVEL_2),
                // Pickup second piece by driving around the reef
                pickupAroundReef(Constants.BLUE_LEFT_FAR_SIDE_CORAL_STATION, Constants.RED_LEFT_FAR_SIDE_CORAL_STATION, Constants.BLUE_LEFT_PICKUP_AROUND_REEF_INTERMEDIATE, Constants.RED_LEFT_PICKUP_AROUND_REEF_INTERMEDIATE),
                // Place second piece F2-L4
                place(Constants.BLUE_BRANCH_F2, Constants.RED_BRANCH_F2, RobotMode.CORAL_LEVEL_4),
                // Pickup third piece
                pickup(Constants.BLUE_LEFT_FAR_SIDE_CORAL_STATION, Constants.RED_LEFT_FAR_SIDE_CORAL_STATION),
                // Place third piece F1-L4
                place(Constants.BLUE_BRANCH_F1, Constants.RED_BRANCH_F1, RobotMode.CORAL_LEVEL_4),
                // Pickup fourth piece
                pickup(Constants.BLUE_LEFT_FAR_SIDE_CORAL_STATION, Constants.RED_LEFT_FAR_SIDE_CORAL_STATION),
                // Place fourth piece E2-L2 by driving around the reef
                placeAroundReef(Constants.BLUE_BRANCH_E2, Constants.RED_BRANCH_E2, Constants.BLUE_LEFT_PLACE_AROUND_REEF_INTERMEDIATE, Constants.RED_LEFT_PLACE_AROUND_REEF_INTERMEDIATE, RobotMode.CORAL_LEVEL_2),
                superStructure.goToDefaultPositions()
        ));
        return routine;
    }
}