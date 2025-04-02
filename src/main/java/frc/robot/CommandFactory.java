package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.grabber.Grabber;
import frc.robot.subsystems.led.Led;
import frc.robot.utility.Gamepiece;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class CommandFactory {
    private final Drivetrain drivetrain;
    private final Elevator elevator;
    private final Arm arm;
    private final Grabber grabber;
    private final Climber climber;
    private final Led led;
    private final SuperStructure superStructure;

    public CommandFactory(
            Drivetrain drivetrain,
            Elevator elevator,
            Arm arm,
            Grabber grabber,
            Climber climber,
            Led led,
            SuperStructure superStructure
    ) {
        this.drivetrain = drivetrain;
        this.elevator = elevator;
        this.arm = arm;
        this.grabber = grabber;
        this.climber = climber;
        this.led = led;
        this.superStructure = superStructure;
    }

    public Pose2d getClosestBranch(Pose2d robotPose, RobotMode robotMode) {
        var alliance = DriverStation.getAlliance();

        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            if (robotMode == RobotMode.ALGAE_NET) {
                var blue_line_x = Constants.BLUE_NET_LINE_X;
                var red_line_x = Constants.RED_NET_LINE_X;
                var nearestY = MathUtil.clamp(robotPose.getY(), Constants.RED_NET_X_RANGE[0], Constants.RED_NET_X_RANGE[1]);
                if (Math.abs(robotPose.getX() - blue_line_x) < Math.abs(robotPose.getX() - red_line_x))
                    return new Pose2d(blue_line_x, nearestY, Constants.BLUE_NET_ROTATION);
                return new Pose2d(red_line_x, nearestY, Constants.RED_NET_ROTATION);
            } else if (robotMode == RobotMode.CORAL_LEVEL_1 || robotMode == RobotMode.ALGAE_REMOVE_LOWER || robotMode == RobotMode.ALGAE_REMOVE_UPPER) {
                return robotPose.nearest(Constants.RED_CENTER_POSES);
            } else {
                return robotPose.nearest(Constants.RED_BRANCH_POSES);
            }
        } else {

            if (robotMode == RobotMode.ALGAE_NET) {
                var blue_line_x = Constants.BLUE_NET_LINE_X;
                var red_line_x = Constants.RED_NET_LINE_X;
                var nearestY = MathUtil.clamp(robotPose.getY(), Constants.BLUE_NET_X_RANGE[0], Constants.BLUE_NET_X_RANGE[1]);
                if (Math.abs(robotPose.getX() - blue_line_x) < Math.abs(robotPose.getX() - red_line_x))
                    return new Pose2d(blue_line_x, nearestY, Constants.BLUE_NET_ROTATION);
                return new Pose2d(red_line_x, nearestY, Constants.RED_NET_ROTATION);
            } else if (robotMode == RobotMode.CORAL_LEVEL_1 || robotMode == RobotMode.ALGAE_REMOVE_LOWER || robotMode == RobotMode.ALGAE_REMOVE_UPPER) {
                return robotPose.nearest(Constants.BLUE_CENTER_POSES);
            } else {
                return robotPose.nearest(Constants.BLUE_BRANCH_POSES);
            }
        }
    }

    public Pose2d getClosestCoralStationPosition(Pose2d robotPose) {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            return robotPose.nearest(Constants.RED_CORAL_STATION_POSES);
        } else {
            return robotPose.nearest(Constants.BLUE_CORAL_STATION_POSES);
        }
    }

    public Boolean shouldAlignToReef(RobotMode robotMode) {
        return SmartDashboard.getBoolean("Auto Reef", true) &&
                robotMode != RobotMode.ALGAE_PROCESSOR && robotMode != RobotMode.ALGAE_PICKUP_GROUND;
    }

    public Command alignToReef(Supplier<RobotMode> robotMode,
                               BooleanSupplier readyToPlace,
                               DoubleSupplier xVelocitySupplier,
                               DoubleSupplier yVelocitySupplier, DoubleSupplier angularVelocitySupplier) {
        return Commands.either(
                        Commands.parallel(
                                drivetrain.goToPosition(() -> getClosestBranch(drivetrain.getPose(), robotMode.get()), () -> robotMode.get() == RobotMode.ALGAE_NET).until(
                                        () -> !robotMode.get().isPlacingMode() && grabber.isGamePieceDetected()).andThen(
                                        doneAligning(robotMode, xVelocitySupplier, yVelocitySupplier, angularVelocitySupplier)),
                                superStructure.followPositions(
                                                () -> Math.min(robotMode.get().getElevatorHeight(), Constants.ArmPositions.ELEVATOR_MAX_MOVING_HEIGHT),
                                                () -> MathUtil.clamp(robotMode.get().getShoulderAngle(),
                                                        Units.degreesToRadians(90.0) - Constants.ArmPositions.SHOULDER_MAX_MOVING_OFFSET,
                                                        Units.degreesToRadians(90.0) + Constants.ArmPositions.SHOULDER_MAX_MOVING_OFFSET),
                                                () -> robotMode.get().getWristAngle())
                                        .until(() -> drivetrain.getPose().getTranslation().getDistance(
                                                getClosestBranch(drivetrain.getPose(), robotMode.get()).getTranslation()
                                        ) < Units.feetToMeters(1.0))
                                        .andThen(superStructure.followPositions(robotMode))
                        ),
                        drivetrain.drive(xVelocitySupplier, yVelocitySupplier, angularVelocitySupplier)
                                .alongWith(superStructure.followPositions(robotMode)),
                        () -> shouldAlignToReef(robotMode.get())
                )
                .alongWith(
                        Commands.either(Commands.waitUntil(() -> readyToPlace.getAsBoolean() && superStructure.atPosition(robotMode.get()) &&
                                        (drivetrain.atTargetPosition() || !shouldAlignToReef(robotMode.get()) || robotMode.get() == RobotMode.ALGAE_PROCESSOR)).andThen(
                                        Commands.either(grabber.placeAlgae(), Commands.either(grabber.placeCoralSlow(),
                                                Commands.either(grabber.placeCoralL4(), grabber.placeCoral(), () -> robotMode.get() == RobotMode.CORAL_LEVEL_4),
                                                () -> robotMode.get() == RobotMode.CORAL_LEVEL_1), () -> robotMode.get().getGamepiece() == Gamepiece.ALGAE)),
                                Commands.either(grabber.grabAlgae(), grabber.grabCoral(), () -> robotMode.get().getGamepiece() == Gamepiece.ALGAE),
                                () -> robotMode.get().isPlacingMode())
                );
    }

    public Command doneAligning(Supplier<RobotMode> robotMode,
                                DoubleSupplier xVelocitySupplier,
                                DoubleSupplier yVelocitySupplier, DoubleSupplier angularVelocitySupplier) {
        return Commands.either(drivetrain.drive(() -> 0.0, () -> MathUtil.clamp(yVelocitySupplier.getAsDouble(), -Constants.NET_MAX_SPEED, Constants.NET_MAX_SPEED), () -> 0.0),
                drivetrain.drive(xVelocitySupplier, yVelocitySupplier, angularVelocitySupplier),
                () -> robotMode.get() == RobotMode.ALGAE_NET);
    }

    public Command alignToCoralStation(BooleanSupplier coralInTheWay) {
        return Commands.deadline(
                grabber.grabCoral(),
                superStructure.followPositions(() -> coralInTheWay.getAsBoolean() ? RobotMode.CORAL_STATION_WITH_CORAL.getElevatorHeight() : RobotMode.CORAL_STATION.getElevatorHeight(),
                        () -> coralInTheWay.getAsBoolean() ? RobotMode.CORAL_STATION_WITH_CORAL.getShoulderAngle() : RobotMode.CORAL_STATION.getShoulderAngle(),
                        () -> coralInTheWay.getAsBoolean() ? RobotMode.CORAL_STATION_WITH_CORAL.getWristAngle() : RobotMode.CORAL_STATION.getWristAngle()),
                Commands.either(
                        Commands.deferredProxy(() -> drivetrain.goToPosition(() ->
                                getClosestCoralStationPosition(drivetrain.getPose()), () -> false)),
                        Commands.none(),
                        () -> SmartDashboard.getBoolean("Auto Coral Station", true)
                ));
    }
}