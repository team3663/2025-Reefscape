package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
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

    public Pose2d getClosestBranch(Pose2d robotPose) {
        var alliance = DriverStation.getAlliance();

        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red && !Constants.IS_ANDYMARK) {
            return robotPose.nearest(Constants.RED_WELDED_BRANCH_POSES);
        } else if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red && Constants.IS_ANDYMARK) {
            return robotPose.nearest(Constants.RED_ANDYMARK_BRANCH_POSES);
        } else if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Blue && !Constants.IS_ANDYMARK) {
            return robotPose.nearest(Constants.BLUE_WELDED_BRANCH_POSES);
        } else {
            return robotPose.nearest(Constants.BLUE_ANDYMARK_BRANCH_POSES);
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

    public Command alignToReef(Supplier<RobotMode> robotMode,
                               BooleanSupplier readyToPlace,
                               DoubleSupplier xVelocitySupplier,
                               DoubleSupplier yVelocitySupplier, DoubleSupplier angularVelocitySupplier) {
        return Commands.either(
                        Commands.parallel(
                                drivetrain.goToPosition(() -> getClosestBranch(drivetrain.getPose()).plus(Constants.ROBOT_REEF_OFFSET), false),
                                superStructure.followPositions(
                                                () -> Math.min(robotMode.get().getElevatorHeight(), Constants.ArmPositions.ELEVATOR_MAX_MOVING_HEIGHT),
                                                () -> MathUtil.clamp(robotMode.get().getShoulderAngle(),
                                                        Units.degreesToRadians(90.0) - (Constants.ArmPositions.SHOULDER_MAX_MOVING_ANGLE - Units.degreesToRadians(90.0)),
                                                        Constants.ArmPositions.SHOULDER_MAX_MOVING_ANGLE),
                                                () -> robotMode.get().getWristAngle())
                                        .until(() -> drivetrain.getPose().getTranslation().getDistance(
                                                getClosestBranch(drivetrain.getPose()).plus(Constants.ROBOT_REEF_OFFSET).getTranslation()
                                        ) < Units.feetToMeters(1.0))
                                        .andThen(superStructure.followPositions(robotMode))
                        ),
                        drivetrain.drive(xVelocitySupplier, yVelocitySupplier, angularVelocitySupplier)
                                .alongWith(superStructure.followPositions(robotMode)),
                        () -> SmartDashboard.getBoolean("Auto Reef", true)
                )
                .alongWith(
                        Commands.either(Commands.waitUntil(readyToPlace).andThen(
                                Commands.either(grabber.placeAlgae(), Commands.either(grabber.placeCoralSlow(), grabber.placeCoral(), () -> robotMode.get() == RobotMode.CORAL_LEVEL_1), () -> robotMode.get().getGamepiece() == Gamepiece.ALGAE)),
                                Commands.either(grabber.grabAlgae(), grabber.grabCoral(), () -> robotMode.get().getGamepiece() == Gamepiece.ALGAE),
                                () -> robotMode.get().isPlacingMode())
                );
    }

    public Command alignToCoralStation() {
        return Commands.deadline(
                grabber.grabCoral(),
                superStructure.followPositions(() -> Constants.ArmPositions.CORAL_STATION_ELEVATOR_HEIGHT,
                        () -> Constants.ArmPositions.CORAL_STATION_SHOULDER_ANGLE,
                        () -> Constants.ArmPositions.CORAL_STATION_WRIST_ANGLE),
                Commands.either(
                        Commands.deferredProxy(() -> drivetrain.goToPosition(() ->
                                getClosestCoralStationPosition(drivetrain.getPose()).plus(Constants.ROBOT_CORAL_STATION_OFFSET), true)),
                        Commands.none(),
                        () -> SmartDashboard.getBoolean("Auto Coral Station", true)
                ));
    }

    public Command grabCoral() {
        return Commands.sequence(
                superStructure.goToPositions(RobotMode.CORAL_STATION),
                grabber.grabCoral(),
                superStructure.goToDefaultPositions()
        );
    }

    public Command placeCoral() {
        return grabber.placeCoral().withDeadline(
                Commands.waitUntil(grabber::getGamePieceNotDetected)
                        .andThen(Commands.waitSeconds(0.25))
        ).andThen(superStructure.goToDefaultPositions());
    }
}