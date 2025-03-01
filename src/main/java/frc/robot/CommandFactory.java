package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
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

    /**
     * Runs the grabber backwards until it doesn't have the game piece anymore to release the game piece
     */
    public Command releaseGamePiece(Supplier<RobotMode> robotMode) {
        return grabber.followVoltage(() -> robotMode.get().isRunGrabberReverse() ? -6.0 : 6.0);
    }

    public Command alignToReef(Supplier<RobotMode> robotMode, DoubleSupplier xVelocitySupplier,
                               DoubleSupplier yVelocitySupplier, DoubleSupplier angularVelocitySupplier) {
        return Commands.either(
                drivetrain.goToPosition(() -> getClosestBranch(drivetrain.getPose()).plus(Constants.ROBOT_REEF_OFFSET), false),
                drivetrain.drive(xVelocitySupplier, yVelocitySupplier, angularVelocitySupplier),
                () -> SmartDashboard.getBoolean("Auto Reef", true)
        ).alongWith(superStructure.followPositions(
                () -> {
                    if (drivetrain.isNotMoving()) return robotMode.get().getElevatorHeight();
                    else return Math.min(robotMode.get().getElevatorHeight(), Constants.ArmPositions.ELEVATOR_MAX_MOVING_HEIGHT);
                }, () -> {
                    if (drivetrain.isNotMoving()) return robotMode.get().getShoulderAngle();
                    else return MathUtil.clamp(robotMode.get().getShoulderAngle(),
                                Constants.ArmPositions.SHOULDER_MAX_MOVING_ANGLE - Units.degreesToRadians(90), Constants.ArmPositions.SHOULDER_MAX_MOVING_ANGLE);
                }, () -> robotMode.get().getWristAngle()));
    }

    public Command alignToCoralStation() {
        Debouncer[] debouncerHolder = new Debouncer[1];
        return Commands.parallel(
                        superStructure.followPositions(() -> Constants.ArmPositions.CORAL_STATION_ELEVATOR_HEIGHT,
                                () -> Constants.ArmPositions.CORAL_STATION_SHOULDER_ANGLE,
                                () -> Constants.ArmPositions.CORAL_STATION_WRIST_ANGLE),
                        grabber.followVoltage(() -> 6.0),
                        Commands.either(
                                Commands.deferredProxy(() -> drivetrain.goToPosition(() ->
                                        getClosestCoralStationPosition(drivetrain.getPose()).plus(Constants.ROBOT_CORAL_STATION_OFFSET), true)),
                                Commands.idle(),
                                () -> SmartDashboard.getBoolean("Auto Coral Station", true)
                        ))
                .withDeadline(
                        Commands.sequence(
                                Commands.runOnce(() -> debouncerHolder[0] = new Debouncer(0.04)),
                                Commands.waitUntil(() -> debouncerHolder[0].calculate(grabber.isGamePieceDetected()))
                        ));
    }

    public Command grabCoral() {
        return grabber.followVoltage(() -> 6.0).withDeadline(Commands.waitUntil(grabber::isGamePieceDetected).andThen(Commands.waitSeconds(0.04)));
    }

    public Command placeCoral() {
        return grabber.followVoltage(() -> 6.0).until(grabber::getGamePieceNotDetected);
    }
}