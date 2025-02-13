package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.grabber.Grabber;
import frc.robot.subsystems.led.Led;

import java.util.List;

import static edu.wpi.first.wpilibj2.command.Commands.runEnd;

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

    /**
     * Runs the grabber backwards until it doesn't have the game piece anymore to release the game piece
     */
    public Command releaseGamePiece() {
        return runEnd(() -> {
            grabber.withVoltage(-1.0);
        }, grabber::stop).until(grabber::getGamePieceNotDetected);
    }

    /**
     * Tells the Elevator and Arm to go to the required positions to intake a coral from the coral station
     */
    public Command goToCoralStationAndIntake() {
        return superStructure.goToPositions(Constants.ArmPositions.CORAL_STATION_ELEVATOR_HEIGHT,
                        Constants.ArmPositions.CORAL_STATION_SHOULDER_ANGLE,
                        Constants.ArmPositions.CORAL_STATION_WRIST_ANGLE)
                .andThen(grabber.withVoltageUntilDetected(-1.0));
    }

    /**
     * Takes in the closest branch to the robot and creates a PathPlanner path to that pose
     *
     * @param branchPose requires a Pose2d of the branch closest to the robot's current position
     */
    public static Command pathToPoseCommand(Pose2d branchPose) {
        PathConstraints constraints = new PathConstraints(5.0,
                5.0, 2 * Math.PI,
                4 * Math.PI);

        Command alignToBranch = AutoBuilder.pathfindToPose(
                branchPose,
                constraints,
                0.0);

        return alignToBranch;
    }
}