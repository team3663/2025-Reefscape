package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.grabber.Grabber;
import frc.robot.subsystems.led.Led;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

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

    private boolean atTargetPosition(double currentPosition, double targetPosition, double threshold) {
        return Math.abs(currentPosition - targetPosition) <= threshold;
    }

    /**
     * Tells the super Structure where to go based on the Robot Mode
     *
     * @param robotMode A supplier for the current RobotMode so it knows where to go
     * @return The command to follow the current position based on the Robot Mode
     */
    public Command goToPosition(Supplier<RobotMode> robotMode) {
        DoubleSupplier targetRobotModeElevatorHeight = () -> switch (robotMode.get()) {
            case CORAL_LEVEL_1 -> Constants.ArmPositions.CORAL_LEVEL_1_ELEVATOR_HEIGHT;
            case CORAL_LEVEL_2 -> Constants.ArmPositions.CORAL_LEVEL_2_ELEVATOR_HEIGHT;
            case CORAL_LEVEL_3 -> Constants.ArmPositions.CORAL_LEVEL_3_ELEVATOR_HEIGHT;
            case CORAL_LEVEL_4 -> Constants.ArmPositions.CORAL_LEVEL_4_ELEVATOR_HEIGHT;
            case ALGAE_PROCESSOR -> Constants.ArmPositions.ALGAE_PROCESSOR_ELEVATOR_HEIGHT;
            case ALGAE_NET -> Constants.ArmPositions.ALGAE_NET_ELEVATOR_HEIGHT;
            case ALGAE_REMOVE_LOWER -> Constants.ArmPositions.REMOVE_ALGAE_LOWER_ELEVATOR_HEIGHT;
            case ALGAE_REMOVE_UPPER -> Constants.ArmPositions.REMOVE_ALGAE_UPPER_ELEVATOR_HEIGHT;
        };
        DoubleSupplier targetElevatorHeight = () -> {
            if (atTargetPosition(elevator.getPosition(), targetRobotModeElevatorHeight.getAsDouble(), Elevator.POSITION_THRESHOLD * 2) ||
                    atTargetPosition(arm.getShoulderPosition(), Constants.ArmPositions.SHOULDER_SAFE_ANGLE, Constants.ArmPositions.SHOULDER_SAFE_THRESHOLD))
                return targetRobotModeElevatorHeight.getAsDouble();
            else return elevator.getPosition();
        };
        DoubleSupplier targetShoulderAngle = () -> {
            if (!atTargetPosition(elevator.getPosition(), targetRobotModeElevatorHeight.getAsDouble(), Elevator.POSITION_THRESHOLD))
                return Constants.ArmPositions.SHOULDER_SAFE_ANGLE;
            switch (robotMode.get()) {
                case CORAL_LEVEL_1 -> { return Constants.ArmPositions.CORAL_LEVEL_1_SHOULDER_ANGLE;}
                case CORAL_LEVEL_2 -> { return Constants.ArmPositions.CORAL_LEVEL_2_SHOULDER_ANGLE;}
                case CORAL_LEVEL_3 -> { return Constants.ArmPositions.CORAL_LEVEL_3_SHOULDER_ANGLE;}
                case CORAL_LEVEL_4 -> { return Constants.ArmPositions.CORAL_LEVEL_4_SHOULDER_ANGLE;}
                case ALGAE_PROCESSOR -> { return Constants.ArmPositions.ALGAE_PROCESSOR_SHOULDER_ANGLE;}
                case ALGAE_NET -> { return Constants.ArmPositions.ALGAE_NET_SHOULDER_ANGLE;}
                case ALGAE_REMOVE_LOWER -> { return Constants.ArmPositions.REMOVE_ALGAE_LOWER_SHOULDER_ANGLE;}
                case ALGAE_REMOVE_UPPER -> { return Constants.ArmPositions.REMOVE_ALGAE_UPPER_SHOULDER_ANGLE;}
            }
            return 0.0;
        };
        DoubleSupplier targetWristAngle = () -> switch (robotMode.get()) {
            case CORAL_LEVEL_1 -> Constants.ArmPositions.CORAL_LEVEL_1_WRIST_ANGLE;
            case CORAL_LEVEL_2 -> Constants.ArmPositions.CORAL_LEVEL_2_WRIST_ANGLE;
            case CORAL_LEVEL_3 -> Constants.ArmPositions.CORAL_LEVEL_3_WRIST_ANGLE;
            case CORAL_LEVEL_4 -> Constants.ArmPositions.CORAL_LEVEL_4_WRIST_ANGLE;
            case ALGAE_PROCESSOR -> Constants.ArmPositions.ALGAE_PROCESSOR_WRIST_ANGLE;
            case ALGAE_NET -> Constants.ArmPositions.ALGAE_NET_WRIST_ANGLE;
            case ALGAE_REMOVE_LOWER -> Constants.ArmPositions.REMOVE_ALGAE_LOWER_WRIST_ANGLE;
            case ALGAE_REMOVE_UPPER -> Constants.ArmPositions.REMOVE_ALGAE_UPPER_WRIST_ANGLE;
        };

        return superStructure.followPositions(targetElevatorHeight, targetShoulderAngle, targetWristAngle);
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
     * Tells the Elevator and Arm to go to the required positions to place on level 4
     */
    public Command goToL4() {
        return superStructure.goToPositions(Constants.ArmPositions.CORAL_LEVEL_4_ELEVATOR_HEIGHT,
                Constants.ArmPositions.CORAL_LEVEL_4_SHOULDER_ANGLE,
                Constants.ArmPositions.CORAL_LEVEL_4_WRIST_ANGLE);
    }

    /**
     * Tells the Elevator and Arm to go to the required positions to place on level 3
     */
    public Command goToL3() {
        return superStructure.goToPositions(Constants.ArmPositions.CORAL_LEVEL_3_ELEVATOR_HEIGHT,
                Constants.ArmPositions.CORAL_LEVEL_3_SHOULDER_ANGLE,
                Constants.ArmPositions.CORAL_LEVEL_3_WRIST_ANGLE);
    }

    /**
     * Tells the Elevator and Arm to go to the required positions to place on level 2
     */
    public Command goToL2() {
        return superStructure.goToPositions(Constants.ArmPositions.CORAL_LEVEL_2_ELEVATOR_HEIGHT,
                Constants.ArmPositions.CORAL_LEVEL_2_SHOULDER_ANGLE,
                Constants.ArmPositions.CORAL_LEVEL_2_WRIST_ANGLE);
    }

    /**
     * Tells the Elevator and Arm to go to the required positions to place on level 1
     */
    public Command goToL1() {
        return superStructure.goToPositions(Constants.ArmPositions.CORAL_LEVEL_1_ELEVATOR_HEIGHT,
                Constants.ArmPositions.CORAL_LEVEL_1_SHOULDER_ANGLE,
                Constants.ArmPositions.CORAL_LEVEL_1_WRIST_ANGLE);
    }

    /**
     * Tells the Elevator and Arm to go to the required positions to place in the Net
     */
    public Command goToNet() {
        return superStructure.goToPositions(Constants.ArmPositions.ALGAE_NET_ELEVATOR_HEIGHT,
                Constants.ArmPositions.ALGAE_NET_SHOULDER_ANGLE,
                Constants.ArmPositions.ALGAE_NET_WRIST_ANGLE);
    }

    /**
     * Tells the Elevator and Arm to go to the required positions to place in the Processor
     */
    public Command goToProcessor() {
        return superStructure.goToPositions(Constants.ArmPositions.ALGAE_PROCESSOR_ELEVATOR_HEIGHT,
                Constants.ArmPositions.ALGAE_PROCESSOR_SHOULDER_ANGLE,
                Constants.ArmPositions.ALGAE_PROCESSOR_WRIST_ANGLE);
    }

    /**
     * Tells the Elevator and Arm to go to the required positions to remove the upper algae
     */
    public Command goToRemoveUpper() {
        return superStructure.goToPositions(Constants.ArmPositions.REMOVE_ALGAE_UPPER_ELEVATOR_HEIGHT,
                Constants.ArmPositions.REMOVE_ALGAE_UPPER_SHOULDER_ANGLE,
                Constants.ArmPositions.REMOVE_ALGAE_UPPER_WRIST_ANGLE);
    }

    /**
     * Tells the Elevator and Arm to go to the required positions to remove the lower algae
     */
    public Command goToRemoveLower() {
        return superStructure.goToPositions(Constants.ArmPositions.REMOVE_ALGAE_LOWER_ELEVATOR_HEIGHT,
                Constants.ArmPositions.REMOVE_ALGAE_LOWER_SHOULDER_ANGLE,
                Constants.ArmPositions.REMOVE_ALGAE_LOWER_WRIST_ANGLE);
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
}