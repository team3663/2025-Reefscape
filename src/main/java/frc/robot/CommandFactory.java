package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.grabber.Grabber;
import frc.robot.subsystems.led.Led;

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
     * Tells the Elevator and Arm to go to the required positions to place on level 4
     */
    public Command goToL4() {
        return Commands.parallel(
                superStructure.goToPositions(Constants.ScoringPositions.CORAL_LEVEL_4_ELEVATOR_HEIGHT,
                        Constants.ScoringPositions.CORAL_LEVEL_4_SHOULDER_ANGLE,
                        Constants.ScoringPositions.CORAL_LEVEL_4_WRIST_ANGLE));
    }

    /**
     * Tells the Elevator and Arm to go to the required positions to place on level 3
     */
    public Command goToL3() {
        return Commands.parallel(
                superStructure.goToPositions(Constants.ScoringPositions.CORAL_LEVEL_3_ELEVATOR_HEIGHT,
                        Constants.ScoringPositions.CORAL_LEVEL_3_SHOULDER_ANGLE,
                        Constants.ScoringPositions.CORAL_LEVEL_3_WRIST_ANGLE));
    }

    /**
     * Tells the Elevator and Arm to go to the required positions to place on level 2
     */
    public Command goToL2() {
        return Commands.parallel(
                superStructure.goToPositions(Constants.ScoringPositions.CORAL_LEVEL_2_ELEVATOR_HEIGHT,
                        Constants.ScoringPositions.CORAL_LEVEL_2_SHOULDER_ANGLE,
                        Constants.ScoringPositions.CORAL_LEVEL_2_WRIST_ANGLE));
    }

    /**
     * Tells the Elevator and Arm to go to the required positions to place on level 1
     */
    public Command goToL1() {
        return Commands.parallel(
                superStructure.goToPositions(Constants.ScoringPositions.CORAL_LEVEL_1_ELEVATOR_HEIGHT,
                        Constants.ScoringPositions.CORAL_LEVEL_1_SHOULDER_ANGLE,
                        Constants.ScoringPositions.CORAL_LEVEL_1_WRIST_ANGLE));
    }

    /**
     * Tells the Elevator and Arm to go to the required positions to place in the Net
     */
    public Command goToNet() {
        return Commands.parallel(
                superStructure.goToPositions(Constants.ScoringPositions.ALGAE_NET_ELEVATOR_HEIGHT,
                        Constants.ScoringPositions.ALGAE_NET_SHOULDER_ANGLE,
                        Constants.ScoringPositions.ALGAE_NET_WRIST_ANGLE));
    }

    /**
     * Tells the Elevator and Arm to go to the required positions to place in the Processor
     */
    public Command goToProcessor() {
        return Commands.parallel(
                superStructure.goToPositions(Constants.ScoringPositions.ALGAE_PROCESSOR_ELEVATOR_HEIGHT,
                        Constants.ScoringPositions.ALGAE_PROCESSOR_SHOULDER_ANGLE,
                        Constants.ScoringPositions.ALGAE_PROCESSOR_WRIST_ANGLE));
    }

    /**
     * Tells the Elevator and Arm to go to the required positions to remove the upper algae
     */
    public Command goToRemoveUpper() {
        return Commands.parallel(
                superStructure.goToPositions(Constants.ScoringPositions.REMOVE_ALGAE_UPPER_ELEVATOR_HEIGHT,
                        Constants.ScoringPositions.REMOVE_ALGAE_UPPER_SHOULDER_ANGLE,
                        Constants.ScoringPositions.REMOVE_ALGAE_UPPER_WRIST_ANGLE));
    }

    /**
     * Tells the Elevator and Arm to go to the required positions to remove the lower algae
     */
    public Command goToRemoveLower() {
        return Commands.parallel(
                superStructure.goToPositions(Constants.ScoringPositions.REMOVE_ALGAE_LOWER_ELEVATOR_HEIGHT,
                        Constants.ScoringPositions.REMOVE_ALGAE_LOWER_SHOULDER_ANGLE,
                        Constants.ScoringPositions.REMOVE_ALGAE_LOWER_WRIST_ANGLE));
    }
}