package frc.robot;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.elevator.Elevator;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

@Logged
public class SuperStructure extends SubsystemBase {
    private static final double ELEVATOR_DEFAULT_POSITION = 0;
    private static final double SHOULDER_DEFAULT_ANGLE = Units.degreesToRadians(90);
    private static final double WRIST_DEFAULT_ANGLE = 0;

    @NotLogged
    private final Elevator elevator;
    @NotLogged
    private final Arm arm;

    private final Mechanism2d mechanism;
    private final MechanismLigament2d targetElevatorMechanism;
    private final MechanismLigament2d targetShoulderMechanism;
    private final MechanismLigament2d targetWristMechanism;
    private final MechanismLigament2d currentElevatorMechanism;
    private final MechanismLigament2d currentShoulderMechanism;
    private final MechanismLigament2d currentWristMechanism;

    public SuperStructure(Elevator elevator, Arm arm) {
        this.elevator = elevator;
        this.arm = arm;

        mechanism = new Mechanism2d(
                2.0 * (arm.getConstants().shoulderLength() + arm.getConstants().wristLength()),
                elevator.getConstants().maximumPosition() + arm.getConstants().shoulderLength() + arm.getConstants().wristLength());
        var targetRoot = mechanism.getRoot(
                "Target",
                arm.getConstants().shoulderLength() + arm.getConstants().wristLength(),
                0.0);
        targetElevatorMechanism = new MechanismLigament2d(
                "TargetElevator",
                0.0, 90.0,
                10, new Color8Bit(Color.kDarkOrange));
        targetRoot.append(targetElevatorMechanism);
        targetShoulderMechanism = new MechanismLigament2d(
                "TargetShoulder",
                arm.getConstants().shoulderLength(), 0.0,
                10, new Color8Bit(Color.kOrange));
        targetElevatorMechanism.append(targetShoulderMechanism);
        targetWristMechanism = new MechanismLigament2d(
                "TargetWrist",
                arm.getConstants().wristLength(), 0.0,
                10, new Color8Bit(Color.kGreen));
        targetShoulderMechanism.append(targetWristMechanism);

        var currentRoot = mechanism.getRoot(
                "Current",
                arm.getConstants().shoulderLength() + arm.getConstants().wristLength(),
                0.0);
        currentElevatorMechanism = new MechanismLigament2d(
                "CurrentElevator",
                0.0, 90.0,
                9, new Color8Bit(Color.kDarkBlue));
        currentRoot.append(currentElevatorMechanism);
        currentShoulderMechanism = new MechanismLigament2d(
                "CurrentShoulder",
                arm.getConstants().shoulderLength(), 0.0,
                9, new Color8Bit(Color.kBlue));
        currentElevatorMechanism.append(currentShoulderMechanism);
        currentWristMechanism = new MechanismLigament2d(
                "CurrentWrist",
                arm.getConstants().wristLength(), 0.0,
                9, new Color8Bit(Color.kLightBlue));
        currentShoulderMechanism.append(currentWristMechanism);

        SmartDashboard.putData("Superstructure", mechanism);
    }

    @Override
    public void periodic() {
        targetElevatorMechanism.setLength(elevator.getTargetPosition());
        targetShoulderMechanism.setAngle(Units.radiansToDegrees(arm.getTargetShoulderPosition()) - 90.0);
        targetWristMechanism.setAngle(Units.radiansToDegrees(arm.getTargetWristPosition()));

        currentElevatorMechanism.setLength(elevator.getPosition());
        currentShoulderMechanism.setAngle(Units.radiansToDegrees(arm.getShoulderPosition()) - 90.0);
        currentWristMechanism.setAngle(Units.radiansToDegrees(arm.getWristPosition()));
    }

    public Command stop() {
        return runOnce(
                () -> {
                    elevator.stop();
                    arm.stop();
                }
        );
    }

    public boolean atTargetPositions() {
        return elevator.atTargetPosition() && arm.atTargetPositions();
    }

    public Command followPositions(DoubleSupplier elevatorPosition, DoubleSupplier shoulderPosition, DoubleSupplier wristPosition) {
        return Commands.parallel(
                arm.followPositions(shoulderPosition, wristPosition),
                elevator.followPosition(elevatorPosition),
                run(() -> {}));
    }

    public Command goToPositions(double elevatorPosition, double shoulderPosition, double wristPosition) {
        return Commands.parallel(
                elevator.goToPosition(elevatorPosition),
                arm.goToPositions(shoulderPosition, wristPosition),
                runOnce(() -> {}));
    }

    /**
     * Tells the super Structure where to go based on the Robot Mode
     *
     * @param robotMode A supplier for the current RobotMode so it knows where to go
     * @return The command to follow the current position based on the Robot Mode
     */
    public Command followPositions(Supplier<RobotMode> robotMode) {
        DoubleSupplier targetElevatorHeight = () -> {
            if (elevator.atPosition(robotMode.get().getElevatorHeight(), Elevator.POSITION_THRESHOLD) ||
                    arm.shoulderAtPosition(Constants.ArmPositions.SHOULDER_SAFE_ANGLE, Constants.ArmPositions.SHOULDER_SAFE_THRESHOLD))
                return robotMode.get().getElevatorHeight();
            else return elevator.getTargetPosition();
        };
        DoubleSupplier targetShoulderAngle = () -> {
            if (!elevator.atPosition(robotMode.get().getElevatorHeight(), Elevator.POSITION_THRESHOLD))
                return Constants.ArmPositions.SHOULDER_SAFE_ANGLE;
            return robotMode.get().getShoulderAngle();
        };
        DoubleSupplier targetWristAngle = () -> robotMode.get().getWristAngle();

        return this.followPositions(targetElevatorHeight, targetShoulderAngle, targetWristAngle);
    }

    public Command goToDefaultPositions() {
        return goToPositions(ELEVATOR_DEFAULT_POSITION, SHOULDER_DEFAULT_ANGLE, WRIST_DEFAULT_ANGLE);
    }

    public Command zero() {
        return Commands.parallel(
                arm.zeroWrist(),
                elevator.zero(),
                runOnce(() -> {
                })
        );
    }
}