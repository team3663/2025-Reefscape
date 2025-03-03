package frc.robot;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.MathUtil;
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

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

@Logged
public class SuperStructure extends SubsystemBase {

    @NotLogged
    private final Elevator elevator;
    @NotLogged
    private final Arm arm;
    private final BooleanSupplier haveAlgae;

    private final Mechanism2d mechanism;
    private final MechanismLigament2d targetElevatorMechanism;
    private final MechanismLigament2d targetShoulderMechanism;
    private final MechanismLigament2d targetWristMechanism;
    private final MechanismLigament2d currentElevatorMechanism;
    private final MechanismLigament2d currentShoulderMechanism;
    private final MechanismLigament2d currentWristMechanism;

    public SuperStructure(Elevator elevator, Arm arm, BooleanSupplier haveAlgae) {
        this.elevator = elevator;
        this.arm = arm;
        this.haveAlgae = haveAlgae;

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

    private double getMinimumAllowableElevatorPosition(double shoulderAngle, double wristAngle) {
        return Math.max(
                elevator.getConstants().minimumPosition(),
                Math.max(
                        -(arm.getConstants().shoulderLength() + Constants.SHOULDER_BUFFER) * Math.sin(shoulderAngle),
                        -arm.getConstants().shoulderLength() * Math.sin(shoulderAngle) - (arm.getConstants().wristLength() + Constants.WRIST_BUFFER)
                                * Math.sin(shoulderAngle + wristAngle)));
    }

    private double getMinimumAllowableWristAngle(double elevatorPosition, double shoulderAngle) {
        double length = (arm.getConstants().shoulderLength() * Math.sin(shoulderAngle) + elevatorPosition - Constants.WRIST_BUFFER) / arm.getConstants().wristLength();
        if (Math.abs(length) <= 1.0) {
            return Math.max(arm.getConstants().minimumWristAngle(), -shoulderAngle - Math.asin(length));
        }
        return arm.getConstants().minimumWristAngle();
    }

    private double getMinimumAllowableShoulderAngle(double currentElevatorPos, double targetElevatorPos) {
        double minimumAllowablePos = arm.getConstants().minimumShoulderAngle();
        if (Math.abs((currentElevatorPos - Constants.SHOULDER_BUFFER) / arm.getConstants().shoulderLength()) <= 1) {
            minimumAllowablePos = Math.max(minimumAllowablePos, -Math.asin((currentElevatorPos - Constants.SHOULDER_BUFFER) / arm.getConstants().shoulderLength()));
        }
        if (Math.abs((targetElevatorPos - Constants.SHOULDER_BUFFER) / arm.getConstants().shoulderLength()) <= 1) {
            minimumAllowablePos = Math.max(minimumAllowablePos, -Math.asin((targetElevatorPos - Constants.SHOULDER_BUFFER) / arm.getConstants().shoulderLength()));
        }
        return minimumAllowablePos;
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
                arm.followPositions(
                        () -> {
                            double pos = shoulderHaveAlgaePosition(
                                    Math.max(shoulderPosition.getAsDouble(),
                                            getMinimumAllowableShoulderAngle(elevator.getPosition(), elevatorPosition.getAsDouble())));

                            if (!elevator.atPosition(elevatorPosition.getAsDouble(), Units.inchesToMeters(4.0)))
                                return MathUtil.clamp(pos, Constants.ArmPositions.SHOULDER_SAFE_ANGLE - Constants.ArmPositions.SHOULDER_SAFE_THRESHOLD
                                        + Constants.ArmPositions.SHOULDER_SAFE_BUFFER, Constants.ArmPositions.SHOULDER_SAFE_ANGLE +
                                        Constants.ArmPositions.SHOULDER_SAFE_THRESHOLD - Constants.ArmPositions.SHOULDER_SAFE_BUFFER);

                            return pos;
                        },
                        () -> Math.max(wristPosition.getAsDouble(), getMinimumAllowableWristAngle(elevator.getPosition(), arm.getShoulderPosition()))),
                elevator.followPosition(() -> {
                    if (!elevator.atPosition(elevatorPosition.getAsDouble()) &&
                            !arm.shoulderAtPosition(Constants.ArmPositions.SHOULDER_SAFE_ANGLE, Constants.ArmPositions.SHOULDER_SAFE_THRESHOLD)) {
                        return elevator.getTargetPosition();
                    }

                    return Math.max(elevatorPosition.getAsDouble(), getMinimumAllowableElevatorPosition(arm.getShoulderPosition(), arm.getWristPosition()));
                }),
                run(() -> {
                }));
    }

    public Command goToPositions(double elevatorPosition, double shoulderPosition, double wristPosition) {
        return followPositions(() -> elevatorPosition, () -> shoulderPosition, () -> wristPosition)
                .until(() -> elevator.atPosition(elevatorPosition) && arm.atPositions(shoulderPosition, wristPosition));
    }

    private double shoulderHaveAlgaePosition(double shoulderPosition) {
        return haveAlgae.getAsBoolean() ? Math.min(shoulderPosition, Constants.ArmPositions.SHOULDER_ALGAE_MAX_ANGLE) : shoulderPosition;
    }

    /**
     * Tells the super Structure where to go based on the Robot Mode
     *
     * @param robotMode A supplier for the current RobotMode so it knows where to go
     * @return The command to follow the current position based on the Robot Mode
     */
    public Command followPositions(Supplier<RobotMode> robotMode) {
        DoubleSupplier targetElevatorHeight = () -> robotMode.get().getElevatorHeight();
        DoubleSupplier targetShoulderAngle = () -> robotMode.get().getShoulderAngle();
        DoubleSupplier targetWristAngle = () -> robotMode.get().getWristAngle();

        return this.followPositions(targetElevatorHeight, targetShoulderAngle, targetWristAngle);
    }

    public Command goToPositions(RobotMode robotMode) {
        return followPositions(() -> robotMode).until(this::atTargetPositions);
    }

    public Command goToDefaultPositions() {
        return goToPositions(Constants.ArmPositions.ELEVATOR_DEFAULT_POSITION, Constants.ArmPositions.SHOULDER_DEFAULT_ANGLE, Constants.ArmPositions.WRIST_DEFAULT_ANGLE);
    }

    public Command zero() {
        return Commands.parallel(
                arm.zeroWrist(),
                elevator.zero(),
                runOnce(() -> {
                })
        );
    }

    public Command resetPositions() {
        return Commands.parallel(arm.resetWristPositionToDefault(), elevator.resetPosition());
    }
}