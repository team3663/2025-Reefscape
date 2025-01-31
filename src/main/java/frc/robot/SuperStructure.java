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

@Logged
public class SuperStructure extends SubsystemBase {
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
        targetShoulderMechanism.setAngle(Units.radiansToDegrees(arm.getTargetShoulderPosition()) + 90.0);
        targetWristMechanism.setAngle(Units.radiansToDegrees(arm.getTargetWristPosition()));

        currentElevatorMechanism.setLength(elevator.getPosition());
        currentShoulderMechanism.setAngle(Units.radiansToDegrees(arm.getShoulderPosition()) + 90.0);
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

    public Command followPositions(DoubleSupplier elevatorPosition, DoubleSupplier shoulderPosition, DoubleSupplier wristPosition) {
        return Commands.parallel(
                arm.followPositions(shoulderPosition, wristPosition),
                elevator.followPosition(elevatorPosition));
    }

    public Command goToPositions(double elevatorPosition, double shoulderPosition, double wristPosition) {
        return runEnd(
                () -> {
                        elevator.goToPosition(elevatorPosition);
                        arm.goToPositions(shoulderPosition, wristPosition);
                }, this::stop
        );
    }

    public Command resetPositions() {
        return runOnce(
                () -> {
                    elevator.resetPosition();
                    arm.resetPositions();
                }
        );
    }
}