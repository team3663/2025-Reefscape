package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shoulder.Shoulder;
import frc.robot.subsystems.elevator.Elevator;

import java.util.function.DoubleSupplier;

public class SuperStructure extends SubsystemBase {
    private final Elevator elevator;
    private final Shoulder shoulder;

    public SuperStructure(Elevator elevator, Shoulder shoulder) {
        this.elevator = elevator;
        this.shoulder = shoulder;
    }

    private boolean armNotCollide(double elevatorCurrentPos, double armCurrentPos, double elevatorTargetPos, double armTargetPos) {
        return true;
    }

    private boolean armNotCollide(double elevatorTargetPos, double armTargetPos) {
        return armNotCollide(elevator.getPosition(), shoulder.getPosition(), elevatorTargetPos, armTargetPos);
    }

    public Command stop() {
        return runOnce(
                () -> {
                    elevator.stop();
                    shoulder.stop();
                }
        );
    }

    public Command followPositions(DoubleSupplier elevatorPosition, DoubleSupplier armPosition) {
        return Commands.parallel(
                elevator.followPosition(elevatorPosition),
                shoulder.followPosition(armPosition));
    }

    public Command goToPositions(double elevatorPosition, double armPosition) {
        return runEnd(
                () -> {
                        elevator.goToPosition(elevatorPosition);
                        shoulder.goToPosition(armPosition);
                }, this::stop
        );
    }

    public Command resetPositions() {
        return runOnce(
                () -> {
                    elevator.resetPosition();
                    shoulder.resetPosition();
                }
        );
    }
}