package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.elevator.Elevator;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class SuperStructure extends SubsystemBase {
    private final Elevator elevator;
    private final Arm arm;

    public SuperStructure(Elevator elevator, Arm arm) {
        this.elevator = elevator;
        this.arm = arm;
    }

    private boolean armNotCollide(double elevatorCurrentPos, double armCurrentPos, double elevatorTargetPos, double armTargetPos) {
        return true;
    }

    private boolean armNotCollide(double elevatorTargetPos, double armTargetPos) {
        return armNotCollide(elevator.getPosition(), arm.getPosition(), elevatorTargetPos, armTargetPos);
    }

    public Command stop() {
        return runOnce(
                () -> {
                    elevator.stop();
                    arm.stop();
                }
        );
    }

    public Command followPositions(DoubleSupplier elevatorPosition, DoubleSupplier armPosition) {
        BooleanSupplier go = () -> armNotCollide(elevatorPosition.getAsDouble(), armPosition.getAsDouble());
        return Commands.parallel(
                elevator.followPosition(elevatorPosition, go),
                arm.followPosition(armPosition, go));


//        return runOnce(
//                () -> {
//                    BooleanSupplier go = () -> armNotCollide(elevatorPosition.getAsDouble(), armPosition.getAsDouble());
//                    elevator.followPosition(elevatorPosition, go);
//                    arm.followPosition(armPosition, go);
//                }
//        );

    }

    public Command goToPositions(double elevatorPosition, double armPosition) {
        return runEnd(
                () -> {
                    if (armNotCollide(elevatorPosition, armPosition)) {
                        elevator.goToPosition(elevatorPosition);
                        arm.goToPosition(armPosition);
                    }
                }, this::stop
        );
    }

    public Command resetPositions() {
        return runOnce(
                () -> {
                    elevator.resetPosition();
                    arm.resetPosition();
                }
        );
    }
}