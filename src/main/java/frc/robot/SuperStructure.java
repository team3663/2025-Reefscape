package frc.robot;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.elevator.Elevator;

import java.util.function.DoubleSupplier;

import static edu.wpi.first.wpilibj2.command.Commands.runEnd;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

@Logged
public class SuperStructure {
    private final Elevator elevator;
    private final Arm arm;

    private final CommandXboxController controller = new CommandXboxController(0);

    public SuperStructure(Elevator elevator, Arm arm) {
        this.elevator = elevator;
        this.arm = arm;
    }

    private boolean armCollide(double elevatorCurrentPos, double armCurrentPos, double elevatorTargetPos, double armTargetPos) {
        return false;
    }

    private boolean armCollide(double elevatorTargetPos, double armTargetPos) {
        return armCollide(elevator.getPosition(), arm.getPosition(), elevatorTargetPos, armTargetPos);
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
        return runEnd(
                () -> {
                    if (armCollide(elevatorPosition.getAsDouble(), armPosition.getAsDouble())) {
                        elevator.goToPosition(elevatorPosition.getAsDouble());
                        arm.goToPosition(armPosition.getAsDouble());
                    }
                }, this::stop
        );
    }

    public Command goToPositions(double elevatorPosition, double armPosition) {
        return runEnd(
                () -> {
                    if (armCollide(elevatorPosition, armPosition)) {
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