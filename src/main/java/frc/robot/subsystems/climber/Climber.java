package frc.robot.subsystems.climber;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.DoubleSupplier;

@Logged
public class Climber extends SubsystemBase {
    private final ClimberIO io;
    private final ClimberInputs inputs = new ClimberInputs();

    private static final double POSITION_THRESHOLD = Units.degreesToRadians(1.0);
    private double targetPosition = 0.0;
    private double targetVoltage = 0.0;


    public Climber(ClimberIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
    }

    public boolean atTargetPosition() {
        return Math.abs(inputs.currentPosition - targetPosition) < POSITION_THRESHOLD;
    }

    public Command stop() {
        return runOnce(() -> {
                    targetPosition = 0.0;
                    targetVoltage = 0.0;
                    io.stop();
                }
        );
    }

    public Command resetPosition() {
        return runOnce(
                io::resetPosition
        );
    }


    public Command goToPosition(double position) {
        return runEnd(
                () -> {
                    io.setTargetPosition(position);
                    targetPosition = position;
                }, io::stop
        ).until(this::atTargetPosition);
    }

    public Command followPosition(DoubleSupplier position) {
        return runEnd(
                () -> {
                    targetPosition = position.getAsDouble();
                    io.setTargetPosition(targetPosition);
                },
                io::stop
        );
    }
}
