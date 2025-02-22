package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;

@Logged
public class Elevator extends SubsystemBase {
    private static final double WAIT_TIME = 0.25; // In seconds
    public static final double POSITION_THRESHOLD = Units.inchesToMeters(1.0);
    private static final double VELOCITY_THRESHOLD = Units.rotationsPerMinuteToRadiansPerSecond(1.0);

    private final ElevatorIO io;
    private final ElevatorInputs inputs = new ElevatorInputs();
    private final Constants constants;
    private final SysIdRoutine sysIdRoutine;

    private boolean zeroed = false;
    private double targetPosition = 0.0;

    public Elevator(ElevatorIO io) {
        this.io = io;
        this.constants = io.getConstants();

        // Creating a SysId Routine
        sysIdRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(
                        Volts.of(0.25).per(Second),
                        Volts.of(4),
                        null,
                        (state) -> SignalLogger.writeString("state", state.toString())),
                new SysIdRoutine.Mechanism(
                        io::runSysId,
                        null,
                        this));
    }

    public Constants getConstants() {
        return constants;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
    }

    public double getVelocity() {
        return inputs.currentVelocityMotor1;
    }

    public double getPosition() {
        return inputs.currentPositionMotor1;
    }

    public boolean atTargetPosition() {
        return atPosition(targetPosition);
    }

    public boolean atPosition(double position) {
        return Math.abs(inputs.currentPositionMotor1 - position) < POSITION_THRESHOLD;
    }

    public boolean atPosition(double position, double threshold) {
        return Math.abs(inputs.currentPositionMotor1 - position) < threshold;
    }

    public double getTargetPosition() {
        return targetPosition;
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }

    public Command stop() {
        return runOnce(() -> {
            targetPosition = 0.0;
            io.stop();
        });
    }

    public Command resetPosition() {
        return Commands.runOnce(() -> {
            io.resetPosition();
            zeroed = true;
        });
    }

    public Command goToPosition(double position) {
        return run(() -> {
            if (zeroed) {
                targetPosition = getValidPosition(position);
                io.setTargetPosition(targetPosition);
            }
        }).until(this::atTargetPosition);
    }

    public Command followPosition(DoubleSupplier position) {
        return run(() -> {
            if (zeroed) {
                targetPosition = getValidPosition(position.getAsDouble());
                io.setTargetPosition(targetPosition);
            }
        });
    }

    private double getValidPosition(double position) {
        return Math.max(constants.minimumPosition, Math.min(constants.maximumPosition, position));
    }

    public Command zero() {
        // Run the Elevator backwards until stopped and then stop
        return runEnd(() -> {
            io.setTargetVoltage(-1.0);
            targetPosition = constants.minimumPosition;
        }, io::stop)
                // While doing that wait until the elevator stops (Hit the hard stop)
                // Also stop the previous command when this one stops (It hit the hard stop and reset position)
                .withDeadline(waitUntil(() -> Math.abs(inputs.currentVelocityMotor1) < VELOCITY_THRESHOLD)
                        // Then reset the elevator position and set zeroed to true
                        .andThen(() -> {
                            io.resetPosition();
                            zeroed = true;
                        })
                        // Before we check if we're at the bottom hard stop, wait a little so that it doesn't think we hit it because we haven't started going yet
                        .beforeStarting(waitSeconds(WAIT_TIME)));
    }

    public record Constants(
            double minimumPosition,
            double maximumPosition
    ) {}
}