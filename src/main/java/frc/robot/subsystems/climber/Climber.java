package frc.robot.subsystems.climber;

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

@Logged
public class Climber extends SubsystemBase {
    private static final double VELOCITY_THRESHOLD = Units.rotationsPerMinuteToRadiansPerSecond(1);
    private static final double POSITION_THRESHOLD = Units.degreesToRadians(5.0);
    private static final double WAIT_TIME = 0.25;

    private static final double DEPLOY_ANGLE = Units.degreesToRadians(7.0);
    private static final double CLIMB_ANGLE = Units.degreesToRadians(158.5);
    private static final double STOW_THRESHOLD = Units.degreesToRadians(20);

    private final ClimberIO io;
    private final Constants constants;
    private final ClimberInputs inputs = new ClimberInputs();
    private final SysIdRoutine sysIdRoutine;

    private boolean zeroed = true;
    private double targetPosition = 0.0;
    private double targetVoltage = 0.0;

    public Climber(ClimberIO io) {
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

    @Override
    public void periodic() {
        io.updateInputs(inputs);
    }

    public boolean atTargetPosition() {
        return Math.abs(inputs.currentPosition - targetPosition) < POSITION_THRESHOLD;
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
                    targetVoltage = 0.0;
                    io.stop();
                }
        );
    }

    public Command goToPosition(double position) {
        return runEnd(
                () -> {
                    if (zeroed) {
                        targetPosition = getValidPosition(position);
                        io.setTargetPosition(targetPosition);
                    }
                }, io::stop
        ).until(this::atTargetPosition);
    }

    public Command followPosition(DoubleSupplier position) {
        return runEnd(
                () -> {
                    if (zeroed) {
                        targetPosition = getValidPosition(position.getAsDouble());
                        io.setTargetPosition(targetPosition);
                    }
                },
                io::stop
        );
    }

    private double getValidPosition(double position) {
        return Math.max(constants.minimumPosition, Math.min(constants.maximumPosition, position));
    }

    public Command stow(){
        return goToPosition(constants.minimumPosition)
                .withDeadline(Commands.waitUntil(()-> inputs.currentPosition <STOW_THRESHOLD).andThen(waitSeconds(0.5)));
    }

    public Command zero() {
        // Run the Elevator backwards until stopped and then stop
//        return runEnd(() -> {
//            io.setTargetVoltage(-1.0);
//            targetPosition = constants.minimumPosition;
//        }, io::stop)
//                // While doing that wait until the elevator stops (Hit the hard stop)
//                // Also stop the previous command when this one stops (It hit the hard stop and reset position)
//                .withDeadline(waitUntil(() -> Math.abs(inputs.currentVelocity) < VELOCITY_THRESHOLD)
//                        // Then reset the elevator position and set wristZeroed to true
//                        .andThen(() -> {
//                            io.resetPosition();
//                            zeroed = true;
//                        })
//                        // Before we check if we're at the bottom hard stop, wait a little so that it doesn't think we hit it because we haven't started going yet
//                        .beforeStarting(waitSeconds(WAIT_TIME)));
        return Commands.none();
    }

//    public Command defaultCommand()
//    {
//        return followPosition(constants::minimumPosition);
//    }

    public Command arm() {
        return followPosition(() -> DEPLOY_ANGLE);
    }

    public Command climb() {
        return followPosition(() -> CLIMB_ANGLE);
    }

    public record Constants(
            double maximumPosition,
            double minimumPosition
    ) {}
}
