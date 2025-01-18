package frc.robot.subsystem.climber;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.DoubleSupplier;

public class Climber extends SubsystemBase {
    private final ClimberInputs inputs;
    private final ClimberIO io;
    private static final double POSITION_THRESHOLD =Units.degreesToRadians(1.0);
    private double targetPosition = 0.0;
    private double targetVoltage = 0.0;

    public Climber(ClimberInputs inputs, ClimberIO io) {
        this.inputs = inputs;
        this.io = io;
    }
    @Override
    public void periodic(){
        io.updateInputs(inputs);
    }
    public boolean atTargetPosition() {
        return Math.abs(inputs.currentPosition - targetPosition) < POSITION_THRESHOLD;
    }
    public Command stop(){
        return runOnce(
                io::stop
        );
    }
    public Command resetPosition(){
        return runOnce(
                () -> io.resetPosition()
        );
    }
    public Command goToPosition(double position){
        return run(
                () -> {
                    io.setTargetPosition(position);
                    targetPosition = position;
                }
        ).until(this::atTargetPosition);
    }
    public Command followPosition(DoubleSupplier position){
        return runEnd(
                () -> {
                    io.setTargetPosition(position.getAsDouble());
                    targetPosition = position.getAsDouble();
                },
                io::stop
        );
    }
    public Command withVoltageUntilBeam(double voltage){
        return runEnd(() -> {
                targetVoltage = voltage;
                io.setTargetVoltage(targetVoltage);
                }, io::stop
        ).until(() -> inputs.beamBrakeState);
    }

}
