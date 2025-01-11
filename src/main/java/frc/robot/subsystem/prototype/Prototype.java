package frc.robot.subsystem.prototype;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.DoubleSupplier;
@Logged
public class Prototype extends SubsystemBase {
    private static final double VELOCITY_THRESHOLD = Units.rotationsPerMinuteToRadiansPerSecond(100.0);
    private static final double POSITION_THRESHOLD = Units.degreesToRadians(5);
    private final PrototypeIO io;
    private final PrototypeInputs inputs = new PrototypeInputs();

    private double targetVelocity1 = 0.0;
    private double targetVelocity2 = 0.0;
    private double targetPosition1 = 0.0;
    private double targetPosition2= 0.0;
    private double targetVoltage1 = 0.0;
    private double targetVoltage2 = 0.0;
    public Prototype(PrototypeIO io) {
        this.io = io;
    }
    @Override
    public void periodic(){
        io.updateInputs(inputs);
    }
    //Updating Inputs
    public double getVelocity1(){return inputs.currentVelocity1;}
    public double getVoltage1(){return inputs.currentAppliedVoltage1;}
    public double getVoltage2(){return inputs.currentAppliedVoltage2;}
    public boolean atTargetPosition1(){return Math.abs(inputs.currentPosition1 - targetPosition1) < POSITION_THRESHOLD;}
    public boolean atTargetVelocity1(){return Math.abs(inputs.currentVelocity1 - targetVelocity1) < VELOCITY_THRESHOLD;}
    public double getVelocity2(){return inputs.currentVelocity2;}
    public boolean atTargetPosition2(){return Math.abs(inputs.currentPosition2 - targetPosition2) < POSITION_THRESHOLD;}
    public boolean atTargetVelocity2(){return Math.abs(inputs.currentVelocity2 - targetVelocity2) < VELOCITY_THRESHOLD;}
    public Command stop() {
        return runOnce(
                io::stop
        );
    }
    public Command stop1(){
        return runOnce(
                io::stop1
        );
    }
    public Command stop2(){
        return runOnce(
                io::stop2
        );
    }
    public Command withVelocity1(double velocity){
        return runEnd(
                () -> {
                    io.setTargetVelocity1(velocity);
                    targetVelocity1 = velocity;
                },
                io::stop1
        );
    }
    public Command withVelocity2(double velocity){
        return runEnd(
                () -> {
                    io.setTargetVelocity2(velocity);
                    targetVelocity2 = velocity;
                },
                io::stop2
        );
    }
    public Command followVelocity1(DoubleSupplier velocity) {
        return runEnd(
                () -> {
                    io.setTargetVelocity1(velocity.getAsDouble());
                    targetVelocity1 = velocity.getAsDouble();
                },
                io::stop1
        );
    }
    public Command followVelocity2(DoubleSupplier velocity) {
        return runEnd(
                () -> {
                    io.setTargetVelocity2(velocity.getAsDouble());
                    targetVelocity2 = velocity.getAsDouble();
                },
                io::stop2
        );
    }
    public Command followVelocity(DoubleSupplier velocity1, DoubleSupplier velocity2) {
        return runEnd(
                () -> {
                    io.setTargetVelocity1(velocity1.getAsDouble());
                    io.setTargetVelocity2(velocity2.getAsDouble());
                    targetVelocity1 = velocity1.getAsDouble();
                    targetVelocity2 = velocity2.getAsDouble();
                },
                io::stop
        );
    }
    public Command resetPosition1() {
        return runOnce(
                () -> io.resetPosition1()
        );
    }
    public Command resetPosition2() {
        return runOnce(
                () -> io.resetPosition2()
        );
    }

    public Command goToPosition1(double position1) {
        return run(
                () -> {
                    io.setTargetPosition1(position1);
                    targetPosition1 = position1;
                }
        ).until(this::atTargetPosition1);
    }
    public Command goToPosition2(double position2) {
        return run(
                () -> {
                    io.setTargetPosition2(position2);
                    targetPosition2 = position2;
                }
        ).until(this::atTargetPosition2);
    }

    public Command followPosition1(DoubleSupplier position1) {
        return runEnd(
                () -> {
                    io.setTargetPosition1(position1.getAsDouble());
                    targetPosition1 = position1.getAsDouble();
                },
                io::stop1
        );
    }
    public Command followPosition2(DoubleSupplier position2) {
        return runEnd(
                () -> {
                    io.setTargetPosition2(position2.getAsDouble());
                    targetPosition2 = position2.getAsDouble();
                },
                io::stop2
        );
    }
    public Command followVoltage1(DoubleSupplier voltage1) {
        return runEnd(
                () -> {
                    io.setTargetVoltage1(voltage1.getAsDouble());
                    targetVoltage1 = voltage1.getAsDouble();
                },
                io::stop1
        );
    }
    public Command followVoltage2(DoubleSupplier voltage2) {
        return runEnd(
                () -> {
                    io.setTargetVoltage2(voltage2.getAsDouble());
                    targetVoltage2 = voltage2.getAsDouble();
                },
                io::stop2
        );
    }
    public Command followVoltage(DoubleSupplier voltage1, DoubleSupplier voltage2) {
        return runEnd(
                () -> {
                    io.setTargetVoltage1(voltage1.getAsDouble());
                    targetVoltage1 = voltage1.getAsDouble();
                    io.setTargetVoltage2(voltage2.getAsDouble());
                    targetVoltage2 = voltage2.getAsDouble();

                },
                io::stop
        );
    }
    public Command getVoltage1(DoubleSupplier voltage1) {
        return runEnd(
                () -> {
                    io.setTargetVoltage1(voltage1.getAsDouble());
                    targetVoltage1 = voltage1.getAsDouble();
                },
                io::stop1
        );
    }
    public Command getVoltage2(DoubleSupplier voltage2) {
        return runEnd(
                () -> {
                    io.setTargetVoltage2(voltage2.getAsDouble());
                    targetVoltage2 = voltage2.getAsDouble();
                },
                io::stop2
        );
    }

}
