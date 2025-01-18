package frc.robot.subsystems.drivetrain;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.DoubleSupplier;

@Logged
public class Drivetrain extends SubsystemBase {
    @NotLogged
    private final DrivetrainIO io;
    private final DrivetrainInputs inputs = new DrivetrainInputs();

    public Drivetrain(DrivetrainIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
    }

    public Command drive(DoubleSupplier xVelocity, DoubleSupplier yVelocity, DoubleSupplier angularVelocity) {
        return runEnd(() -> {
            io.driveFieldOriented(xVelocity.getAsDouble(), yVelocity.getAsDouble(), angularVelocity.getAsDouble());
        }, io::stop);
    }
}
