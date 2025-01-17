// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.prototype.P2025PrototypeIO;
import frc.robot.subsystems.prototype.Prototype;

@Logged
public class RobotContainer {
    private final Prototype prototype = new Prototype(new P2025PrototypeIO(new TalonFX(0, "3663"), new TalonFX(1, "3663")));
    private final CommandXboxController driverController = new CommandXboxController(0);

    public RobotContainer() {

        configureBindings();
    }

    private double modifyAxis(double value) {
        double clippedValue = MathUtil.applyDeadband(value, 0.08);
        return -Math.copySign(clippedValue * clippedValue, value);

    }

    private void configureBindings() {

        prototype.setDefaultCommand(prototype.followVoltage(
                () -> 12 * (modifyAxis(driverController.getLeftY())),
                () -> 12 * (modifyAxis(driverController.getRightY()))));

        driverController.a().onTrue(prototype.stopMotors());

        driverController.b().onTrue(prototype.followVoltage(() -> 6, () -> 6));

        driverController.x().onTrue(prototype.followVoltage(() -> 4, () -> 4));

        driverController.y().onTrue(prototype.followVoltage(() -> 2, () -> 2));

    }


    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
