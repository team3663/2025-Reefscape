// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystem.prototype.P2025PrototypeIO;
import frc.robot.subsystem.prototype.Prototype;
import edu.wpi.first.math.MathUtil;
import frc.robot.subsystem.prototype.PrototypeIO;
@Logged
public class RobotContainer {
  private final TalonFX motor1 = new TalonFX(0);
  private final TalonFX motor2 = new TalonFX(1);
  private final CommandXboxController controller = new CommandXboxController(0);
  private final P2025PrototypeIO io = new P2025PrototypeIO(motor1, motor2);

  private final Prototype prototype = new Prototype(io);

  private double modifyAxis(double value) {
    double clippedValue = MathUtil.applyDeadband(value, 0.08);
    return -Math.copySign((clippedValue * clippedValue) * 12, value);
  }
  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    controller.a().whileTrue(prototype.stop());
    prototype.setDefaultCommand(prototype.followVoltage(
            () -> modifyAxis(controller.getRawAxis(1)),
            () -> modifyAxis(controller.getRawAxis(5))));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
