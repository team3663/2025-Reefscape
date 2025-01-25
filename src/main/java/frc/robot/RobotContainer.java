// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.climber.C2025ClimberIO;
import frc.robot.subsystems.climber.Climber;
import frc.robot.utility.ControllerHelper;

@Logged
public class RobotContainer {
  private final Climber climber = new Climber(new C2025ClimberIO(
          new TalonFX(0), new CANdi(0), new CANcoder(0)));

  private final CommandXboxController controller = new CommandXboxController(0);
  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    controller.a().onTrue(climber.goToPosition(Units.rotationsToRadians(2.0)));
    controller.b().onTrue(climber.followPosition(() -> ControllerHelper.modifyAxis(controller.getLeftY(), Units.rotationsToRadians(3.0))));
    controller.x().onTrue(climber.goToPosition(Units.rotationsToRadians(1.0)));
    controller.y().onTrue(climber.stop());
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
