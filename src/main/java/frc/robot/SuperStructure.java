package frc.robot;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.elevator.Elevator;

@Logged
public class SuperStructure {
    private final Elevator elevator;
    private final Arm arm;

    private final CommandXboxController controller = new CommandXboxController(0);

    public SuperStructure(Elevator elevator, Arm arm) {
        this.elevator = elevator;
        this.arm = arm;
    }
}
