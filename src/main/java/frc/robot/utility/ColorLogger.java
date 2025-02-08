package frc.robot.utility;

import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.wpilibj.util.Color;

@CustomLoggerFor(Color.class)
public class ColorLogger extends ClassSpecificLogger<Color> {
    public ColorLogger() {
        super(Color.class);
    }

    @Override
    protected void update(EpilogueBackend epilogueBackend, Color color) {
        epilogueBackend.log("red", color.red);
        epilogueBackend.log("green", color.green);
        epilogueBackend.log("blue", color.blue);
    }
}
