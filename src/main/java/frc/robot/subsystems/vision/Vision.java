package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {

    private final VisionIO[] ios;
    private final VisionInputs[] visionInputs;

    private Vision(VisionIO... ios) {
       this.ios = ios;

        visionInputs = new VisionInputs[ios.length];
        for (int i = 0; i < visionInputs.length; i++) {
            visionInputs[i] = new VisionInputs();
        }
    }

    @Override
    public void periodic() {

        for (int i = 0; i < ios.length; i++) {
            ios[i].updateInputs(visionInputs[i]);

        }
    }
}

