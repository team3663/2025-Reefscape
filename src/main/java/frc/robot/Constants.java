package frc.robot;

import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.math.util.Units;

public class Constants {

    public static final double MK4_2PLUS_REDUCTION = (50.0 / 16.0) * (17.0 / 27.0) * (45.0 / 15.0);
    public static final double MK4_3PLUS_REDUCTION = (50.0 / 16.0) * (16.0 / 28.0) * (45.0 / 15.0);

    public static final double MK4N_STEER_REDUCTION = 18.75;
    public static final double MK4N_STEER_INERTIA = 0.00001;
    public static final double MK4N_STEER_FRICTION_VOLTAGE = 0.25;
    public static final Slot0Configs MK4N_STEER_PID_CONSTANTS = new Slot0Configs()
            .withKP(100.0);
    public static final double MK4N_WHEEL_RADIUS = Units.inchesToMeters(4.0);

    public static final double MK4I_STEER_REDUCTION = (150.0 / 7.0);
    public static final double MK4I_STEER_INERTIA = 0.00001;
    public static final double MK4I_STEER_FRICTION_VOLTAGE = 0.25;
    public static final double MK4_WHEEL_RADIUS = Units.inchesToMeters(1.93);
    public static final Slot0Configs MK4I_STEER_PID_CONSTANTS = new Slot0Configs()
            .withKP(50);

}
