package frc.robot.subsystems.shooter;

import com.pathplanner.lib.auto.PIDConstants;

public class ShooterConstants {
    public static final PIDConstants PID_CONSTANTS = new PIDConstants(1, 0, 0);
    public static final double Kf = 0;
    public static final double SHOOTER_VELOCITY_DEADBAND = 50;
    public static final int TICKS_PER_ROTATION = 2048;
    public static final double MAX_WARMUP_VELOCITY = 4600;
    public static final double MOMENT_OF_INERTIA = 0.0006;
}
