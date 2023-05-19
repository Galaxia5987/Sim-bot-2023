package frc.robot.subsystems.intake;

public class IntakeConstants {
    public static final double GEARING = 0;
    public static final double ANGLE_MOTOR_MOMENT_OF_INERTIA = 0;
    public static final double SPIN_MOTOR_MOMENT_OF_INERTIA = 0;
    public static final double VOLT_COMPENSATION = 12;
    public static final double FALCON_TICKS_PER_ROTATION = 2048;
    public static final double TICKS_PER_DEGREE = FALCON_TICKS_PER_ROTATION/ 360*GEARING;
}
