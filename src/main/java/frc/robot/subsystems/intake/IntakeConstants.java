package frc.robot.subsystems.intake;

public class IntakeConstants {
    public static final double ANGLE_MOTOR_GEARING = 35.26;
    public static final double SPIN_MOTOR_GEARING = 0;
    public static final double ANGLE_MOTOR_MOMENT_OF_INERTIA = 0;
    public static final double SPIN_MOTOR_MOMENT_OF_INERTIA = 0;
    public static final double VOLT_COMPENSATION = 12;
    public static final double FALCON_TICKS_PER_ROTATION = 2048 * ANGLE_MOTOR_GEARING;
    public static final double ANGLE_MOTOR_TICKS_PER_RADIAN = FALCON_TICKS_PER_ROTATION/ (Math.PI*2);
}
