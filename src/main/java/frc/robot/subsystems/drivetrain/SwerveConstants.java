package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import edu.wpi.first.math.geometry.Translation2d;

public class SwerveConstants {
    public static final double[] OFFSETS =
            {0.5686593642164841, 0.26232458155811456, 0.00854002521350063, 0.5429330635733266};

    public static final double VOLT_COMP_SATURATION = 12;
    public static final int CURRENT_LIMIT = 50;
    public static final int ALT_CURRENT_LIMIT = 20;

    public static final double robotWidth = 0.512; //[m] //TODO: update
    public static final double robotLength = 0.67; //[m]
    public static final double WHEEL_DIAMETER = 0.1023679821; //[m]
    public static final double DRIVE_REDUCTION = (1 / 2.0) * (24.0 / 22.0) * (15.0 / 45.0);
    public static final double ANGLE_REDUCTION = (14.0 / 72.0) * 0.5;
    public static final double DriveMotorMomentOfInertia = 0.025;
    public static final double AngleMotorMomentOfInertia = 0.004;

    // kP, kI, kD, kF, sCurveStrength, cruiseVelocity, acceleration, allowableError,
    // maxIntegralAccum, peakOutput
    public static final double[] FRONT_LEFT_MOTION_MAGIC_CONFIGS = {1, 0, 0, 0.2, 1, 21288, 25000, 10, 5, 1};
    public static final double[] FRONT_RIGHT_MOTION_MAGIC_CONFIGS = {1, 0, 0, 0.2, 1, 21288, 25000, 10, 5, 1};
    public static final double[] REAR_LEFT_MOTION_MAGIC_CONFIGS = {1, 0, 0, 0.2, 1, 21288, 25000, 10, 5, 1};
    public static final double[] REAR_RIGHT_MOTION_MAGIC_CONFIGS = {1, 0, 0, 0.2, 1, 21288, 25000, 10, 5, 1};

    public static final double[][] motionMagicConfigs = {
            FRONT_LEFT_MOTION_MAGIC_CONFIGS,
            FRONT_RIGHT_MOTION_MAGIC_CONFIGS,
            REAR_LEFT_MOTION_MAGIC_CONFIGS,
            REAR_RIGHT_MOTION_MAGIC_CONFIGS};

    public static final double DRIVE_kP = 0.005;
    public static final double DRIVE_kI = 0.0;
    public static final double DRIVE_kD = 0.148;
    public static final double DRIVE_KF = 0.05;

    public static final double OMEGA_kP = 0.5;
    public static final double OMEGA_kI = 0.0;
    public static final double OMEGA_kD = 0.0;

    public static final double MAX_X_Y_VELOCITY = 6380.0 / 60.0 * //[m/s]
            DRIVE_REDUCTION *
            WHEEL_DIAMETER * Math.PI;

    public static final double MAX_OMEGA_VELOCITY = MAX_X_Y_VELOCITY / //[m/s]
            Math.sqrt((robotLength / 2) * (robotLength / 2) + (robotWidth / 2) * (robotWidth / 2));

    public static final double NEUTRAL_DEADBAND = 0.15;
    public static final double XBOX_DEADBAND = 0.15;


    public static final double TICKS_PER_RADIAN = 42 / ANGLE_REDUCTION / (Math.PI * 2);
    public static final double TICKS_PER_METER = (42 / DRIVE_REDUCTION) / (Math.PI * WHEEL_DIAMETER);

    public static final Translation2d[] wheelPositions = {
            new Translation2d(robotLength / 2, robotWidth / 2),   //FL
            new Translation2d(robotLength / 2, -robotWidth / 2),   //FR
            new Translation2d(-robotLength / 2, robotWidth / 2),  //RL
            new Translation2d(-robotLength / 2, -robotWidth / 2)}; //RR

    public static final double MAX_VELOCITY_AUTO = 4.0;
    public static final double MAX_ACCELERATION_AUTO = 2.5;

    public static double AUTO_X_Kp = 3.20;
    public static double AUTO_X_Ki = 0.0;
    public static double AUTO_X_Kd = 0.73;
    public static double AUTO_Y_Kp = 3.20;
    public static double AUTO_Y_Ki = 0.0;
    public static double AUTO_Y_Kd = 0.6;
    public static double AUTO_ROTATION_Kp = 10;
    public static double AUTO_ROTATION_Ki = 0.0;
    public static double AUTO_ROTATION_Kd = 0.0;

    public static double FORWARD_BALANCE_TIME = 0.65;
    public static double BACKWARD_BALANCE_TIME = 0.55;
}
