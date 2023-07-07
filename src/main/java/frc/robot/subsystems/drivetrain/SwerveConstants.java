package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import edu.wpi.first.math.geometry.Translation2d;

public class SwerveConstants {
    public static final double[] OFFSETS =
            {0.013, 0.2134, 0.594, 0.082};

    public static final double VOLT_COMP_SATURATION = 12;
    public static final SupplyCurrentLimitConfiguration SUPPLY_CURRENT_LIMIT = new SupplyCurrentLimitConfiguration(true, 50, 0, 0);
    public static final StatorCurrentLimitConfiguration STATOR_CURRENT_LIMIT = new StatorCurrentLimitConfiguration(true, 50, 0, 0);

    public static final double robotWidth = 0.587; //[m]
    public static final double robotLength = 0.62; //[m]
    public static final double WHEEL_DIAMETER = 0.1023679821; //[m]
    public static final double DRIVE_REDUCTION = (1 / 2.0) * (22.0 / 24.0) * (15.0 / 45.0);
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

    public static final double OMEGA_kP = 1.8;
    public static final double OMEGA_kI = 0.0;
    public static final double OMEGA_kD = 0.0;

    public static final double MAX_X_Y_VELOCITY = 6380.0 / 60.0 * //[m/s]
            DRIVE_REDUCTION *
            WHEEL_DIAMETER * Math.PI;

    public static final double MAX_OMEGA_VELOCITY = MAX_X_Y_VELOCITY/ //[m/s]
            Math.sqrt((robotLength/2)*(robotLength/2)+(robotWidth/2)*(robotWidth/2));

    public static final TalonFXInvertType CLOCKWISE = TalonFXInvertType.Clockwise;
    public static final TalonFXInvertType COUNTER_CLOCKWISE = TalonFXInvertType.CounterClockwise;

    public static final double NEUTRAL_DEADBAND = 0.15;
    public static final double XBOX_DEADBAND = 0.15;


    public static final double TICKS_PER_RADIAN = 2048/ANGLE_REDUCTION/(Math.PI*2);
    public static final double TICKS_PER_METER = (2048/DRIVE_REDUCTION)/(Math.PI*WHEEL_DIAMETER);

    public static final Translation2d[] wheelPositions = {
            new Translation2d(robotLength/2, robotWidth/2),   //FL
            new Translation2d(robotLength/2, -robotWidth/2),   //FR
            new Translation2d(-robotLength/2, robotWidth/2),  //RL
            new Translation2d(-robotLength/2, -robotWidth/2)}; //RR
}
