package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import edu.wpi.first.math.geometry.Translation2d;

public class SwerveConstants {
    public static final double TICKS_PER_RADIAN = 2048/(Math.PI*2);

    public static final double VOLT_COMP_SATURATION = 12;
    public static final SupplyCurrentLimitConfiguration SUPPLY_CURRENT_LIMIT = new SupplyCurrentLimitConfiguration(true, 50, 0, 0);
    public static final StatorCurrentLimitConfiguration STATOR_CURRENT_LIMIT = new StatorCurrentLimitConfiguration(true, 50, 0, 0);

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

    public static final double DRIVE_kP = 0;
    public static final double DRIVE_kI = 0;
    public static final double DRIVE_kD = 0;

    public static final double MAX_X_VELOCITY = 0;
    public static final double MAX_Y_VELOCITY = 0;
    public static final double MAX_OMEGA_VELOCITY = 0;

    public static final TalonFXInvertType CLOCKWISE = TalonFXInvertType.Clockwise;
    public static final TalonFXInvertType COUNTER_CLOCKWISE = TalonFXInvertType.CounterClockwise;

    public static final double NEUTRAL_DEADBAND = 0.15;
    public static final double XBOX_DEADBAND = 0.15;

    public static final double robotWidth = 0;
    public static final double robotLength = 0;
    public static final Translation2d[] wheelPositions = {
            new Translation2d(-(robotWidth/2), robotLength/2),   //FL
            new Translation2d(robotWidth/2, robotLength/2),   //FR
            new Translation2d(-(robotWidth/2), -(robotLength/2)),  //RL
            new Translation2d(robotWidth/2, -(robotLength/2))}; //RR
}
