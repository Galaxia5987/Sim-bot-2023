package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;

public class SwerveConstants {
        public static final double TICKS_PER_ROTATION = 2048;
        public static final int[] OFFSETS = { 0, 0, 0, 0 };

        public static final double DRIVETRAIN_TRACK_WIDTH_METERS = 0.51594;
        public static final double DRIVETRAIN_WHEELBASE_METERS = 0.66594;

        public static final double DRIVE_REDUCTION = (1 / 2.0) * (22.0 / 24.0) * (15.0 / 45.0);
        public static final double ANGLE_REDUCTION = (14.0 / 72.0) * 0.5;
        public static final double DRIVE_MOMENT_OF_INERTIA = 0.025;
        public static final double ANGLE_MOMENT_OF_INERTIA = 0.004;
        public static final double WHEEL_DIAMETER = 0.1023679821;

        public static final StatorCurrentLimitConfiguration STATOR_CURRENT_LIMIT_CONFIG = new StatorCurrentLimitConfiguration(
                        true, 50, 2, 0.02);
        public static final SupplyCurrentLimitConfiguration SUPPLY_CURRENT_LIMIT_CONFIG = new SupplyCurrentLimitConfiguration(
                        true, 50, 2, 0.02);
        public static final double NEUTRAL_DEADBAND = 0.1;

        // kP, kI, kD, kF, sCurveStrength, cruiseVelocity, acceleration, allowableError,
        // maxIntegralAccum, peakOutput
        public static final double[] FRONT_LEFT_MOTION_MAGIC_CONFIGS = { 1, 0, 0, 0.2, 1, 21288, 25000, 10, 5, 1 };
        public static final double[] FRONT_RIGHT_MOTION_MAGIC_CONFIGS = { 1, 0, 0, 0.2, 1, 21288, 25000, 10, 5, 1 };
        public static final double[] REAR_LEFT_MOTION_MAGIC_CONFIGS = { 1, 0, 0, 0.2, 1, 21288, 25000, 10, 5, 1 };
        public static final double[] REAR_RIGHT_MOTION_MAGIC_CONFIGS = { 1, 0, 0, 0.2, 1, 21288, 25000, 10, 5, 1 };

        public static final double[][] MOTION_MAGIC_CONFIGS = { FRONT_LEFT_MOTION_MAGIC_CONFIGS,
                        FRONT_RIGHT_MOTION_MAGIC_CONFIGS, REAR_LEFT_MOTION_MAGIC_CONFIGS,
                        REAR_RIGHT_MOTION_MAGIC_CONFIGS };

        public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 *
                        DRIVE_REDUCTION *
                        WHEEL_DIAMETER * Math.PI;
        public static final double MAX_ROTATIONAL_VELOCITY = MAX_VELOCITY_METERS_PER_SECOND /
                        Math.sqrt(Math.pow(DRIVETRAIN_TRACK_WIDTH_METERS / 2, 2)
                                + Math.pow(DRIVETRAIN_WHEELBASE_METERS / 2, 2));

        public static final double MAX_VELOCITY_AUTO = 4;
        public static final double MAX_ACCELERATION_AUTO = 2.5;

        public static final double VELOCITY_FEEDFORWARD_REAL = 0.33031;
        public static final double ACCELERATION_FEEDFORWARD_REAL = 0.080553;
        public static final double STATIC_FEEDFORWARD_REAL = 0.21189;

        public static final double VELOCITY_FEEDFORWARD_SIM = 0.33031;
        public static final double ACCELERATION_FEEDFORWARD_SIM = 0.080553;
        public static final double STATIC_FEEDFORWARD_SIM = 0.21189 / 2.0;

        public static final double DRIVE_Kp = 3.5;
        public static final double DRIVE_Ki = 0.0;
        public static final double DRIVE_Kd = 0.0;
}
