package frc.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

public class ArmConstants {

    public static final int WAIT_TIME = 0;

    //shoulder physics
    public static final double SHOULDER_GEARING = 106.7; // Arbitrary units
    public static final double SHOULDER_MASS = 4; // [kg]
    public static final double SHOULDER_LENGTH = 0.75679; // [m]
    public static final double SHOULDER_MOMENT_OF_INERTIA = 2.613; // [kg*m^2]
    public static final double SHOULDER_CENTER_OF_MASS_RADIUS = 0.32; // [m]
    public static final int SHOULDER_NUMBER_OF_MOTORS = 2; // Arbitrary units
    public static final double SHOULDER_ARM_LENGTH = 0.75679; //[m]

    //elbow physics
    public static final double ELBOW_GEARING = 51.3; // Arbitrary units
    public static final double ELBOW_MASS = 3.5; // [kg]
    public static final double ELBOW_LENGTH = 0.75889; // [m]
    public static final double ELBOW_MOMENT_OF_INERTIA = 2.65; // [kg*m^2]
    public static final double ELBOW_CENTER_OF_MASS_RADIUS = 0.377; // [m]
    public static final int ELBOW_NUMBER_OF_MOTORS = 2; // Arbitrary units
    public static final double ELBOW_ARM_LENGTH = 0.75889; //[m]

    // motor configuration
    public static final double VOLT_COMP_SATURATION = 10; //[V]
    public static final boolean ENABLE_VOLT_COMPENSATION = true;
    public static final double SETPOINT_DEADBAND = 1;
    public static final double SHOULDER_FALCON_TICKS_PER_REVOLUTION = 2048 * SHOULDER_GEARING;
    public static final double TICKS_PER_RADIAN_SHOULDER = SHOULDER_FALCON_TICKS_PER_REVOLUTION / (Math.PI * 2);
    public static final double ELBOW_FALCON_TICKS_PER_REVOLUTION = 2048 * ELBOW_GEARING;
    public static final double TICKS_PER_RADIAN_ELBOW = ELBOW_FALCON_TICKS_PER_REVOLUTION / (Math.PI * 2);
    public static final boolean MAIN_CLOCKWISE = true;
    public static final TalonFXInvertType AUX_CLOCKWISE = TalonFXInvertType.Clockwise;
    public static final TalonFXConfiguration shoulderMainMotorConfig = new TalonFXConfiguration();
    public static final TalonFXConfiguration shoulderAuxMotorConfig = new TalonFXConfiguration();
    public static final TalonFXConfiguration elbowMainMotorConfig = new TalonFXConfiguration();
    public static final TalonFXConfiguration elbowAuxMotorConfig = new TalonFXConfiguration();
    public static final double SHOULDER_FEED_FORWARD_MULTIPLIER = 0;
    public static final double ELBOW_FEED_FORWARD_MULTIPLIER = 0;
    public static final int SHOULDER_VELOCITY = 0;
    public static final int ELBOW_VELOCITY = 0;

    public static final Slot0Configs SHOULDER_PID= new Slot0Configs()
                .withKP(ArmConstants.shoulderP)
                .withKI(ArmConstants.shoulderI)
                .withKD(ArmConstants.shoulderD);

    public static final Slot1Configs ELBOW_PID = new Slot1Configs()
                .withKP(ArmConstants.elbowP)
                .withKI(ArmConstants.elbowI)
                .withKD(ArmConstants.elbowD);

    public static final double SENSOR_TO_MECHANISM_RATIO= 1;
    public static final double ROTOR_TO_SENSOR_RATIO= 1;




    public static final double ELBOW_ZERO_POSITION = 360 - 53.33; //[degrees]
    public static final double SHOULDER_ZERO_POSITION = 180 - 65.53; //[degrees]
    public static final double END_POSITION_UPPER_Y_LIMIT = 0; //[cm]
    public static final double END_POSITION_LOWER_Y_LIMIT = 0; //[cm]
    public static final double END_POSITION_UPPER_X_LIMIT = 0; //[cm]
    public static final double END_POSITION_LOWER_X_LIMIT = 0; //[cm]
    public static final double SHOULDER_ABSOLUTE_ENCODER_OFFSET = 0.4090 - SHOULDER_ZERO_POSITION / 360.0;
    public static final double ELBOW_ABSOLUTE_ENCODER_OFFSET = 0.57396956434 - ELBOW_ZERO_POSITION / 360.0;
    public static final double CURRENT_LIMIT = 10;


    //PID
    public static final double shoulderP = 4.0;
    public static final double shoulderI = 0.0;
    public static final double shoulderD = 0.0;
    public static final double elbowP = 1;
    public static final double elbowI = 0.0;
    public static final double elbowD = 0.0;

    public static final Translation3d shoulderOrigin = new Translation3d(-0.29, 0, 0.47);


    public static final double SHOULDER_FEED_FORWARD = 0.06;
    public static final double ELBOW_FEED_FORWARD = 0.04;

    //arm positions
    public static final Translation2d STARTING_POSITION = new Translation2d(-0.4, 0.06);
    public static final Translation2d FEEDER_POSITION = new Translation2d(0.2, 0.775);
    public static final Translation2d RETRACTED_POSITION = new Translation2d();
    //    public static final Translation2d UPPER_CONE_SCORING1 = new Translation2d(1.055, 0.914);
    public static final Translation2d UPPER_CONE_SCORING = new Translation2d(1.13, 0.92);
    //    public static final Translation2d MIDDLE_CONE_SCORING2 = new Translation2d(0.6945, 0.475);
    public static final Translation2d MIDDLE_CONE_SCORING2 = new Translation2d(0.929, 0.59);
    public static final Translation2d MIDDLE_CONE_SCORING1 = new Translation2d(0.28, 0.90);
    public static final Translation2d UPPER_CUBE_SCORING = new Translation2d(1.195, 0.741);
    public static final Translation2d MIDDLE_CUBE_SCORING = new Translation2d(0.8, 0.481);
    public static final Translation2d OUT_ROBOT1 = new Translation2d(-0.392, 0.0);
    public static final Translation2d IN_ROBOT1 = new Translation2d(-0.01, 0.8);
    public static final Translation2d IN_ROBOT2 = new Translation2d(-0.34, -0.13);
    public static final Translation2d OUT_ROBOT2 = new Translation2d(-0.4508, 0.3976);
    public static final Translation2d FLOOR_SCORING = new Translation2d();

    public static final SystemConstants.JointConstants SHOULDER_JOINT_CONSTANTS = new SystemConstants.JointConstants(
            SHOULDER_MASS, SHOULDER_LENGTH, SHOULDER_MOMENT_OF_INERTIA, SHOULDER_CENTER_OF_MASS_RADIUS, SHOULDER_GEARING, SHOULDER_NUMBER_OF_MOTORS);
    public static final SystemConstants.JointConstants ELBOW_JOINT_CONSTANTS = new SystemConstants.JointConstants(
            ELBOW_MASS, ELBOW_LENGTH, ELBOW_MOMENT_OF_INERTIA, ELBOW_CENTER_OF_MASS_RADIUS, ELBOW_GEARING, ELBOW_NUMBER_OF_MOTORS);

    public static final SystemConstants ARM_CONSTANTS = new SystemConstants(SHOULDER_JOINT_CONSTANTS, ELBOW_JOINT_CONSTANTS);

    public static final double FEEDER_DISTANCE = 23;
    public static final double FEEDER_MINIMUM_DISTANCE = 16;
}