package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.math.geometry.Translation2d;
import utils.units.Units;

public class IntakeConstants {
    public static final double ANGLE_GEAR_RATIO = 35.26;
    public static final double SPIN_GEAR_RATIO = 10; //Not a real value
    public static final double kP = 0.1;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kF = 0; // not relevant anymore
    public static final Slot0Configs PIDGains = new Slot0Configs().withKP(IntakeConstants.kP).withKI(IntakeConstants.kI).withKD(IntakeConstants.kD);

    public static final double FALCON_TICKS_PER_ROTATION = 2048;
    public static final double TICKS_PER_DEGREE = FALCON_TICKS_PER_ROTATION / 360 * ANGLE_GEAR_RATIO;

    public static final double ANGLE_UP = -5.5; //[deg]
    public static final double ANGLE_DOWN = -99; //[deg]

    public static final double INTAKE_POWER = 0.8; //[%]
    public static final double INTAKE_ANGLE_VELOCITY = Units.rpmToRps(1100) * FALCON_TICKS_PER_ROTATION * 10; //[RPM]
    public static final double INTAKE_ANGLE_MAX_ACCELERATION = INTAKE_ANGLE_VELOCITY / 0.5; //[RPM/s]
    public static final double MAX_CURRENT = 8;

    public static final double ANGLE_MOTOR_POWER = 0.3;
    public static final int INTAKE_MECH_OFFSET = 90; //Degrees
    public static final double AXIS_RADIUS = 0.5143; //Meters
    public static final double INTAKE_X = 0.261;
    public static final double INTAKE_Y = 0.443;
    public static final double INTAKE_SIM_ANGLE_OFFSET = -44.8;
    public static final Translation2d ROOT_POSITION = new Translation2d(INTAKE_X, INTAKE_Y);
}
