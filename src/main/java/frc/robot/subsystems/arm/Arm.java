package frc.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Ports;
import frc.robot.subsystems.LoggedSubsystem;
import frc.robot.subsystems.arm.commands.ArmXboxControl;
import frc.robot.utils.math.AngleUtil;
import frc.robot.utils.units.UnitModel;
import org.littletonrobotics.junction.Logger;

public class Arm extends LoggedSubsystem<ArmInputsAutoLogged> {
    private static Arm INSTANCE = null;

    private final ArmKinematics kinematics = new ArmKinematics(ArmConstants.SHOULDER_ARM_LENGTH, ArmConstants.ELBOW_ARM_LENGTH);

    private final TalonFX shoulderMainMotor = new TalonFX(Ports.ArmPorts.SHOULDER_MAIN_MOTOR);
    private final TalonFX shoulderAuxMotor = new TalonFX(Ports.ArmPorts.SHOULDER_AUX_MOTOR);
    private final DutyCycleEncoder shoulderEncoder = new DutyCycleEncoder(Ports.ArmPorts.SHOULDER_ENCODER);

    private final TalonFX elbowMainMotor = new TalonFX(Ports.ArmPorts.ELBOW_MAIN_MOTOR);
    private final TalonFX elbowAuxMotor = new TalonFX(Ports.ArmPorts.ELBOW_AUX_MOTOR);
    private final DutyCycleEncoder elbowEncoder = new DutyCycleEncoder(Ports.ArmPorts.ELBOW_ENCODER);

    private final UnitModel unitModelShoulder = new UnitModel(ArmConstants.TICKS_PER_RADIAN_SHOULDER);
    private final UnitModel unitModelElbow = new UnitModel(ArmConstants.TICKS_PER_RADIAN_ELBOW);

    private double shoulderOffset = ArmConstants.SHOULDER_ABSOLUTE_ENCODER_OFFSET;
    private double elbowOffset = ArmConstants.ELBOW_ABSOLUTE_ENCODER_OFFSET;

    private double shoulderFeedforward;
    private double elbowFeedforward;

    private double ySetpoint = 0;

    private Arm() {
        super(new ArmInputsAutoLogged());
        shoulderMainMotor.configFactoryDefault();
        shoulderAuxMotor.configFactoryDefault();
        elbowMainMotor.configFactoryDefault();
        elbowAuxMotor.configFactoryDefault();

        shoulderMainMotor.setInverted(TalonFXInvertType.CounterClockwise);
        elbowMainMotor.setInverted(ArmConstants.MAIN_CLOCKWISE);

        configureMainMotor(shoulderMainMotor,
                ArmConstants.shoulderP,
                ArmConstants.shoulderI,
                ArmConstants.shoulderD);
        configureAuxMotor(shoulderAuxMotor, shoulderMainMotor);

        configureMainMotor(elbowMainMotor,
                ArmConstants.elbowP,
                ArmConstants.elbowI,
                ArmConstants.elbowD);
        configureAuxMotor(elbowAuxMotor, elbowMainMotor);

//        shoulderMainMotor.config_IntegralZone(0, 0.0001);
//        shoulderMainMotor.setIntegralAccumulator(1 / 3.0 * ArmConstants.SHOULDER_FALCON_TICKS_PER_REVOLUTION / 360.0);
//
//        elbowMainMotor.config_IntegralZone(0, 0.0001);
//        elbowMainMotor.setIntegralAccumulator(2 / 3.0 * ArmConstants.ELBOW_FALCON_TICKS_PER_REVOLUTION / 360.0);

//        shoulderMainMotor.configClosedLoopPeakOutput(0, 0.5);
//        elbowMainMotor.configClosedLoopPeakOutput(0, 0.5);

        for (int i = 1; i <= 17; i++) {
            shoulderAuxMotor.setStatusFramePeriod(i, 500);
            elbowAuxMotor.setStatusFramePeriod(i, 500);
        }
    }

    /**
     * Get the instance of the arm subsystem.
     *
     * @return Arm instance
     */
    public static Arm getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new Arm();
        }
        return INSTANCE;
    }

    /**
     * Configures the aux motors.
     */
    private void configureAuxMotor(TalonFX auxMotor, TalonFX mainMotor) {
        auxMotor.follow(mainMotor);
        auxMotor.enableVoltageCompensation(ArmConstants.ENABLE_VOLT_COMPENSATION);
        auxMotor.configVoltageCompSaturation(ArmConstants.VOLT_COMP_SATURATION);
        mainMotor.configNeutralDeadband(0);
        auxMotor.setNeutralMode(NeutralMode.Brake);
        auxMotor.setInverted(TalonFXInvertType.FollowMaster);
    }

    /**
     * Configures the main motors.
     */
    private void configureMainMotor(TalonFX mainMotor, double kP, double kI, double kD) {
        mainMotor.enableVoltageCompensation(ArmConstants.ENABLE_VOLT_COMPENSATION);
        mainMotor.configVoltageCompSaturation(ArmConstants.VOLT_COMP_SATURATION);
        mainMotor.setNeutralMode(NeutralMode.Brake);
        mainMotor.configNeutralDeadband(0);

        mainMotor.config_kP(0, kP);
        mainMotor.config_kI(0, kI);
        mainMotor.config_kD(0, kD);
    }

    /**
     * Sets the power of the shoulder motors.
     *
     * @param power desired power. [-1,1]
     */
    public void setShoulderJointPower(double power) {
        shoulderMainMotor.set(TalonFXControlMode.PercentOutput, power);
    }

    /**
     * Sets the power of the elbow motors.
     *
     * @param power desired power. [-1,1]
     */
    public void setElbowJointPower(double power) {
        elbowMainMotor.set(TalonFXControlMode.PercentOutput, power);
    }

    /**
     * Gets the angle of the shoulder joint.
     *
     * @return angle of the shoulder joint. [rad]
     */
    public Rotation2d getShoulderJointAngle() {
        return AngleUtil.normalize(Rotation2d.fromRotations(shoulderEncoder.getAbsolutePosition() - shoulderOffset));
    }

    /**
     * Sets the angle of the shoulder joint.
     *
     * @param angle desired angle. [degrees]
     */
    public void setShoulderJointAngle(double angle) {
        setShoulderJointAngle(angle, 0);
    }

    /**
     * Sets the angle of the shoulder joint.
     *
     * @param angle desired angle. [degrees]
     */
    public void setShoulderJointAngle(double angle, double ffMultiplier) {
        angle = AngleUtil.normalize(Rotation2d.fromDegrees(angle)).getDegrees();
        double error = unitModelShoulder.toTicks(Math.toRadians(angle)) - unitModelShoulder.toTicks(getShoulderJointAngle().getRadians());
        if (shoulderEncoder.isConnected()) {
            shoulderMainMotor.set(TalonFXControlMode.Position, shoulderMainMotor.getSelectedSensorPosition() + error,
                    DemandType.ArbitraryFeedForward, ffMultiplier * shoulderFeedforward);
        } else {
            shoulderMainMotor.neutralOutput();
        }
        loggerInputs.shoulderSetpoint = angle;
        loggerInputs.shoulderError = error;
    }

    /**
     * Gets the angle of the elbow joint.
     *
     * @return elbow joint angle. [rad]
     */
    public Rotation2d getElbowJointAngle() {
        return AngleUtil.normalize(Rotation2d.fromRotations(elbowEncoder.getAbsolutePosition() - elbowOffset));
    }

    /**
     * Sets the angle of the elbow joint.
     *
     * @param angle desired angle. [degrees]
     */
    public void setElbowJointAngle(double angle) {
        setElbowJointAngle(angle, 0);
    }

    /**
     * Sets the angle of the elbow joint.
     *
     * @param angle desired angle. [degrees]
     */
    public void setElbowJointAngle(double angle, double ffMultiplier) {
        angle = AngleUtil.normalize(Rotation2d.fromDegrees(angle)).getDegrees();
        double error = unitModelElbow.toTicks(Math.toRadians(angle)) - unitModelElbow.toTicks(getElbowJointAngle().getRadians());
        if (elbowEncoder.isConnected()) {
            elbowMainMotor.set(TalonFXControlMode.Position, elbowMainMotor.getSelectedSensorPosition() + error,
                    DemandType.ArbitraryFeedForward, ffMultiplier * elbowFeedforward);
        } else {
            elbowMainMotor.neutralOutput();
        }
        loggerInputs.elbowSetpoint = angle;
        loggerInputs.elbowError = error;
    }

    /**
     * Calculates the position of the end of the arm.
     *
     * @return Translation2d of the position.
     */
    public Translation2d getEndPosition() {
        double shoulderAngle = getShoulderJointAngle().getRadians();
        double elbowAngle = getElbowJointAngle().getRadians();
        return kinematics.forwardKinematics(shoulderAngle, shoulderAngle + elbowAngle - Math.PI);
    }

    /**
     * Sets the position of the end of the arm.
     *
     * @param armLocation Translation2d of the desired location.
     */
    public void setEndPosition(Translation2d armLocation) {
        setEndPosition(armLocation, 0, 0);
    }

    /**
     * Sets the position of the end of the arm.
     *
     * @param armLocation Translation2d of the desired location.
     */
    public void setEndPosition(Translation2d armLocation, double shoulderFFMultiplier, double elbowFFMultiplier) {
        var angles = kinematics.inverseKinematics(armLocation);
        setShoulderJointAngle(Math.toDegrees(angles.shoulderAngle), shoulderFFMultiplier);
        setElbowJointAngle(Math.toDegrees(angles.elbowAngle), elbowFFMultiplier);
        ySetpoint = armLocation.getY();
    }

    /**
     * Gets the velocity of the shoulder motors.
     *
     * @return shoulder motors velocity. [rad/sec]
     */
    public double getShoulderMotorVelocity() {
        return unitModelShoulder.toVelocity(shoulderMainMotor.getSelectedSensorVelocity());
    }

    public void setFinalSetpointAngles(Translation2d position) {
        var solution = kinematics.inverseKinematics(position);
        loggerInputs.finalSetpointAngles[0] = Math.toDegrees(solution.shoulderAngle);
        loggerInputs.finalSetpointAngles[1] = Math.toDegrees(solution.elbowAngle);
    }

    /**
     * Gets the velocity of the elbow motors.
     *
     * @return elbow motor velocity. [rad/sec]
     */
    public double getElbowMotorVelocity() {
        return unitModelElbow.toVelocity(elbowMainMotor.getSelectedSensorVelocity());
    }

    public void resetArmEncoders() {
        double elbowAngleReset = elbowEncoder.getAbsolutePosition() * 360.0 - ArmConstants.ELBOW_ZERO_POSITION;
        double shoulderAngleReset = shoulderEncoder.getAbsolutePosition() * 360.0 - ArmConstants.SHOULDER_ZERO_POSITION;
        shoulderOffset = AngleUtil.normalize(shoulderAngleReset) / 360.0;
        elbowOffset = AngleUtil.normalize(elbowAngleReset) / 360.0;
    }

    public Translation2d getElbowJointPosition() {
        Rotation2d shoulderAngle = getShoulderJointAngle();
        return new Translation2d(
                ArmConstants.SHOULDER_ARM_LENGTH * shoulderAngle.getCos(),
                ArmConstants.SHOULDER_ARM_LENGTH * shoulderAngle.getSin());
    }

    public boolean armIsOutOfFrame() {
        Translation2d elbowJoint = getElbowJointPosition(), endPosition = getEndPosition();
        return !(elbowJoint.getX() < 0) || !(endPosition.getX() < 0);
    }

    public boolean armIsInRobot() {
        return getEndPosition().getY() < 0 && getShoulderJointAngle().getDegrees() > 90;
    }

    public void setVelocity(Translation2d velocity) {
        var armVelocities = kinematics.getVelocities(getEndPosition(), velocity);
        setShoulderJointPower(armVelocities.shoulderAngle / Math.PI);
        setElbowJointPower(armVelocities.elbowAngle / (2 * Math.PI));
    }

    /**
     * Stops the motors.
     */
    public void stop() {
        shoulderMainMotor.neutralOutput();
        elbowMainMotor.neutralOutput();
    }
    private Command lastCommand = null;
    private Command currentCommand = null;
    private boolean changedToDefaultCommand = false;
    public boolean changedToDefaultCommand() {
        return changedToDefaultCommand;
    }

    @Override
    public String getSubsystemName() {
        return "Arm";
    }

    @Override
    public void periodic() {
        currentCommand = getCurrentCommand();

        if (currentCommand != null) {
            Logger.getInstance().recordOutput("ArmCommand", currentCommand.getName());
        }

        SmartDashboard.putString("Arm Encoder Offset", "{" + shoulderOffset + ", " + elbowOffset + "}");
        Translation2d position = getEndPosition();
        Rotation2d angle = new Rotation2d(position.getX(), position.getY());
        shoulderFeedforward = ArmConstants.SHOULDER_FEED_FORWARD *
                position.getNorm() * angle.getCos();
        elbowFeedforward = ArmConstants.ELBOW_FEED_FORWARD *
                getElbowJointAngle()
                        .plus(getShoulderJointAngle())
                        .minus(Rotation2d.fromDegrees(180))
                        .getCos() * ArmConstants.ELBOW_ARM_LENGTH;

        changedToDefaultCommand = !(lastCommand instanceof ArmXboxControl) && (currentCommand instanceof ArmXboxControl);

        lastCommand = currentCommand;

    }

    @Override
    public void updateInputs() {
        loggerInputs.shoulderAngle = getShoulderJointAngle().getDegrees();
        loggerInputs.elbowAngle = getElbowJointAngle().getDegrees();
        loggerInputs.shoulderMotorPower = shoulderMainMotor.getMotorOutputPercent();
        loggerInputs.elbowMotorPower = elbowMainMotor.getMotorOutputPercent();
        loggerInputs.shoulderEncoderPosition = shoulderEncoder.getAbsolutePosition();
        loggerInputs.elbowEncoderPosition = elbowEncoder.getAbsolutePosition();
        loggerInputs.shoulderVelocity = getShoulderMotorVelocity();
        loggerInputs.elbowVelocity = getElbowMotorVelocity();
        Translation2d position = getEndPosition();
        loggerInputs.armPosition[0] = position.getX();
        loggerInputs.armPosition[1] = position.getY();
        ArmKinematics.InverseKinematicsSolution kinematicsSolution = kinematics.inverseKinematics(position);
        loggerInputs.inverseKinematicsSolution[0] = Math.toDegrees(kinematicsSolution.shoulderAngle);
        loggerInputs.inverseKinematicsSolution[1] = Math.toDegrees(kinematicsSolution.elbowAngle);
        loggerInputs.ySetpoint = ySetpoint;

        loggerInputs.feedforward[0] = shoulderFeedforward;
        loggerInputs.feedforward[1] = elbowFeedforward;

        loggerInputs.shoulderOutputVoltage = shoulderMainMotor.getMotorOutputVoltage();
    }

    public ArmKinematics getKinematics() {
        return kinematics;
    }
}