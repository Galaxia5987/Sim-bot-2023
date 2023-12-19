package frc.robot.subsystems.arm;


import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Ports;
import utils.math.AngleUtil;

import static frc.robot.subsystems.arm.ArmConstants.*;


public class ArmIOReal implements ArmIO {

    private final TalonFX shoulderMainMotor;
    private final TalonFX shoulderAuxMotor;
    private final TalonFX elbowMainMotor;
    private final TalonFX elbowAuxMotor;
    private final DutyCycleEncoder shoulderEncoder;
    private final DutyCycleEncoder elbowEncoder;


    private static ArmIOReal INSTANCE = null;

    private final ArmKinematics kinematics = new ArmKinematics(ArmConstants.SHOULDER_ARM_LENGTH, ArmConstants.ELBOW_ARM_LENGTH);

    private double shoulderOffset = ArmConstants.SHOULDER_ABSOLUTE_ENCODER_OFFSET;
    private double elbowOffset = ArmConstants.ELBOW_ABSOLUTE_ENCODER_OFFSET;

    private double shoulderFeedforward;
    private double elbowFeedforward;

    private Command lastCommand;
    private boolean changedToDefaultCommand = false;

    private double ySetpoint = 0;

    private ControlRequest motorControlRequest = null;




    ArmIOReal() {

        this.shoulderEncoder = new DutyCycleEncoder(Ports.ArmPorts.SHOULDER_ENCODER);
        this.elbowEncoder = new DutyCycleEncoder(Ports.ArmPorts.ELBOW_ENCODER);

        this.shoulderMainMotor = new TalonFX(Ports.ArmPorts.SHOULDER_MAIN_MOTOR);

        this.shoulderAuxMotor = new TalonFX(Ports.ArmPorts.SHOULDER_AUX_MOTOR);

        this.elbowMainMotor = new TalonFX(Ports.ArmPorts.ELBOW_MAIN_MOTOR);

        this.elbowAuxMotor = new TalonFX(Ports.ArmPorts.ELBOW_AUX_MOTOR);



        shoulderAuxMotor.setControl(new StrictFollower(shoulderMainMotor.getDeviceID()));
        elbowAuxMotor.setControl(new StrictFollower(elbowMainMotor.getDeviceID()));


        shoulderMainMotorConfig.Voltage.PeakForwardVoltage = ArmConstants.VOLT_COMP_SATURATION;
        shoulderMainMotorConfig.Voltage.PeakReverseVoltage = -ArmConstants.VOLT_COMP_SATURATION;
        shoulderMainMotorConfig.Slot0 = SHOULDER_PID;
        elbowMainMotorConfig.CurrentLimits.SupplyCurrentLimit = ArmConstants.CURRENT_LIMIT;
        elbowMainMotorConfig.CurrentLimits.StatorCurrentLimit = ArmConstants.CURRENT_LIMIT;
        shoulderMainMotorConfig.Feedback.SensorToMechanismRatio = SENSOR_TO_MECHANISM_RATIO;
        shoulderMainMotorConfig.Feedback.RotorToSensorRatio = ROTOR_TO_SENSOR_RATIO;
        shoulderMainMotor.getConfigurator().apply(shoulderMainMotorConfig);

        shoulderMainMotor.setNeutralMode(NeutralModeValue.Brake);
        shoulderMainMotor.setInverted(MAIN_CLOCKWISE);

        elbowMainMotorConfig.Voltage.PeakForwardVoltage = ArmConstants.VOLT_COMP_SATURATION;
        elbowMainMotorConfig.Voltage.PeakReverseVoltage = -ArmConstants.VOLT_COMP_SATURATION;
        elbowMainMotorConfig.Slot1 = ELBOW_PID;
        elbowMainMotorConfig.CurrentLimits.SupplyCurrentLimit = ArmConstants.CURRENT_LIMIT;
        elbowMainMotorConfig.CurrentLimits.StatorCurrentLimit = ArmConstants.CURRENT_LIMIT;
        elbowMainMotorConfig.Feedback.SensorToMechanismRatio = SENSOR_TO_MECHANISM_RATIO;
        elbowMainMotorConfig.Feedback.RotorToSensorRatio = ROTOR_TO_SENSOR_RATIO;
        elbowMainMotor.getConfigurator().apply(elbowMainMotorConfig);

        elbowMainMotor.setNeutralMode(NeutralModeValue.Brake);
        elbowMainMotor.setInverted(MAIN_CLOCKWISE);

    }

    /**
     * Get the instance of the arm subsystem.
     *
     * @return Arm instance
     */
    public static ArmIOReal getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new ArmIOReal();
        }
        return INSTANCE;
    }


    /**
     * Sets the power of the shoulder motors.
     *
     * @param power desired power. [-1,1]
     */
    public void setShoulderJointPower(double power) {
        shoulderMainMotor.setControl(new DutyCycleOut(power));
    }

    public double getShoulderJointPower() {
        return shoulderMainMotor.get();
    }

    public double getElbowJointPower() {
        return elbowMainMotor.get();
    }

    /**
     * Sets the power of the elbow motors.
     *
     * @param power desired power. [-1,1]
     */
    public void setElbowJointPower(double power) {
        elbowMainMotor.setControl(new DutyCycleOut(power));
    }


    /**
     * Sets the angle of the shoulder joint.
     *
     * @param angle desired angle. [degrees]
     */
    public void setShoulderJointAngle(Rotation2d angle) {
        setShoulderJointAngle(angle.getRadians(), SHOULDER_FEED_FORWARD_MULTIPLIER, SHOULDER_VELOCITY);
    }


    private void setShoulderJointAngle(double angle, double ffMultiplier, int velocity) {
        angle = AngleUtil.normalize(angle);
        Rotation2d error = new Rotation2d(angle).minus(new Rotation2d(currentShoulderAngle));
        if (shoulderEncoder.isConnected()) {
            motorControlRequest = new PositionVoltage(angle);
            shoulderMainMotor.setControl(new PositionDutyCycle(angle, velocity, true,
                    shoulderFeedforward * ffMultiplier, 0, true)
                    .withEnableFOC(true));
        } else {
            shoulderMainMotor.stopMotor();
        }
    }


    public Rotation2d getShoulderJointAngle() {
        return Rotation2d.fromRotations(shoulderEncoder.getAbsolutePosition() - shoulderOffset);
    }

    public Rotation2d getElbowJointAngle() {
        return Rotation2d.fromRotations(elbowEncoder.getAbsolutePosition() - elbowOffset);
    }

    /**
     * Sets the angle of the elbow joint.
     *
     * @param angle desired angle. [degrees]
     */
    public void setElbowJointAngle(Rotation2d angle) {
        setElbowJointAngle(angle.getRadians(), ELBOW_FEED_FORWARD, ELBOW_VELOCITY);
    }

    /**
     * Sets the angle of the elbow joint.
     *
     * @param angle desired angle. [degrees]
     */
    private void setElbowJointAngle(double angle, double ffMultiplier, int velocity) {
        angle = AngleUtil.normalize(angle);
        Rotation2d error = new Rotation2d(angle).minus(new Rotation2d(currentShoulderAngle));
        if (elbowEncoder.isConnected()) {
            motorControlRequest = new PositionVoltage(angle);
            elbowMainMotor.setControl(new PositionDutyCycle(angle, velocity, true,
                    elbowFeedforward * ffMultiplier, 1, true)
                    .withEnableFOC(true));
        } else {
            elbowMainMotor.stopMotor();
        }
    }

    /**
     * Calculates the position of the end of the arm.
     *
     * @return Translation2d of the position.
     */
    public Translation2d getEndPosition() {
        Rotation2d shoulderAngle = getShoulderJointAngle();
        Rotation2d elbowAngle = getElbowJointAngle();
        return kinematics.forwardKinematics(shoulderAngle.getRadians(),shoulderAngle.getRadians() + elbowAngle.getRadians() - Math.PI);
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
        setShoulderJointAngle(Math.toDegrees(angles.shoulderAngle), shoulderFFMultiplier, 0);
        setElbowJointAngle(Math.toDegrees(angles.elbowAngle), elbowFFMultiplier, 0);
        ySetpoint = armLocation.getY();
    }

    /**
     * Gets the velocity of the shoulder motors.
     *
     * @return shoulder motors velocity. [rad/sec]
     */
    public double getShoulderMotorVelocity() {
        return shoulderMainMotor.getVelocity().getValue();
    }

    public void setFinalSetpointAngles(Translation2d position, ArmInputs inputs) {
        var solution = kinematics.inverseKinematics(position);
        inputs.finalSetpointAngles = new Translation2d( Math.toDegrees(solution.shoulderAngle), Math.toDegrees(solution.elbowAngle));
    }

    /**
     * Gets the velocity of the elbow motors.
     *
     * @return elbow motor velocity. [rad/sec]
     */
    public double getElbowMotorVelocity() {
        return elbowMainMotor.getVelocity().getValue();
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
                        SHOULDER_ARM_LENGTH,
                        shoulderAngle
                );
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

    public boolean changedToDefaultCommand() {
        return changedToDefaultCommand;
    }

    /**
     * Stops the motors.
     */
    public void stop() {
        shoulderMainMotor.stopMotor();
        elbowMainMotor.stopMotor();
    }

    @Override
    public String getSubsystemName() {
        return "Arm";
    }


    @Override
    public void updateInputs(ArmInputs inputs) {
        inputs.shoulderAngle = getShoulderJointAngle().getDegrees();
        inputs.elbowAngle = getElbowJointAngle().getDegrees();
        inputs.shoulderMotorPower = getShoulderJointPower();
        inputs.elbowMotorPower = getElbowJointPower();
        inputs.shoulderEncoderPosition = shoulderEncoder.getAbsolutePosition();
        inputs.elbowEncoderPosition = elbowEncoder.getAbsolutePosition();
        inputs.shoulderVelocity = getShoulderMotorVelocity();
        inputs.elbowVelocity = getElbowMotorVelocity();
        Translation2d position = getEndPosition();
        inputs.armPosition[0] = position.getX();
        inputs.armPosition[1] = position.getY();
        ArmKinematics.InverseKinematicsSolution kinematicsSolution = kinematics.inverseKinematics(position);
        inputs.inverseKinematicsSolution[0] = Math.toDegrees(kinematicsSolution.shoulderAngle);
        inputs.inverseKinematicsSolution[1] = Math.toDegrees(kinematicsSolution.elbowAngle);
        inputs.ySetpoint = ySetpoint;

        inputs.feedforward[0] = shoulderFeedforward;
        inputs.feedforward[1] = elbowFeedforward;

    }

    public ArmKinematics getKinematics() {
        return kinematics;
    }
}

