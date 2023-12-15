package frc.robot.subsystems.arm;


import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Ports;
import frc.robot.subsystems.arm.commands.ArmXboxControl;
import org.littletonrobotics.junction.Logger;
import utils.math.AngleUtil;


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

    private double currentShoulderAngle;
    private double currentElbowAngle;


    ArmIOReal() {
        this.shoulderEncoder = new DutyCycleEncoder(Ports.ArmPorts.SHOULDER_ENCODER);
        this.elbowEncoder = new DutyCycleEncoder(Ports.ArmPorts.ELBOW_ENCODER);

        this.shoulderMainMotor = new TalonFX(Ports.ArmPorts.SHOULDER_MAIN_MOTOR);
        TalonFXConfiguration shoulderMainMotorConfig = new TalonFXConfiguration();

        this.shoulderAuxMotor = new TalonFX(Ports.ArmPorts.SHOULDER_AUX_MOTOR);
        TalonFXConfiguration shoulderAuxMotorConfig = new TalonFXConfiguration();

        this.elbowMainMotor = new TalonFX(Ports.ArmPorts.ELBOW_MAIN_MOTOR);
        TalonFXConfiguration elbowMainMotorConfig = new TalonFXConfiguration();

        this.elbowAuxMotor = new TalonFX(Ports.ArmPorts.ELBOW_AUX_MOTOR);
        TalonFXConfiguration elbowAuxMotorConfig = new TalonFXConfiguration();

        Slot0Configs shoulderPID = new Slot0Configs()
                .withKP(ArmConstants.shoulderP)
                .withKI(ArmConstants.shoulderI)
                .withKD(ArmConstants.shoulderD);

        Slot1Configs elbowPID = new Slot1Configs()
                .withKP(ArmConstants.elbowP)
                .withKI(ArmConstants.elbowI)
                .withKD(ArmConstants.elbowD);


        shoulderAuxMotor.setControl(new StrictFollower(shoulderMainMotor.getDeviceID()));
        elbowAuxMotor.setControl(new StrictFollower(elbowMainMotor.getDeviceID()));


        shoulderMainMotorConfig.Voltage.PeakForwardVoltage = ArmConstants.VOLT_COMP_SATURATION;
        shoulderMainMotorConfig.Voltage.PeakReverseVoltage = -ArmConstants.VOLT_COMP_SATURATION;
        shoulderMainMotorConfig.Slot0 = shoulderPID;
        elbowMainMotorConfig.CurrentLimits.SupplyCurrentLimit = ArmConstants.CURRENT_LIMIT.currentLimit;
        elbowMainMotorConfig.CurrentLimits.StatorCurrentLimit = ArmConstants.CURRENT_LIMIT.currentLimit;
        shoulderMainMotorConfig.Feedback.SensorToMechanismRatio = 1;
        shoulderMainMotorConfig.Feedback.RotorToSensorRatio = 1;
        shoulderMainMotor.getConfigurator().apply(shoulderMainMotorConfig);

        shoulderMainMotor.setNeutralMode(NeutralModeValue.Brake);
        shoulderMainMotor.setInverted(true);

        elbowMainMotorConfig.Voltage.PeakForwardVoltage = ArmConstants.VOLT_COMP_SATURATION;
        elbowMainMotorConfig.Voltage.PeakReverseVoltage = -ArmConstants.VOLT_COMP_SATURATION;
        elbowMainMotorConfig.Slot1 = elbowPID;
        elbowMainMotorConfig.CurrentLimits.SupplyCurrentLimit = ArmConstants.CURRENT_LIMIT.currentLimit;
        elbowMainMotorConfig.CurrentLimits.StatorCurrentLimit = ArmConstants.CURRENT_LIMIT.currentLimit;
        elbowMainMotorConfig.Feedback.SensorToMechanismRatio = 1;
        elbowMainMotorConfig.Feedback.RotorToSensorRatio = 1;
        shoulderMainMotor.getConfigurator().apply(shoulderMainMotorConfig);

        shoulderMainMotor.setNeutralMode(NeutralModeValue.Brake);
        shoulderMainMotor.setInverted(true);
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
    public void setShoulderJointAngle(double angle) {
        setShoulderJointAngle(angle, 0, 0);
    }


    public void setShoulderJointAngle(double angle, double ffMultiplier, int velocity) {
        angle = AngleUtil.normalize(angle);
        Rotation2d error = new Rotation2d(angle).minus(new Rotation2d(currentShoulderAngle));
        if (shoulderEncoder.isConnected()) {
            motorControlRequest = new PositionVoltage(angle);
            shoulderMainMotor.setControl(new PositionVoltage(angle, velocity, true, shoulderFeedforward * ffMultiplier, 0, true));

        } else {
            shoulderMainMotor.stopMotor();
        }
    }

    /**
     * Gets the angle of the elbow joint.
     *
     * @return elbow joint angle. [rad]
     */
//    public Rotation2d getElbowJointAngle() {
////        return AngleUtil.normalize((Rotation2d.fromRotations(elbowEncoder.getAbsolutePosition() - elbowOffset)));
//        return (elbowEncoder.getAbsolutePosition().);
//    }
    public Rotation2d getShoulderJointAngle() {
        return Rotation2d.fromRotations(shoulderEncoder.getAbsolutePosition() - shoulderOffset);
    }

        public Rotation2d getElbowJointAngle () {
            return Rotation2d.fromRotations(elbowEncoder.getAbsolutePosition() - elbowOffset);
        }

            /**
             * Sets the angle of the elbow joint.
             *
             * @param angle desired angle. [degrees]
             */
            public void setElbowJointAngle ( double angle){
                setElbowJointAngle(angle, 0, 0);
            }

            /**
             * Sets the angle of the elbow joint.
             *
             * @param angle desired angle. [degrees]
             */
            public void setElbowJointAngle ( double angle, double ffMultiplier, int velocity){
                angle = AngleUtil.normalize(angle);
                Rotation2d error = new Rotation2d(angle).minus(new Rotation2d(currentShoulderAngle));
                if (elbowEncoder.isConnected()) {
                    motorControlRequest = new PositionVoltage(angle);
                    elbowMainMotor.setControl(new PositionVoltage(angle, velocity, true, shoulderFeedforward * ffMultiplier, 1, true));

                } else {
                    elbowMainMotor.stopMotor();
                }
            }

            /**
             * Calculates the position of the end of the arm.
             *
             * @return Translation2d of the position.
             */
            public Translation2d getEndPosition () {
                double shoulderAngle = getShoulderJointAngle().getRadians();
                double elbowAngle = getElbowJointAngle().getRadians();
                return kinematics.forwardKinematics(shoulderAngle, shoulderAngle + elbowAngle - Math.PI);
            }

            /**
             * Sets the position of the end of the arm.
             *
             * @param armLocation Translation2d of the desired location.
             */
            public void setEndPosition (Translation2d armLocation){
                setEndPosition(armLocation, 0, 0);
            }

            /**
             * Sets the position of the end of the arm.
             *
             * @param armLocation Translation2d of the desired location.
             */
            public void setEndPosition (Translation2d armLocation,double shoulderFFMultiplier, double elbowFFMultiplier)
            {
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
            public double getShoulderMotorVelocity () {
                return shoulderMainMotor.getVelocity().getValue();
            }

            public void setFinalSetpointAngles (Translation2d position, ArmInputsLogged inputs){
                var solution = kinematics.inverseKinematics(position);
                inputs.finalSetpointAngles[0] = Math.toDegrees(solution.shoulderAngle);
                inputs.finalSetpointAngles[1] = Math.toDegrees(solution.elbowAngle);
            }

            /**
             * Gets the velocity of the elbow motors.
             *
             * @return elbow motor velocity. [rad/sec]
             */
            public double getElbowMotorVelocity () {
                return elbowMainMotor.getVelocity().getValue();
            }

            public void resetArmEncoders () {
                double elbowAngleReset = elbowEncoder.getAbsolutePosition() * 360.0 - ArmConstants.ELBOW_ZERO_POSITION;
                double shoulderAngleReset = shoulderEncoder.getAbsolutePosition() * 360.0 - ArmConstants.SHOULDER_ZERO_POSITION;
                shoulderOffset = AngleUtil.normalize(shoulderAngleReset) / 360.0;
                elbowOffset = AngleUtil.normalize(elbowAngleReset) / 360.0;
            }

            public Translation2d getElbowJointPosition () {
                Rotation2d shoulderAngle = getShoulderJointAngle();
                return new Translation2d(
                        ArmConstants.SHOULDER_ARM_LENGTH * shoulderAngle.getCos(),
                        ArmConstants.SHOULDER_ARM_LENGTH * shoulderAngle.getSin());
            }

            public boolean armIsOutOfFrame () {
                Translation2d elbowJoint = getElbowJointPosition(), endPosition = getEndPosition();
                return !(elbowJoint.getX() < 0) || !(endPosition.getX() < 0);
            }

            public boolean armIsInRobot () {
                return getEndPosition().getY() < 0 && getShoulderJointAngle().getDegrees() > 90;
            }

            public void setVelocity (Translation2d velocity){
                var armVelocities = kinematics.getVelocities(getEndPosition(), velocity);
                setShoulderJointPower(armVelocities.shoulderAngle / Math.PI);
                setElbowJointPower(armVelocities.elbowAngle / (2 * Math.PI));
            }

            public boolean changedToDefaultCommand () {
                return changedToDefaultCommand;
            }

            /**
             * Stops the motors.
             */
            public void stop () {
                shoulderMainMotor.set(0);
                elbowMainMotor.set(0);
            }

            @Override
            public String getSubsystemName () {
                return "Arm";
            }


            @Override
            public void updateInputs (ArmInputsLogged inputs){
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

            public ArmKinematics getKinematics () {
                return kinematics;
            }
        }

