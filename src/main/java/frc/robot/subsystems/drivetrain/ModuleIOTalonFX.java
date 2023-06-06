package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.utils.Utils;
import frc.robot.utils.math.differential.BooleanTrigger;

import static frc.robot.Constants.TALON_TIMEOUT;
import static frc.robot.subsystems.drivetrain.SwerveConstants.*;

public class ModuleIOTalonFX implements ModuleIO {

    private final TalonFX driveMotor;
    private final TalonFX angleMotor;

    private final DutyCycleEncoder encoder;
    private final double offsetRads;
    private final BooleanTrigger encoderConnectionTrigger =
            new BooleanTrigger(false, false);
    private boolean initializedAngleFalcon = false;

    private double[] motionMagicConfigs = new double[10];

    public ModuleIOTalonFX(int drivePort, boolean driveInverted,
                           int anglePort, boolean angleInverted,
                           int encoderPort, double offsetRads) {
        driveMotor = new TalonFX(drivePort);
        angleMotor = new TalonFX(anglePort);

        encoder = new DutyCycleEncoder(encoderPort);
        this.offsetRads = offsetRads;

        driveMotor.configFactoryDefault(TALON_TIMEOUT);
        angleMotor.configFactoryDefault(TALON_TIMEOUT);

        driveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, TALON_TIMEOUT);
        driveMotor.setInverted(driveInverted);
        driveMotor.setNeutralMode(NeutralMode.Brake);
        driveMotor.selectProfileSlot(1, 0);
        driveMotor.configNeutralDeadband(NEUTRAL_DEADBAND, TALON_TIMEOUT);
        driveMotor.configSupplyCurrentLimit(SUPPLY_CURRENT_LIMIT_CONFIG, TALON_TIMEOUT);
        driveMotor.configStatorCurrentLimit(STATOR_CURRENT_LIMIT_CONFIG, TALON_TIMEOUT);
        driveMotor.configGetSupplyCurrentLimit(SUPPLY_CURRENT_LIMIT_CONFIG, TALON_TIMEOUT);
        driveMotor.configGetStatorCurrentLimit(STATOR_CURRENT_LIMIT_CONFIG, TALON_TIMEOUT);
        driveMotor.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToZero, TALON_TIMEOUT);
        driveMotor.setStatusFramePeriod(1, 50);
        driveMotor.setStatusFramePeriod(3, 500);

        angleMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, TALON_TIMEOUT);
        angleMotor.configFeedbackNotContinuous(false, TALON_TIMEOUT);
        angleMotor.setInverted(angleInverted);
        angleMotor.configSupplyCurrentLimit(SUPPLY_CURRENT_LIMIT_CONFIG, TALON_TIMEOUT);
        angleMotor.configStatorCurrentLimit(STATOR_CURRENT_LIMIT_CONFIG, TALON_TIMEOUT);
        angleMotor.configGetSupplyCurrentLimit(SUPPLY_CURRENT_LIMIT_CONFIG, TALON_TIMEOUT);
        angleMotor.configGetStatorCurrentLimit(STATOR_CURRENT_LIMIT_CONFIG, TALON_TIMEOUT);
        angleMotor.setStatusFramePeriod(1, 50);
        angleMotor.setStatusFramePeriod(3, 500);

        for (int i = 5; i <= 17; i++) {
            driveMotor.setStatusFramePeriod(i, 500);
            angleMotor.setStatusFramePeriod(i, 500);
        }

        angleMotor.setNeutralMode(NeutralMode.Brake);
        angleMotor.selectProfileSlot(0, 0);
    }

    @Override
    public void updateInputs(ModuleInputs inputs) {
        inputs.angleRads = SwerveModuleUtils
                .getModuleAngleFromFalcon(angleMotor.getSelectedSensorPosition());
        inputs.encoderAngleRads = SwerveModuleUtils
                .getModuleAngleFromEncoder(encoder.getAbsolutePosition());
        inputs.encoderConnected = encoder.isConnected();
        inputs.appliedAngleVoltage = angleMotor.getMotorOutputVoltage();
        inputs.appliedAngleCurrent = angleMotor.getSupplyCurrent();

        inputs.velocityMetersPerSecond = SwerveModuleUtils
                .getModuleVelocity(driveMotor.getSelectedSensorVelocity());
        inputs.moduleDistanceMeters = SwerveModuleUtils
                .getModuleDistance(driveMotor.getSelectedSensorPosition());
        inputs.appliedDriveVoltage = driveMotor.getMotorOutputVoltage();
        inputs.appliedDriveCurrent = driveMotor.getSupplyCurrent();

        encoderConnectionTrigger.update(inputs.encoderConnected);
    }

    @Override
    public void setAngle(double angleRads) {
        double ticks = SwerveModuleUtils.getFalconTicksFromAngle(angleRads);
        angleMotor.set(TalonFXControlMode.MotionMagic, ticks);
    }

    @Override
    public void setDriveVelocity(double velocity) {
        driveMotor.set(TalonFXControlMode.Velocity, velocity);
    }

    @Override
    public void configMotionMagic(double[] motionMagicConfigs) {
        if (!Utils.epsilonEquals(this.motionMagicConfigs[0], motionMagicConfigs[0])) {
            angleMotor.config_kP(0, motionMagicConfigs[0], TALON_TIMEOUT);
        }
        if (!Utils.epsilonEquals(this.motionMagicConfigs[1], motionMagicConfigs[1])) {
            angleMotor.config_kI(0, motionMagicConfigs[1], TALON_TIMEOUT);
        }
        if (!Utils.epsilonEquals(this.motionMagicConfigs[2], motionMagicConfigs[2])) {
            angleMotor.config_kD(0, motionMagicConfigs[2], TALON_TIMEOUT);
        }
        if (!Utils.epsilonEquals(this.motionMagicConfigs[3], motionMagicConfigs[3])) {
            angleMotor.config_kF(0, motionMagicConfigs[3], TALON_TIMEOUT);
        }
        if (!Utils.epsilonEquals(this.motionMagicConfigs[4], motionMagicConfigs[4])) {
            angleMotor.configMotionSCurveStrength((int) motionMagicConfigs[4], TALON_TIMEOUT);
        }
        if (!Utils.epsilonEquals(this.motionMagicConfigs[5], motionMagicConfigs[5])) {
            angleMotor.configMotionCruiseVelocity(motionMagicConfigs[5], TALON_TIMEOUT);
        }
        if (!Utils.epsilonEquals(this.motionMagicConfigs[6], motionMagicConfigs[6])) {
            angleMotor.configMotionAcceleration(motionMagicConfigs[6], TALON_TIMEOUT);
        }
        if (!Utils.epsilonEquals(this.motionMagicConfigs[7], motionMagicConfigs[7])) {
            angleMotor.configAllowableClosedloopError(0, motionMagicConfigs[7], TALON_TIMEOUT);
        }
        if (!Utils.epsilonEquals(this.motionMagicConfigs[8], motionMagicConfigs[8])) {
            angleMotor.configMaxIntegralAccumulator(0, motionMagicConfigs[8], TALON_TIMEOUT);
        }
        if (!Utils.epsilonEquals(this.motionMagicConfigs[9], motionMagicConfigs[9])) {
            angleMotor.configClosedLoopPeakOutput(0, motionMagicConfigs[9], TALON_TIMEOUT);
        }
        this.motionMagicConfigs = motionMagicConfigs;
    }

    @Override
    public void resetAngle() {
        angleMotor.setSelectedSensorPosition(
                SwerveModuleUtils.getFalconTicksFromEncoder(encoder.get()) -
                        SwerveModuleUtils.getFalconTicksFromAngle(offsetRads));
        initializedAngleFalcon = true;
    }

    @Override
    public boolean encoderJustConnected() {
        return encoderConnectionTrigger.triggered();
    }

    @Override
    public boolean initializedAngleFalcon() {
        return initializedAngleFalcon;
    }
}
