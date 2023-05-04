package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
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

    private double setpointAngle = 0;

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
        inputs.setpointAngleRads = setpointAngle;
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
        setpointAngle = angleRads;
        double ticks = SwerveModuleUtils.getFalconTicksFromAngle(angleRads);
        angleMotor.set(TalonFXControlMode.MotionMagic, ticks);
    }

    @Override
    public void setDriveVoltage(double voltage) {
        driveMotor.set(TalonFXControlMode.PercentOutput, voltage / 12.0);
    }

    @Override
    public void configMotionMagic(double[] motionMagicConfigs) {
        angleMotor.config_kP(0, motionMagicConfigs[0], TALON_TIMEOUT);
        angleMotor.config_kI(0, motionMagicConfigs[1], TALON_TIMEOUT);
        angleMotor.config_kD(0, motionMagicConfigs[2], TALON_TIMEOUT);
        angleMotor.config_kF(0, motionMagicConfigs[3], TALON_TIMEOUT);
        angleMotor.configMotionSCurveStrength((int) motionMagicConfigs[4], TALON_TIMEOUT);
        angleMotor.configMotionCruiseVelocity(motionMagicConfigs[5], TALON_TIMEOUT);
        angleMotor.configMotionAcceleration(motionMagicConfigs[6], TALON_TIMEOUT);
        angleMotor.configAllowableClosedloopError(0, motionMagicConfigs[7], TALON_TIMEOUT);
        angleMotor.configMaxIntegralAccumulator(0, motionMagicConfigs[8], TALON_TIMEOUT);
        angleMotor.configClosedLoopPeakOutput(0, motionMagicConfigs[9], TALON_TIMEOUT);
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
