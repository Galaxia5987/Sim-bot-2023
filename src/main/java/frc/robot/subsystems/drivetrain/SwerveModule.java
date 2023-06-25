package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.math.differential.Integral;
import frc.robot.utils.units.UnitModel;
import org.littletonrobotics.junction.Logger;

public class SwerveModule extends SubsystemBase {

    private final SwerveModuleInputsAutoLogged loggerInputs = new SwerveModuleInputsAutoLogged();

    private final TalonFX driveMotor;
    private final TalonFX angleMotor;
    private final DutyCycleEncoder encoder;
    private final double[] motionMagicConfigs;

    private final int number;
    private final UnitModel ticksPerRad = new UnitModel(SwerveConstants.TICKS_PER_RADIAN);
    private final UnitModel ticksPerMeter = new UnitModel(SwerveConstants.TICKS_PER_METER);

    private Integral driveSupplyChargeUsedCoulomb = new Integral(0, 0);
    private Integral driveStatorChargeUsedCoulomb = new Integral(0, 0);

    private Integral angleSupplyChargeUsedCoulomb = new Integral(0, 0);
    private Integral angleStatorChargeUsedCoulomb = new Integral(0, 0);

    public SwerveModule(int driveMotorPort, int angleMotorPort, int encoderID,
                        double[] motionMagicConfigs, int number) {
        this.driveMotor = new TalonFX(driveMotorPort);
        this.angleMotor = new TalonFX(angleMotorPort);
        this.encoder = new DutyCycleEncoder(encoderID);
        this.motionMagicConfigs = motionMagicConfigs;
        this.number = number;

        driveMotor.configFactoryDefault(Constants.TALON_TIMEOUT);
        angleMotor.configFactoryDefault(Constants.TALON_TIMEOUT);

        driveMotor.config_kP(0, SwerveConstants.DRIVE_kP, Constants.TALON_TIMEOUT);
        driveMotor.config_kI(0, SwerveConstants.DRIVE_kI, Constants.TALON_TIMEOUT);
        driveMotor.config_kD(0, SwerveConstants.DRIVE_kD, Constants.TALON_TIMEOUT);
        driveMotor.enableVoltageCompensation(true);
        driveMotor.configVoltageCompSaturation(SwerveConstants.VOLT_COMP_SATURATION);
        driveMotor.configNeutralDeadband(SwerveConstants.NEUTRAL_DEADBAND);
        driveMotor.setNeutralMode(NeutralMode.Brake);
        driveMotor.configSupplyCurrentLimit(SwerveConstants.SUPPLY_CURRENT_LIMIT);
        driveMotor.configStatorCurrentLimit(SwerveConstants.STATOR_CURRENT_LIMIT);
        driveMotor.setInverted(SwerveConstants.CLOCKWISE);

        angleMotor.enableVoltageCompensation(true);
        angleMotor.configVoltageCompSaturation(SwerveConstants.VOLT_COMP_SATURATION);
        angleMotor.configNeutralDeadband(SwerveConstants.NEUTRAL_DEADBAND);
        angleMotor.setNeutralMode(NeutralMode.Brake);
        angleMotor.configSupplyCurrentLimit(SwerveConstants.SUPPLY_CURRENT_LIMIT);
        angleMotor.configStatorCurrentLimit(SwerveConstants.STATOR_CURRENT_LIMIT);
        angleMotor.setInverted(SwerveConstants.CLOCKWISE);
        configMotionMagic(motionMagicConfigs);
    }

    public void configMotionMagic(double[] motionMagicConfigs) {
        angleMotor.config_kP(0, motionMagicConfigs[0], Constants.TALON_TIMEOUT);
        angleMotor.config_kI(0, motionMagicConfigs[1], Constants.TALON_TIMEOUT);
        angleMotor.config_kD(0, motionMagicConfigs[2], Constants.TALON_TIMEOUT);
        angleMotor.config_kF(0, motionMagicConfigs[3], Constants.TALON_TIMEOUT);
        angleMotor.configMotionSCurveStrength((int) motionMagicConfigs[4], Constants.TALON_TIMEOUT);
        angleMotor.configMotionCruiseVelocity(motionMagicConfigs[5], Constants.TALON_TIMEOUT);
        angleMotor.configMotionAcceleration(motionMagicConfigs[6], Constants.TALON_TIMEOUT);
        angleMotor.configAllowableClosedloopError(0, motionMagicConfigs[7], Constants.TALON_TIMEOUT);
        angleMotor.configMaxIntegralAccumulator(0, motionMagicConfigs[8], Constants.TALON_TIMEOUT);
        angleMotor.configClosedLoopPeakOutput(0, motionMagicConfigs[9], Constants.TALON_TIMEOUT);
    }

    public void setModuleState(SwerveModuleState moduleState) {
        moduleState = SwerveModuleState.optimize(moduleState, new Rotation2d(loggerInputs.angle));
        setSpeed(moduleState.speedMetersPerSecond);
        setAngle(moduleState.angle.getRadians());
    }

    public SwerveModuleState getModuleState() {
        return new SwerveModuleState(
                ticksPerMeter.toUnits(driveMotor.getSelectedSensorVelocity()),
                new Rotation2d(ticksPerRad.toUnits(angleMotor.getSelectedSensorPosition()))
        );
    }

    public void setSpeed(double speed) {
        driveMotor.set(TalonFXControlMode.Velocity, ticksPerMeter.toTicks100ms(speed));
    }

    public void setAngle(double angle){
        loggerInputs.angleSetpoint = angle;
        Rotation2d error = new Rotation2d(loggerInputs.angleSetpoint).minus(new Rotation2d(loggerInputs.angle));
        angleMotor.set(TalonFXControlMode.MotionMagic, loggerInputs.angleMotorTicks + ticksPerRad.toTicks(error.getRadians()));
    }

    public double getSupplyCurrent() {
        return driveMotor.getSupplyCurrent() + angleMotor.getSupplyCurrent();
    }

    public double getStatorCurrent() {
        return driveMotor.getStatorCurrent() + angleMotor.getStatorCurrent();
    }

    public double getPosition() {
        return encoder.getAbsolutePosition();
    }

    public void updateOffset(double offset) {
        angleMotor.setSelectedSensorPosition(
                ((encoder.getAbsolutePosition() - offset) * 2048) / SwerveConstants.ANGLE_REDUCTION);
    }

    @Override
    public void periodic() {
        loggerInputs.absolutePosition = encoder.getAbsolutePosition();

        loggerInputs.driveMotorSupplyCurrent = driveMotor.getSupplyCurrent();
        loggerInputs.driveMotorStatorCurrent = driveMotor.getStatorCurrent();
        driveSupplyChargeUsedCoulomb.update(loggerInputs.driveMotorSupplyCurrent);
        loggerInputs.driveMotorSupplyCurrentOverTime = driveSupplyChargeUsedCoulomb.get();
        driveStatorChargeUsedCoulomb.update(loggerInputs.driveMotorStatorCurrent);
        loggerInputs.driveMotorStatorCurrentOverTime = driveStatorChargeUsedCoulomb.get();
        loggerInputs.driveMotorTicks = driveMotor.getSelectedSensorPosition();

        loggerInputs.angleMotorSupplyCurrent = angleMotor.getSupplyCurrent();
        loggerInputs.angleMotorStatorCurrent = angleMotor.getStatorCurrent();
        angleSupplyChargeUsedCoulomb.update(loggerInputs.angleMotorSupplyCurrent);
        loggerInputs.angleMotorSupplyCurrentOverTime = angleSupplyChargeUsedCoulomb.get();
        angleStatorChargeUsedCoulomb.update(loggerInputs.angleMotorStatorCurrent);
        loggerInputs.angleMotorStatorCurrentOverTime = angleStatorChargeUsedCoulomb.get();
        loggerInputs.angleMotorPosition = angleMotor.getSelectedSensorPosition();
        loggerInputs.angleMotorTicks = angleMotor.getSelectedSensorPosition();

        loggerInputs.angle = ticksPerRad.toUnits(angleMotor.getSelectedSensorPosition());

        Logger.getInstance().processInputs("module_" + number, loggerInputs);
    }
}
