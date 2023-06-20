package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.math.differential.Integral;

public class SwerveModule extends SubsystemBase {

    private final SwerveModuleInputsAutoLogged loggerInputs = new SwerveModuleInputsAutoLogged();

    private final TalonFX driveMotor;
    private final TalonFX angleMotor;
    private final DutyCycleEncoder encoder;

    private final int number;

    private Integral driveSupplyChargeUsedCoulomb = new Integral(0, 0);
    private Integral driveStatorChargeUsedCoulomb = new Integral(0, 0);

    private Integral angleSupplyChargeUsedCoulomb = new Integral(0, 0);
    private Integral angleStatorChargeUsedCoulomb = new Integral(0, 0);

    public SwerveModule(int driveMotorPort, int angleMotorPort, int encoderID, int number) {
        this.driveMotor = new TalonFX(driveMotorPort);
        this.angleMotor = new TalonFX(angleMotorPort);
        this.encoder = new DutyCycleEncoder(encoderID);
        this.number = number;

        driveMotor.configFactoryDefault(Constants.TALON_TIMEOUT);
        angleMotor.configFactoryDefault(Constants.TALON_TIMEOUT);

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
    }

    public void setModuleState(SwerveModuleState moduleState){
        setSpeed(moduleState.speedMetersPerSecond);
        setAngle(moduleState.angle.getRadians());
    }

    public void setSpeed(double speed){
        driveMotor.set(TalonFXControlMode.Velocity, speed);
    }

    public void setAngle(double angle){
        angleMotor.set(TalonFXControlMode.Position, angle);
    }

    @Override
    public void periodic() {
        loggerInputs.driveMotorSupplyCurrent = driveMotor.getSupplyCurrent();
        loggerInputs.driveMotorStatorCurrent = driveMotor.getStatorCurrent();
        driveSupplyChargeUsedCoulomb.update(loggerInputs.driveMotorSupplyCurrent);
        loggerInputs.driveMotorSupplyCurrentOverTime = driveSupplyChargeUsedCoulomb.get();
        driveStatorChargeUsedCoulomb.update(loggerInputs.driveMotorStatorCurrent);
        loggerInputs.driveMotorStatorCurrentOverTime = driveStatorChargeUsedCoulomb.get();

        loggerInputs.angleMotorSupplyCurrent = angleMotor.getSupplyCurrent();
        loggerInputs.angleMotorStatorCurrent = angleMotor.getStatorCurrent();
        angleSupplyChargeUsedCoulomb.update(loggerInputs.angleMotorSupplyCurrent);
        loggerInputs.angleMotorSupplyCurrentOverTime = angleSupplyChargeUsedCoulomb.get();
        angleStatorChargeUsedCoulomb.update(loggerInputs.angleMotorStatorCurrent);
        loggerInputs.angleMotorStatorCurrentOverTime = angleStatorChargeUsedCoulomb.get();
    }
}
