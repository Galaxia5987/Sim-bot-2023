package frc.robot.subsystems.intake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import frc.robot.Constants;
import frc.robot.Ports;
import utils.units.UnitModel;

public class IntakeIOReal implements IntakeIO {
    private final CANSparkMax spinMotor = new CANSparkMax(Ports.Intake.INTAKE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final TalonFX angleMotor = new TalonFX(Ports.Intake.ANGLE_MOTOR);
    private final UnitModel unitModel = new UnitModel(IntakeConstants.TICKS_PER_DEGREE);

    public IntakeIOReal() {
        angleMotor.configFactoryDefault();
        spinMotor.restoreFactoryDefaults();

        spinMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
        spinMotor.enableVoltageCompensation(Constants.NOMINAL_VOLTAGE);
        spinMotor.setInverted(Ports.Intake.POWER_INVERTED);
        for (int i = 1; i <= 6; i++) {
            spinMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.fromId(i), 50000);
        }
        spinMotor.burnFlash();

        angleMotor.setNeutralMode(NeutralMode.Brake);
        angleMotor.enableVoltageCompensation(true);
        angleMotor.configVoltageCompSaturation(Constants.NOMINAL_VOLTAGE);
        angleMotor.setInverted(Ports.Intake.ANGLE_INVERTED);
        angleMotor.config_kP(0, IntakeConstants.kP);
        angleMotor.config_kI(0, IntakeConstants.kI);
        angleMotor.config_kD(0, IntakeConstants.kD);
        angleMotor.config_kF(0, IntakeConstants.kF);
        angleMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(
                true, 30, 0, 0
        ));
        angleMotor.configClosedLoopPeakOutput(0, 0.4);
    }

    @Override
    public void setSpinMotorPower(double power) {
        spinMotor.set(power);
    }

    @Override
    public void setAngleMotorAngle(double angle) {
        angleMotor.set(ControlMode.Position, angle);
    }

    @Override
    public void setAngleMotorPower(double power) {
        angleMotor.set(ControlMode.PercentOutput, power);
    }

    @Override
    public void resetEncoder(double angle) {
        angleMotor.setSelectedSensorPosition(
                unitModel.toTicks(angle)
        );
    }

    @Override
    public void updateInputs(IntakeLoggedInputs inputs) {
        inputs.spinMotorPower = spinMotor.getAppliedOutput();
        inputs.angleMotorAngle = unitModel.toUnits(angleMotor.getSelectedSensorPosition());
        inputs.angleMotorVelocity = unitModel.toVelocity(angleMotor.getSelectedSensorVelocity());
        inputs.angleMotorcurrent = angleMotor.getSupplyCurrent();
        inputs.angleMotorPower = angleMotor.getMotorOutputPercent();
    }
}
