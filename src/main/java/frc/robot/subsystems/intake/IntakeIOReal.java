package frc.robot.subsystems.intake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.Ports;
import utils.Utils;
import utils.units.UnitModel;


public class IntakeIOReal implements IntakeIO {
    private final CANSparkMax spinMotor = new CANSparkMax(Ports.Intake.INTAKE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);

    private final TalonFX angleMotorPhx = new TalonFX(0);
    private final TalonFXConfigurator angleConfigurator;
    private final TalonFXConfiguration angleConfiguration = new TalonFXConfiguration();

    private final UnitModel unitModel = new UnitModel(IntakeConstants.TICKS_PER_DEGREE);

    private double currentAngle;

    public IntakeIOReal() {
        angleConfigurator = angleMotorPhx.getConfigurator();
        angleConfiguration.Slot0.kP = IntakeConstants.kP;
        angleConfiguration.Slot0.kI = IntakeConstants.kI;
        angleConfiguration.Slot0.kD = IntakeConstants.kD;
        angleConfigurator.apply(angleConfiguration.Slot0);

        spinMotor.restoreFactoryDefaults();
        spinMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
        spinMotor.enableVoltageCompensation(Constants.NOMINAL_VOLTAGE);
        spinMotor.setInverted(Ports.Intake.POWER_INVERTED);
        for (int i = 1; i <= 6; i++) {
            spinMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.fromId(i), 50000);
        }

        spinMotor.burnFlash();
    }

    @Override
    public void setSpinMotorPower(double power) {
        spinMotor.set(power);
    }

    @Override
    public void setAngleMotorAngle(double angle) {
        var motorRequest = new PositionVoltage(Units.radiansToRotations(angle));
        angleMotorPhx.setControl(motorRequest);
    }

    @Override
    public void setAngleMotorPower(double power) {
        angleMotorPhx.setVoltage(power*12);
    }

    @Override
    public void resetEncoder(double angle) {}

    @Override
    public void updateInputs(IntakeLoggedInputs inputs) {
        angleConfigurator.apply(angleConfiguration.Slot0);
        inputs.spinMotorPower = spinMotor.getAppliedOutput();
        //inputs.setpointSpinMotorPower
        //inputs.setpointAngleMotorPower
        //inputs.setpointAngleMotorAngle
        inputs.angleMotorPower = angleMotorPhx.getMotorVoltage().getValue()/12;
        inputs.angleMotorAngle = angleMotorPhx.getRotorPosition().getValue();
        currentAngle = inputs.angleMotorAngle;
        inputs.angleMotorcurrent = angleMotorPhx.getSupplyCurrent().getValue();
        inputs.spinMotorCurrent = spinMotor.getOutputCurrent();
        inputs.angleMotorVelocity = angleMotorPhx.getRotorVelocity().getValue();
    }
}
