package frc.robot.subsystems.intake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import frc.robot.utils.units.UnitModel;

public class IntakeIOReal implements IntakeIO{
    private final TalonFX angleMotor;
    private final CANSparkMax spinMotor;

    private final UnitModel unitModel = new UnitModel(IntakeConstants.ANGLE_MOTOR_TICKS_PER_RADIAN);

    public IntakeIOReal(int angleMotorPort, int spinMotorPort){
        angleMotor = new TalonFX(angleMotorPort);
        spinMotor = new CANSparkMax(spinMotorPort, CANSparkMaxLowLevel.MotorType.kBrushless);

        angleMotor.enableVoltageCompensation(true);
        angleMotor.configVoltageCompSaturation(IntakeConstants.VOLT_COMPENSATION);
        spinMotor.enableVoltageCompensation(IntakeConstants.VOLT_COMPENSATION);
    }

    @Override
    public void updateInputs(IntakeInputs inputs) {
        inputs.angleMotorVelocity = angleMotor.getActiveTrajectoryVelocity();
//        inputs.spinMotorVelocity = spinMotor.; //TODO: check how to get the velocity of the NEO
        inputs.angle = unitModel.toUnits(angleMotor.getSelectedSensorPosition());
    }

    @Override
    public void setAngleMotorPower(double power) {
        angleMotor.set(ControlMode.PercentOutput, power);
    }

    @Override
    public void setSpinMotorPower(double power) {
        spinMotor.set(power);
    }

    @Override
    public void setAngle(double angle) {
        angleMotor.set(TalonFXControlMode.Position, unitModel.toTicks(angle));
    }
}
