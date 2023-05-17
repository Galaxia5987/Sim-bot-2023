package frc.robot.subsystems.intake;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

public class IntakeIOReal implements IntakeIO{
    private final TalonFX angleMotor;
    private final CANSparkMax spinMotor;

    public IntakeIOReal(int angleMotorPort, int spinMotorPort){
        angleMotor = new TalonFX(angleMotorPort);
        spinMotor = new CANSparkMax(spinMotorPort, CANSparkMaxLowLevel.MotorType.kBrushed);

    }
}
