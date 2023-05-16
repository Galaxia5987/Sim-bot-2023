package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

    public void setAngleMotorVelocity(double velocity){
        intakeIO.setAngleMotorVelocity(velocity);
    }

    public void setAngle(double angle){
        intakeIO.setAngle(angle);
    }

    public void setSpinMotorVelocity(double velocity){
        intakeIO.setSpinMotorVelocity(velocity);
    }
}
