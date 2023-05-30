package frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class Intake extends SubsystemBase {
    private final IntakeIO io;
    private final IntakeInputsAutoLogged inputs;

    private Intake(IntakeIO io) {
        inputs = new IntakeInputsAutoLogged();
        this.io = io;
        if (Robot.isReal()){

        }
    }

    public void setPowerMotor(double power){
        io.setMotorPower(power);
    }

    public double getPowerMotor(){
        return  inputs.powerMotorPower;
    }

    public void setAngleMotor(double angleRads){
        io.setAngleMotor(angleRads);
    }

    public double getAngle(){
        return inputs.angleMotorAngle;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
    }
}
