package frc.robot.subsystems.Iitake;

import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private final SingleJointedArmSim angleMotor;
    private final FlywheelSim powerMotor;
    private Intake INSTANCE;

    public Intake getINSTANCE() {
        if (INSTANCE == null) {
            INSTANCE = new Intake();
        }
        return INSTANCE;
    }

    private Intake() {
        angleMotor = new SingleJointedArmSim();
        powerMotor = new FlywheelSim();
    }

    public void setPowerMotor(double power) {
        powerMotor.setInputVoltage(power);
    }

    public double getPowerMotor() {
        return powerMotor.getAngularVelocityRadPerSec();
    }

    //TODO: PID controller (voltage to angle)
    public void setAngleMotor(double angle) {
        angleMotor.setInputVoltage();
    }

}
