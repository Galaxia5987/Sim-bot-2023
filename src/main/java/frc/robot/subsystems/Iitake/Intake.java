package frc.robot.subsystems.Iitake;

import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
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

    @Override
    public void periodic() {
        io.updateInputs(inputs);
    }
}
