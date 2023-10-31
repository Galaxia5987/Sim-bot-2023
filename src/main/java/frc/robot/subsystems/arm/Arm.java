package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
    private ArmIO io;
    private ArmInputsAutoLogged inputs;
    static Arm INSTANCE;

public Arm getINSTANCE(){
    if(INSTANCE == null){
        INSTANCE = new Arm(io);
    }

        return INSTANCE;

}

private void setElbowPower(double power){
    inputs.
}




    private Arm(ArmIO io){
        this.io = io;
    }

    @Override
    public void periodic() {
io.setElbowAngle();
io.setElbowPower();
io.setShoulderAngle();
io.setShoulderPower();
io.updateInputs();
    }
}
