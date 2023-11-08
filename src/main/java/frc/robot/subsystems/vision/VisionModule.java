package frc.robot.subsystems.vision;

public class VisionModule {

    public final String name;
    public final VisionIO io;
    public final VisionInputsAutoLogged inputs;

    public VisionModule(String name, VisionIO io, VisionInputsAutoLogged inputs) {
        this.name = name;
        this.io = io;
        this.inputs = inputs;
    }
}
