package frc.robot.subsystems.vision;

import org.photonvision.PhotonCamera;

public class VisionModule {

    public final String name;
    public final VisionIO io;
    public final VisionInputsLogged inputs;

    public VisionModule(String name, VisionIO io, VisionInputsLogged inputs) {
        this.name = name;
        this.io = io;
        this.inputs = inputs;
    }


}
