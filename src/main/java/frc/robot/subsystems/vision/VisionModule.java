package frc.robot.subsystems.vision;

import org.photonvision.PhotonCamera;

public class VisionModule {

    public final String name;
    public final VisionIO io;
    public final VisionInputsAutoLogged inputs;

    public VisionModule(String name, VisionIO io, VisionInputsAutoLogged inputs) {
        this.name = name;
        this.io = io;
        this.inputs = inputs;
    }

    public static VisionModule photonVisionIO(String name, int index) {
        return new VisionModule(
                name,
                new PhotonVisionIO(new PhotonCamera(name), VisionConstants.ROBOT_TO_CAM[index]),
                new VisionInputsAutoLogged()
        );
    }
}
