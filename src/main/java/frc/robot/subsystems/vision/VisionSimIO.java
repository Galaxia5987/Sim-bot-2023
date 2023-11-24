package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import org.photonvision.SimPhotonCamera;
import org.photonvision.SimVisionSystem;

public class VisionSimIO implements VisionIO {
    private SimVisionSystem simVisionSystem;
    private SimPhotonCamera simCamera;
    private NetworkTableEntry tableEntry = new NetworkTableEntry()

    public VisionSimIO(Transform3d robotToCam) {
        simVisionSystem = new SimVisionSystem("simCam", 95, robotToCam, 1000, 1600, 1200, 0);
        simCamera = new SimPhotonCamera(NetworkTableInstance.getDefault(), "simCam");
    }


    @Override
    public void setPipeLine(int pipeLineIndex) {

    }

    @Override
    public Result getLatestResult() {

    }

    @Override
    public void updateInputs(VisionInputs inputs) {

    }

}
