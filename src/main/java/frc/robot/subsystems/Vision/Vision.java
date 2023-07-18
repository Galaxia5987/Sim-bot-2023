package frc.robot.subsystems.Vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import org.photonvision.PhotonCamera;

public class Vision extends SubsystemBase {
    private final VisionIO.VisionInputs[] visionInputs;
    private final VisionIO[] io_s;
    private final Pose3d[] estimatedPoses;
    private static Vision INSTANCE;

    public static Vision getINSTANCE(){
        if(INSTANCE == null) {
            if (Robot.isReal()) {
                INSTANCE = new Vision(
                        new IOPhotonVision(new PhotonCamera("camera_1")),
                        new IOPhotonVision(new PhotonCamera("camera_2"))
                );

            }
            else{
                INSTANCE = new Vision(
                        new IOPhotonVision(new PhotonCamera("SimCamera_1")),
                        new IOPhotonVision(new PhotonCamera("SimCamera_2"))
                );
            }
        }
        return INSTANCE;
    }

    private Vision(VisionIO... io_s) {
        this.visionInputs = new VisionIO.VisionInputs[io_s.length];
        this.io_s = io_s;
        this.estimatedPoses = new Pose3d[this.visionInputs.length];
        for(int i = 0; i < this.io_s.length; i++){
           this.estimatedPoses[i] = new Pose3d(new Translation3d(visionInputs[i].poseFieldOriented[0], visionInputs[i].poseFieldOriented[1], visionInputs[i].poseFieldOriented[2]),
                   new Rotation3d(new Quaternion(visionInputs[i].poseFieldOriented[3], visionInputs[i].poseFieldOriented[4], visionInputs[i].poseFieldOriented[5],visionInputs[i].poseFieldOriented[6])));
        }
    }

    public void setPipeLine(int... pipeLine) {
        for (int i = 0; i < this.io_s.length; i++) {
            io_s[i].setPipeLine(pipeLine[i]);
        }
    }

    @Override
    public void periodic() {
        for (int i = 0; i < this.visionInputs.length; i++) {
            this.io_s[i].updateInputs(this.visionInputs[i]);
        }
    }
}
