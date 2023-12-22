package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.*;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.VisionTargetSim;

public class VisionConstants {
    public static final Transform3d[] ROBOT_TO_CAM = new Transform3d[]{
            new Transform3d( //left cam
                    new Translation3d(-0.249, 0.30397, 0.5023),
                    new Rotation3d(Math.toRadians(0), Math.toRadians(0), Math.toRadians(142.5))),
            new Transform3d( //right cam
                    new Translation3d(-0.2363, -0.1, 0.196),
                    new Rotation3d(Math.toRadians(0)+0.16, Math.toRadians(-28)-0.085, Math.toRadians(-151))),
            new Transform3d( //limelight
                    new Translation3d(-0.2263, 0.286, 0.76),
                    new Rotation3d(Math.toRadians(0), Math.toRadians(0), Math.toRadians(16.62)))
    };

//     new Transform3d( //right cam
//                    new Translation3d(-0.2363, -0.527, 0.196),
//                    new Rotation3d(Math.toRadians(0)+0.16, Math.toRadians(-28)-0.085, Math.toRadians(-151)))};

    public static final Transform3d LIME_OFFSET = new Transform3d(new Translation3d( (16.54) / 2,(8.02) / 2, 0), new Rotation3d());
    public static final double TARGET_WIDTH = 0.27;
    public static final double TARGET_LENGTH = 0.27;
    public static final Pose3d[] TARGET_POSITION_REAL = new Pose3d[]{new Pose3d() ,new Pose3d(new Translation3d( 7.24310, -2.93659, 0.46272), new Rotation3d(0,0,180)),new Pose3d(new Translation3d(7.24310, 1.26019, 0.46272), new Rotation3d(0,0,180)), new Pose3d(new Translation3d(7.24310, 0.41621, 0.41621), new Rotation3d(0,0,180)), new Pose3d(new Translation3d(7.90832, 2.74161, 0.695452), new Rotation3d(0,0,180)), new Pose3d(new Translation3d(-7.90832, 2.74161, 0.695452), new Rotation3d()), new Pose3d(new Translation3d(-7.24310, 0.41621, 0.46272), new Rotation3d()), new Pose3d(new Translation3d(-7.24310, 1.26019, 0.46272), new Rotation3d()), new Pose3d(new Translation3d(-7.24310, -2.93659, 0.46272), new Rotation3d())};
    public static final Pose3d[] TARGET_POSITION_LIME = new Pose3d[]{new Pose3d() ,new Pose3d(new Translation3d( 7.24310, -2.93659, 0.46272), new Rotation3d(0,0,180)),new Pose3d(new Translation3d(7.24310, 1.26019, 0.46272), new Rotation3d(0,0,180)), new Pose3d(new Translation3d(7.24310, 0.41621, 0.41621), new Rotation3d(0,0,180)), new Pose3d(new Translation3d(7.90832, 2.74161, 0.695452), new Rotation3d(0,0,180)), new Pose3d(new Translation3d(-7.90832, 2.74161, 0.695452), new Rotation3d()), new Pose3d(new Translation3d(-7.24310, 0.41621, 0.46272), new Rotation3d()), new Pose3d(new Translation3d(-7.24310, 1.26019, 0.46272), new Rotation3d()), new Pose3d(new Translation3d(-7.24310, -2.93659, 0.46272), new Rotation3d())};
    public static final Pose3d[] TARGET_POSITION_SIM = new Pose3d[]{new Pose3d() ,new Pose3d(new Translation3d( 7.24310, -2.93659, 0.46272), new Rotation3d(0,0,180)),new Pose3d(new Translation3d(7.24310, -1.26019, 0.46272), new Rotation3d(0,0,180)), new Pose3d(new Translation3d(7.24310, 0.41621, 0.46272), new Rotation3d(0,0,180)), new Pose3d(new Translation3d(7.90832, 2.74161, 0.695452), new Rotation3d(0,0,180)), new Pose3d(new Translation3d(-7.90832, 2.74161, 0.695452), new Rotation3d()), new Pose3d(new Translation3d(-7.24310, 0.41621, 0.46272), new Rotation3d()), new Pose3d(new Translation3d(-7.24310, 1.26019, 0.46272), new Rotation3d()), new Pose3d(new Translation3d(-7.24310, -2.93659, 0.46272), new Rotation3d())};
    public static final VisionTargetSim[] SIM_VISION_TARGETS = new VisionTargetSim[]{new VisionTargetSim(VisionConstants.TARGET_POSITION_SIM[0], new TargetModel(VisionConstants.TARGET_LENGTH,VisionConstants.TARGET_WIDTH,VisionConstants.TARGET_POSITION_SIM[0].getZ())),new VisionTargetSim(TARGET_POSITION_SIM[1], new TargetModel(TARGET_LENGTH,TARGET_WIDTH,TARGET_POSITION_SIM[1].getZ()),1), new VisionTargetSim(TARGET_POSITION_SIM[2], new TargetModel(TARGET_LENGTH,TARGET_WIDTH,TARGET_POSITION_SIM[2].getZ()),2), new VisionTargetSim(TARGET_POSITION_SIM[3], new TargetModel(TARGET_LENGTH,TARGET_WIDTH,TARGET_POSITION_SIM[3].getZ()),3), new VisionTargetSim(TARGET_POSITION_SIM[4],new TargetModel(TARGET_LENGTH,TARGET_WIDTH,TARGET_POSITION_SIM[4].getZ()),4), new VisionTargetSim(TARGET_POSITION_SIM[5], new TargetModel(TARGET_LENGTH,TARGET_WIDTH,TARGET_POSITION_SIM[5].getZ()),5), new VisionTargetSim(TARGET_POSITION_SIM[6], new TargetModel(TARGET_LENGTH,TARGET_WIDTH,TARGET_POSITION_SIM[6].getZ()),6), new VisionTargetSim(TARGET_POSITION_SIM[7], new TargetModel(TARGET_LENGTH,TARGET_WIDTH,TARGET_POSITION_SIM[7].getZ()),7 ), new VisionTargetSim(TARGET_POSITION_SIM[8],new TargetModel(TARGET_LENGTH,TARGET_WIDTH,TARGET_POSITION_SIM[8].getZ()),8), new VisionTargetSim(new Pose3d(), new TargetModel(0,0,0))};

}
