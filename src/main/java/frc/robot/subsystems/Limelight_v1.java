package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.generated.TunerConstants;

public class Limelight_v1 extends SubsystemBase {

    Swerve m_swerve = TunerConstants.DriveTrain;
    Pigeon2 m_gyro = new Pigeon2(20, "canivore");

    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private final NetworkTable table = inst.getTable("Pose");
    private final DoubleArrayPublisher limelightPub = table.getDoubleArrayTopic("limelight").publish();

    private Pose2d m_lastPose = new Pose2d();

    public Limelight_v1() {
    }

    @Override
    public void periodic() {
        boolean doRejectUpdate = false;
        LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
        if (mt1.tagCount == 1 && mt1.rawFiducials.length == 1) {
            if (mt1.rawFiducials[0].ambiguity > .7) {
                doRejectUpdate = true;
            }
            if (mt1.rawFiducials[0].distToCamera > 3) {
                doRejectUpdate = true;
            }
        }
        if (mt1.tagCount == 0) {
            doRejectUpdate = true;
        }

        if (!doRejectUpdate) {
            m_swerve.setVisionMeasurementStdDevs(VecBuilder.fill(100, 100, 999));
            m_swerve.addVisionMeasurement(
                    mt1.pose,
                    mt1.timestampSeconds);
            this.m_lastPose = mt1.pose;
        }

        limelightPub.set(new double[] {
                mt1.pose.getX(),
                mt1.pose.getY(),
                mt1.pose.getRotation().getDegrees()
        });

    }

    public Pose2d getLimelightPose() {
        return m_lastPose;
    }
}
