package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.generated.TunerConstants;

public class Limelight extends SubsystemBase  {

    Swerve m_swerve = TunerConstants.DriveTrain;
    Pigeon2 m_gyro = new Pigeon2(20, "canivore");

    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private final NetworkTable table = inst.getTable("Pose");
    private final DoubleArrayPublisher limelightPub = table.getDoubleArrayTopic("limelight").publish();

    public Limelight() {
    }

    @Override
    public void periodic() {
        boolean doRejectUpdate = false;
        LimelightHelpers.SetRobotOrientation("limelight", m_swerve.getState().Pose.getRotation().getDegrees(), 0, 0, 0,
                0, 0);
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
        if (Math.abs(m_gyro.getRate()) > 720) // if our angular velocity is greater than 720 degrees per second, ignore
                                              // vision updates
        {
            doRejectUpdate = true;
        }
        if (mt2.tagCount == 0) {
            doRejectUpdate = true;
        }
        if (!doRejectUpdate) {
            m_swerve.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));
            m_swerve.addVisionMeasurement(
                    mt2.pose,
                    mt2.timestampSeconds);
        }

        limelightPub.set(new double[] {
                mt2.pose.getX(),
                mt2.pose.getY(),
                mt2.pose.getRotation().getDegrees()
        });

    }

    public static Pose2d getLimelightPose() {
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
        return mt2.pose;
    }
}
