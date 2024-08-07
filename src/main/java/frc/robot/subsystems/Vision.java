package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase{
    
    private static final Limelight limelight = new Limelight();
    private static final PhotonVision photonVision = new PhotonVision();

    public Vision() {
    }

    @Override
    public void periodic() {
        limelight.periodic();
        photonVision.periodic();
    }
}
