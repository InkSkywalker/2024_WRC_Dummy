package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.generated.TunerConstants;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements
 * subsystem so it can be used in command-based projects easily.
 */
public class Swerve extends SwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    public boolean take_control_yaw = false;
    public double yaw_setpoint = 0.;
    public double yaw_controller_output = 0;
    public PIDController m_yawController = new PIDController(0.2, 0, 0.005);

    public boolean take_control_xy = false;
    public double x_setpoint = 0.;
    public double y_setpoint = 0.;
    public PIDController m_xController = new PIDController(0.1, 0, 0);
    public PIDController m_yController = new PIDController(0.1, 0, 0);
    public double x_controller_output = 0;
    public double y_controller_output = 0;

    public static double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
    public static double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private final NetworkTable table = inst.getTable("Swerve");
    private final DoublePublisher yawtargetpub = table.getDoubleTopic("yaw_target").publish();
    private final DoublePublisher yawnowpub = table.getDoubleTopic("yaw_now").publish();
    private final DoublePublisher yawerrorpub = table.getDoubleTopic("yaw_error").publish();
    private final DoublePublisher yawoutput = table.getDoubleTopic("yaw_output").publish();

    public static final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.Velocity) // I want field-centric
                                                             // driving in open loop
            .withSteerRequestType(SteerRequestType.MotionMagic);
    public static final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    public static final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private final Rotation2d BlueAlliancePerspectiveRotation = Rotation2d.fromDegrees(0);
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private final Rotation2d RedAlliancePerspectiveRotation = Rotation2d.fromDegrees(180);
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean hasAppliedOperatorPerspective = false;

    public Swerve(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency,
            SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
    }

    public Swerve(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    public Command cmd_drive(Supplier<Double> x, Supplier<Double> y, Supplier<Double> rot) {
        Supplier<Double> rot_output;
        Supplier<Double> x_output;
        Supplier<Double> y_output;
        rot_output = () -> {
            if (take_control_yaw) {
                if (yaw_controller_output > MaxAngularRate) {
                    return MaxAngularRate;
                } else if (yaw_controller_output < -MaxAngularRate) {
                    return -MaxAngularRate;
                }
                return yaw_controller_output;
            } else {
                if (rot.get() > MaxAngularRate) {
                    return MaxAngularRate;
                } else if (rot.get() < -MaxAngularRate) {
                    return -MaxAngularRate;
                }
                return rot.get();
            }
        };
        x_output = () -> {
            if (take_control_xy) {
                if (x_controller_output > MaxSpeed) {
                    return MaxSpeed;
                } else if (x_controller_output < -MaxSpeed) {
                    return -MaxSpeed;
                }
                return x_controller_output;
            } else {
                if (x.get() > MaxSpeed) {
                    return MaxSpeed;
                } else if (x.get() < -MaxSpeed) {
                    return -MaxSpeed;
                }
                return x.get();
            }
        };
        y_output = () -> {
            if (take_control_xy) {
                if (y_controller_output > MaxSpeed) {
                    return MaxSpeed;
                } else if (y_controller_output < -MaxSpeed) {
                    return -MaxSpeed;
                }
                return y_controller_output;
            } else {
                if (y.get() > MaxSpeed) {
                    return MaxSpeed;
                } else if (y.get() < -MaxSpeed) {
                    return -MaxSpeed;
                }
                return y.get();
            }
        };
        Supplier<Double> rot_deadband = () -> {
            if (take_control_yaw) {
                return 0.;
            } else {
                return 0.1 * MaxAngularRate;
            }
        };
        return applyRequest(() -> drive.withVelocityX(x_output.get()).withVelocityY(y_output.get())
                .withRotationalRate(rot_output.get()).withRotationalDeadband(rot_deadband.get()));
    }

    public Command cmd_brake_x() {
        return applyRequest(() -> brake);
    }

    public Command cmd_pointwheels(Supplier<Double> x, Supplier<Double> y) {
        return applyRequest(() -> point.withModuleDirection(new Rotation2d(x.get(), y.get())));
    }

    public void setHead() {
        DriverStation.getAlliance().ifPresent((allianceColor) -> {
            if (allianceColor == Alliance.Red) {
                this.getPigeon2().setYaw(RedAlliancePerspectiveRotation.getDegrees());
                this.setOperatorPerspectiveForward(RedAlliancePerspectiveRotation);
            } else {
                this.getPigeon2().setYaw(BlueAlliancePerspectiveRotation.getDegrees());
                this.setOperatorPerspectiveForward(BlueAlliancePerspectiveRotation);
            }
        });
        this.seedFieldRelative();
    }

    @Override
    public void periodic() {
        /* Periodically try to apply the operator perspective */
        /*
         * If we haven't applied the operator perspective before, then we should apply
         * it regardless of DS state
         */
        /*
         * This allows us to correct the perspective in case the robot code restarts
         * mid-match
         */
        /*
         * Otherwise, only check and apply the operator perspective if the DS is
         * disabled
         */
        /*
         * This ensures driving behavior doesn't change until an explicit disable event
         * occurs during testing
         */
        if (!hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent((allianceColor) -> {
                this.setOperatorPerspectiveForward(
                        allianceColor == Alliance.Red ? RedAlliancePerspectiveRotation
                                : BlueAlliancePerspectiveRotation);
                hasAppliedOperatorPerspective = true;
            });
        }

        double yaw_error = -this.yaw_setpoint + this.getState().Pose.getRotation().getDegrees();
        if (yaw_error > 180) {
            yaw_error -= 360;
        }
        if (yaw_error < -180) {
            yaw_error += 360;
        }
        this.yaw_controller_output = m_yawController.calculate(yaw_error, 0);

        this.x_controller_output = m_xController.calculate(this.getState().Pose.getTranslation().getX(),
                this.x_setpoint);
        this.y_controller_output = m_yController.calculate(this.getState().Pose.getTranslation().getY(),
                this.y_setpoint);

        yawtargetpub.set(this.yaw_setpoint);
        yawnowpub.set(this.getState().Pose.getRotation().getDegrees());
        yawerrorpub.set(yaw_error);
        yawoutput.set(this.yaw_controller_output);
    }
}
