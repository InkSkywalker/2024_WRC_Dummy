// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.SteerRequestType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Shooter;
import frc.robot.commands.AimShoot;
import frc.robot.commands.DoAmp;

public class RobotContainer {

    public static boolean isRedAlliance = false;

    /* Setting up bindings for necessary control of the swerve drive platform */
    public static final CommandPS5Controller joystick = new CommandPS5Controller(0); // My joystick
    public static final Swerve drivetrain = TunerConstants.DriveTrain; // My drivetrain
    public static final Intake intake = new Intake(); // My intake
    public static final Arm arm = new Arm();
    public static final Shooter shooter = new Shooter();
    public static final Limelight limelight = new Limelight();
    // public static final PhotonVision photonVision = new PhotonVision();

    private Command AimShoot = new AimShoot();
    private Command DoAmp = new DoAmp();

    private final Telemetry logger = new Telemetry(drivetrain.MaxSpeed);

    private void configureBindings() {
        drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() -> Swerve.drive
                        .withVelocityX(-joystick.getLeftY() * Swerve.MaxSpeed * (isRedAlliance ? -1 : 1))
                        // Drive forward with negative Y (forward)
                        .withVelocityY(-joystick.getLeftX() * Swerve.MaxSpeed * (isRedAlliance ? -1 : 1))
                        // Drive left with negative X (left)
                        .withRotationalRate(-joystick.getRightX() * Swerve.MaxAngularRate)
                // Drive counterclockwise with negative X (left)
                ));

        // joystick.cross().whileTrue(drivetrain.applyRequest(() -> brake));
        // joystick.circle().whileTrue(drivetrain
        // .applyRequest(() -> point.withModuleDirection(new
        // Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

        // reset the field-centric heading on left bumper press
        joystick.L3().onTrue(drivetrain.runOnce(() -> {
            drivetrain.seedFieldRelative();
        }));

        // Intake in
        joystick.L2().onTrue(Commands.runOnce(() -> intake.eat_in()));
        joystick.L2().onFalse(Commands.runOnce(() -> intake.stop()));
        // Intake out
        joystick.L1().onTrue(Commands.runOnce(() -> intake.eat_out()));
        joystick.L1().onFalse(Commands.runOnce(() -> intake.stop()));

        // Arm Up, Arm Down
        joystick.povUp().onTrue(Commands.runOnce(() -> arm.arm_up_volt(false)));
        joystick.povUp().onFalse(Commands.runOnce(() -> arm.stop()));
        joystick.povDown().onTrue(Commands.runOnce(() -> arm.arm_down_volt(false)));
        joystick.povDown().onFalse(Commands.runOnce(() -> arm.stop()));

        joystick.R2().onTrue(Commands.runOnce(() -> {
            arm.arm_up();
            drivetrain.take_control_yaw = true;
        }));
        joystick.R2().onFalse(Commands.runOnce(() -> {
            arm.arm_down();
            drivetrain.take_control_yaw = false;
        }));

        // joystick.L1().onTrue(Commands.runOnce(() -> shooter.shoot_out()));
        // joystick.L1().onFalse(Commands.runOnce(() -> shooter.shoot_break()));

        joystick.R1().whileTrue(AimShoot);
        // joystick.triangle().whileTrue(Commands.runOnce(() -> shooter.shoot_amp()));
        // joystick.triangle().onFalse(Commands.runOnce(() -> shooter.shoot_break()));
        // joystick.circle().whileTrue(Commands.runOnce(() -> arm.arm_amp()));
        // joystick.cross().onTrue(Commands.runOnce(() -> intake.reverse_once()));
        joystick.triangle().whileTrue(DoAmp);

        if (Utils.isSimulation()) {
            drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
        }
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public RobotContainer() {
        configureBindings();
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
