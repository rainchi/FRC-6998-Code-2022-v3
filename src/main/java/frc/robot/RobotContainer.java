// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.MecanumDriveKinematicsConstraint;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.DelayCommand;
import frc.robot.commands.RunTrajectoryCommand;
import frc.robot.commands.WaitShootSpeedCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HangSubsystem;
import frc.robot.subsystems.CollectSubsystem;
import frc.robot.subsystems.ShootSubsystem;

import java.util.ArrayList;
import java.util.List;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // Game Controllers
    private final XboxController controller1 = new XboxController(0);
    private final XboxController controller2 = new XboxController(1);

    // Subsystems
    private final CollectSubsystem intake = new CollectSubsystem();
    private final DriveSubsystem drive = new DriveSubsystem(controller1, intake);
    private final HangSubsystem hang = new HangSubsystem();
    private final ShootSubsystem shoot = new ShootSubsystem(drive);

    private final SendableChooser<String> pathChooser = new SendableChooser<>();
    private Command autoCommand;

    private final AddressableLED hangLightStrip = new AddressableLED(9);
    private final AddressableLEDBuffer hangLightStripBuffer = new AddressableLEDBuffer(148);

    private final NetworkTableInstance nt = NetworkTableInstance.getDefault();

    public static final String AUTO_1 = "Auto 1";
    public static final String AUTO_2 = "Auto 2";
    public static final String AUTO_3 = "Auto 3";

    private DriverStation.Alliance alliance;

    private boolean teleop = false;

    private int m_rainbowFirstPixelHue = 0;

    private boolean enableColorLoop = false;


    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();

        hangLightStrip.setLength(hangLightStripBuffer.getLength());
        hangLightStrip.start();

        // Add path options and send to the dashboard
        pathChooser.setDefaultOption("Auto 1", AUTO_1);
        pathChooser.addOption("Auto 2", AUTO_2);
        pathChooser.addOption("Auto 3", AUTO_3);
        SmartDashboard.putData("Auto choices", pathChooser);
    }


    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        // Add button to command mappings here.
        // See https://docs.wpilib.org/en/stable/docs/softwarc e/commandbased/binding-commands-to-triggers.html
        new JoystickButton(controller1, 7).whenPressed(new InstantCommand(drive::zeroYaw));

        // control intake
        intake.setDefaultCommand(new RunCommand(() -> {
            if (!teleop) return;
            if (controller1.getBButton() || controller2.getBButton()) {
                intake.enableIntake(1);
            } else if (controller1.getXButton() || controller2.getXButton()) {
                intake.enableIntake(-1);
            } else {
                intake.disableIntake();
            }
        }, intake));

        // control left and right hang hooks
        hang.setDefaultCommand(new RunCommand(() -> {
            if (!teleop) return;
            if (hang.isZeroing()) return;
            double value = (controller1.getRightBumper() ? 1 : 0) - (controller1.getLeftBumper() ? 1 : 0);
            double valueCenter = (controller2.getRightBumper() ? 0.8 : 0) - (controller2.getLeftBumper() ? 1 : 0);
            if (value > 0) {
                hang.extendSolenoid();
            } else if (value < 0) {
                hang.collapseSolenoid();
            } else {
                hang.stopSolenoid();
            }
            hang.setCenterHook(valueCenter);
            double left = controller1.getLeftTriggerAxis();
            double right = controller1.getRightTriggerAxis();
            left = Math.abs(left) >= 0.05 ? 1 : 0;
            right = Math.abs(right) >= 0.05 ? 0.8 : 0;
            hang.setLeftAndRightHook(right - left);
            setColorLoop(value != 0 || valueCenter != 0 || right != 0 || left != 0);
        }, hang));
        // control shoot motors and transfer motor
        shoot.setDefaultCommand(new RunCommand(() -> {
            if (!teleop) return;
            if (shoot.isZeroing()) return;
            if (controller2.getLeftTriggerAxis() >= 0.2) {
                hang.disableCompressor();
                shoot.enableShootMotor();
            } else {
                shoot.disableShootMotor();
                hang.enableCompressor();
            }
            if (controller2.getRightTriggerAxis() >= 0.2) {
                shoot.setTransferMotorSpeed(0.8);
            } else {
                shoot.stopTransferMotor();
            }
            shoot.setAngleMotor(controller2.getLeftY());
        }, shoot));
    }


    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        Pose2d startPoint = new Pose2d(0, 0, new Rotation2d(0));
        Pose2d endPoint = new Pose2d(1,0 , new Rotation2d(0));;
        List<Translation2d> waypoints = new ArrayList<>();
        switch (pathChooser.getSelected()) {
            case RobotContainer.AUTO_1:
                endPoint = new Pose2d(1,0 , new Rotation2d(0));
                break;
            case RobotContainer.AUTO_2:
                endPoint = new Pose2d(2,0 , new Rotation2d(0));
                waypoints = List.of(
                        new Translation2d(1,0)
                );
                break;
            case RobotContainer.AUTO_3:
                endPoint = new Pose2d(1.5, 0, new Rotation2d(0));
                break;
            default:
                break;
        }
        var autoVoltageConstraint = new MecanumDriveKinematicsConstraint(drive.getDriveKinematics(), 2);
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                startPoint,
                // Pass through these two interior waypoints, making an 's' curve path
                waypoints,
                // End 3 meters straight ahead of where we started, facing forward
                endPoint,
                // Pass config
                new TrajectoryConfig(2, 0.85)
                        // Add kinematics to ensure max speed is actually obeyed
                        .setKinematics(drive.getDriveKinematics())
                        // Apply the voltage constraint
                        .addConstraint(autoVoltageConstraint)
        );
        autoCommand  = new SequentialCommandGroup(
                new InstantCommand(shoot::enableShootMotor, shoot),
                new ParallelCommandGroup(
                        new RunTrajectoryCommand(drive, trajectory),
                        new SequentialCommandGroup(
                                new DelayCommand(0.25),
                                new InstantCommand(() -> intake.enableIntake(1), intake)
                        ),
                        new SequentialCommandGroup(
                                new DelayCommand(0.5),
                                new InstantCommand(() -> {
                                    shoot.overrideRotate(-0.4);
                                },shoot),
                                new DelayCommand(0.25),
                                new InstantCommand(() -> {
                                    shoot.setAngleMotor(0.5);
                                    shoot.cancelOverrideRotate();
                                }),
                                new WaitShootSpeedCommand(shoot,3300),
                                new DelayCommand(0.5),
                                new InstantCommand(() -> shoot.setTransferMotorSpeed(0.8),shoot),
                                new DelayCommand(0.5),
                                new InstantCommand(shoot::stopTransferMotor)
                        )
                ),
                new InstantCommand(intake::disableIntake, intake),
                new InstantCommand(() -> shoot.setAngleMotor(0)),
                new InstantCommand(() -> shoot.setTransferMotorSpeed(0.8),shoot),
                new DelayCommand(1),
                new InstantCommand(shoot::stopTransferMotor, shoot),
                new InstantCommand(shoot::disableShootMotor, shoot)
        );
        return autoCommand;
    }

    public void robotInit() {
        drive.zeroYaw();
        setColorLoop(true);
        shoot.disableLimeLightGreenLED();
    }

    public void autonomousInit() {
        teleop = false;
        alliance = DriverStation.getAlliance();
        shoot.setAlliance(alliance);
        intake.setAlliance(alliance);
        shoot.enableLimelightGreenLED();
        shoot.zero();
        hang.zero();
        setColorLoop(false);
    }

    public void teleopInit() {
        teleop = true;
        alliance = DriverStation.getAlliance();
        shoot.setAlliance(alliance);
        intake.setAlliance(alliance);
        shoot.enableLimelightGreenLED();
        shoot.zero();
        hang.zero();
        setColorLoop(false);
    }

    public void testInit() {
        teleop = false;
        alliance = DriverStation.getAlliance();
        shoot.setAlliance(alliance);
        intake.setAlliance(alliance);
        shoot.enableLimelightGreenLED();
        shoot.zero();
        hang.zero();
        setColorLoop(false);
    }

    public void robotPeriodic() {
        rainbow();
    }

    public void disableInit() {
        teleop = false;
        shoot.disableLimeLightGreenLED();
        setColorLoop(true);
    }

    private void rainbow() {
        if (!enableColorLoop) return;
        // For every pixel
        for (var i = 0; i < hangLightStripBuffer.getLength(); i++) {
            // Calculate the hue - hue is easier for rainbows because the color
            // shape is a circle so only one value needs to precess
            final var hue = (m_rainbowFirstPixelHue + (i * 180 / hangLightStripBuffer.getLength())) % 180;
            // Set the value
            hangLightStripBuffer.setHSV(i, hue, 255, 128);
        }
        // Increase by to make the rainbow "move"
        m_rainbowFirstPixelHue += 3;
        // Check bounds
        m_rainbowFirstPixelHue %= 180;
        hangLightStrip.setData(hangLightStripBuffer);
    }

    private void setColorLoop(boolean enable) {
        if (enable) {
            enableColorLoop = true;
        } else if (enableColorLoop) {
            enableColorLoop = false;
            for (int i = 0; i < hangLightStripBuffer.getLength(); i++) {
                hangLightStripBuffer.setRGB(i, alliance == DriverStation.Alliance.Red ? 255 : 0, 0, alliance == DriverStation.Alliance.Blue ? 255 : 0);
            }
            hangLightStrip.setData(hangLightStripBuffer);
        }
    }
}
