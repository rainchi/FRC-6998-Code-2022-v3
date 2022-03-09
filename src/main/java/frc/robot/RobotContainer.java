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
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.DelayCommand;
import frc.robot.commands.RunTrajectoryCommand;
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
    private final DriveSubsystem drive = new DriveSubsystem();
    private final HangSubsystem hang = new HangSubsystem();
    private final ShootSubsystem shoot = new ShootSubsystem(drive);

    private final SendableChooser<String> pathChooser = new SendableChooser<>();

    private final AddressableLED hangLightStrip = new AddressableLED(9);
    private final AddressableLEDBuffer hangLightStripBuffer = new AddressableLEDBuffer(148);

    private final NetworkTableInstance nt = NetworkTableInstance.getDefault();

    public static final String AUTO_1 = "Auto 1";
    public static final String AUTO_2 = "Auto 2";
    public static final String AUTO_3 = "Auto 3";

    private DriverStation.Alliance alliance;

    private boolean teleop = false;

    private double povYFwd = 0;
    private double povYBwd = 0;
    private double povZCCW = 0;
    private double povZCW = 0;

    private int blinkCount = 0;
    private int lastBrightness = 0;

    private double baseSpeed = 2100;
    private double xOffset = 0;


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
        new POVButton(controller1, 0)
                .whenPressed(new InstantCommand(() -> povYFwd = 0.3)).whenReleased(new InstantCommand(() -> povYFwd = 0));
        new POVButton(controller1, 90)
                .whenPressed(new InstantCommand(() -> povZCW = 0.3)).whenReleased(new InstantCommand(() -> povZCW = 0));
        new POVButton(controller1, 180)
                .whenPressed(new InstantCommand(() -> povYBwd = -0.3)).whenReleased(new InstantCommand(() -> povYBwd = 0));
        new POVButton(controller1, 270)
                .whenPressed(new InstantCommand(() -> povZCCW = -0.3)).whenReleased(new InstantCommand(() -> povZCCW = 0));

        new POVButton(controller2, 0)
                .whenPressed(new InstantCommand(() -> baseSpeed += 50));
        new POVButton(controller2, 90)
                .whenPressed(new InstantCommand(() -> xOffset -= 0.2));
        new POVButton(controller2, 180)
                .whenPressed(new InstantCommand(() -> baseSpeed -= 50));
        new POVButton(controller2, 270)
                .whenPressed(new InstantCommand(() -> xOffset += 0.2));

        new JoystickButton(controller2, 8).whenPressed(new InstantCommand(() -> shoot.forceDisableAlignment(!shoot.isForceDisableAlignment())));


        // control intake
        intake.setDefaultCommand(new RunCommand(() -> {
            if (!teleop) return;
            if (controller1.getBButton() || controller2.getBButton()) {
                intake.enableIntake(false);
            } else if (controller1.getXButton() || controller2.getXButton()) {
                intake.enableIntake(true);
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
        }, hang));
        // control shoot motors and transfer motor
        shoot.setDefaultCommand(new RunCommand(() -> {
            if (!teleop) return;
            if (controller2.getLeftTriggerAxis() >= 0.2) {
                hang.disableCompressor();
                shoot.enableShootMotor();
            } else {
                shoot.disableShootMotor();
                hang.enableCompressor();
            }
            if (controller2.getRightTriggerAxis() >= 0.2) {
                shoot.setTransferMotorSpeed(0.8);
            } else if (controller2.getYButton()) {
                shoot.setTransferMotorSpeed(-0.8);
            } else {
                shoot.stopTransferMotor();
            }
            double rot = controller2.getRightX();
            rot = Math.abs(rot) >= 0.2 ? rot : 0;
            if (rot != 0) {
                shoot.overrideRotate(Math.copySign(0.5, rot));
            } else {
                shoot.cancelOverrideRotate();
            }
            shoot.setBaseSpeed(baseSpeed);
            shoot.setXOffset(xOffset);
            SmartDashboard.putNumber("Base Speed", baseSpeed);
        }, shoot));
    }


    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        Pose2d startPoint = new Pose2d(0, 0, new Rotation2d(0));
        Pose2d endPoint = new Pose2d(1, 0, new Rotation2d(0));
        ;
        List<Translation2d> waypoints = new ArrayList<>();
        switch (pathChooser.getSelected()) {
            case RobotContainer.AUTO_1:
                endPoint = new Pose2d(1, 0, new Rotation2d(0));
                break;
            case RobotContainer.AUTO_2:
                endPoint = new Pose2d(2, 0, new Rotation2d(0));
                waypoints = List.of(
                        new Translation2d(1, 0)
                );
                break;
            case RobotContainer.AUTO_3:
                endPoint = new Pose2d(3.74, -1.22, Rotation2d.fromDegrees(15));
                break;
            default:
                break;
        }
        var autoVoltageConstraint = new MecanumDriveKinematicsConstraint(drive.getDriveKinematics(), 4);
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                startPoint,
                // Pass through these two interior waypoints, making an 's' curve path
                waypoints,
                // End 3 meters straight ahead of where we started, facing forward
                endPoint,
                // Pass config
                new TrajectoryConfig(4, 1)
                        // Add kinematics to ensure max speed is actually obeyed
                        .setKinematics(drive.getDriveKinematics())
                        // Apply the voltage constraint
                        .addConstraint(autoVoltageConstraint)
        );
//        Command autoCommand = new SequentialCommandGroup(
//                new InstantCommand(shoot::enableShootMotor, shoot),
//                new ParallelCommandGroup(
//                        new RunTrajectoryCommand(drive, trajectory),
//                        new SequentialCommandGroup(
//                                new DelayCommand(0.25),
//                                new InstantCommand(() -> intake.enableIntake(1), intake)
//                        ),
//                        new SequentialCommandGroup(
//                                new DelayCommand(0.5),
//                                new InstantCommand(() -> {
//                                    shoot.overrideRotate(-0.4);
//                                }, shoot),
//                                new DelayCommand(0.25),
//                                new InstantCommand(() -> {
//                                    shoot.setAngleMotor(0.5);
//                                    shoot.cancelOverrideRotate();
//                                }),
//                                new WaitShootSpeedCommand(shoot, 3300),
//                                new DelayCommand(1),
//                                new InstantCommand(() -> shoot.setTransferMotorSpeed(0.8), shoot),
//                                new DelayCommand(0.5),
//                                new InstantCommand(shoot::stopTransferMotor)
//                        )
//                ),
//                new InstantCommand(intake::disableIntake, intake),
//                new InstantCommand(() -> shoot.setAngleMotor(0)),
//                new InstantCommand(() -> shoot.setTransferMotorSpeed(0.8), shoot),
//                new DelayCommand(1),
//                new InstantCommand(shoot::stopTransferMotor, shoot),
//                new InstantCommand(shoot::disableShootMotor, shoot)
//        );


        Command autoCommand = new SequentialCommandGroup(
                new InstantCommand(() -> shoot.enableShootMotor(3570), shoot),
                new InstantCommand(() -> intake.enableIntake(false), intake),
                new ParallelCommandGroup(
                        new RunTrajectoryCommand(drive, TrajectoryGenerator.generateTrajectory(
                                // Start at the origin facing the +X direction
                                new Pose2d(0, 0, new Rotation2d(0)),
                                // Pass through these two interior waypoints, making an 's' curve path
                                new ArrayList<>(),
                                // End 3 meters straight ahead of where we started, facing forward
                                new Pose2d(1.2, 0, new Rotation2d(0)),
                                // Pass config
                                new TrajectoryConfig(4, 1)
                                        // Add kinematics to ensure max speed is actually obeyed
                                        .setKinematics(DriveSubsystem.getDriveKinematics())
                                        // Apply the voltage constraint
                                        .addConstraint(autoVoltageConstraint)
                        ), new Rotation2d(0)),
                        new SequentialCommandGroup(
                                new InstantCommand(() -> {
                                    shoot.overrideRotate(-0.4);
                                }, shoot),
                                new DelayCommand(0.6),
                                new InstantCommand(shoot::cancelOverrideRotate)
                        )
                ),
                //new InstantCommand(intake::disableIntake, intake),
                new DelayCommand(1.6),
                new InstantCommand(() -> shoot.setTransferMotorSpeed(0.8), shoot),
                new DelayCommand(0.35),
                new InstantCommand(shoot::stopTransferMotor),
                new InstantCommand(() -> shoot.enableShootMotor(3650), shoot),
                // must be 1 if run full path
                new DelayCommand(5),
                new InstantCommand(() -> shoot.setTransferMotorSpeed(0.8), shoot),
                new DelayCommand(0.5),
                new InstantCommand(shoot::stopTransferMotor, shoot),


//                new InstantCommand(() -> intake.enableIntake(1), intake),
//                new InstantCommand(new Runnable() {
//                    @Override
//                    public void run() {
//                        shoot.enableShootMotor(4300);
//                    }
//                }, shoot),
//                new RunTrajectoryCommand(drive, TrajectoryGenerator.generateTrajectory(
//                        // Start at the origin facing the +X direction
//                        new Pose2d(1.2, 0, new Rotation2d(0)),
//                        // Pass through these two interior waypoints, making an 's' curve path
//                        new ArrayList<>(),
//                        // End 3 meters straight ahead of where we started, facing forward
//                        new Pose2d(3.9, -1.48, Rotation2d.fromDegrees(15)),
//                        // Pass config
//                        new TrajectoryConfig(4, 1)
//                                // Add kinematics to ensure max speed is actually obeyed
//                                .setKinematics(drive.getDriveKinematics())
//                                // Apply the voltage constraint
//                                .addConstraint(autoVoltageConstraint)
//                ), Rotation2d.fromDegrees(15)),
//                new CommandBase() {
//                    Timer timer = new Timer();
//
//                    @Override
//                    public void initialize() {
//                        timer.reset();
//                        timer.start();
//
//                    }
//
//                    @Override
//                    public void execute() {
//                        drive.driveCartesian(-1, 0, 0);
//                    }
//
//                    @Override
//                    public void end(boolean interrupted) {
//                        drive.driveCartesian(0, 0, 0);
//                        timer.stop();
//                    }
//
//                    @Override
//                    public boolean isFinished() {
//                        return timer.hasElapsed(1);
//                    }
//                },
//                new DelayCommand(1.5),
//                new InstantCommand(() -> shoot.setTransferMotorSpeed(0.8)),
//                new DelayCommand(3),
                new InstantCommand(shoot::stopTransferMotor, shoot),
                new InstantCommand(shoot::disableShootMotor, shoot),
                new InstantCommand(intake::disableIntake, intake)
        );
        return autoCommand;
    }

    public void robotInit() {
        drive.zeroYaw();
        shoot.disableLimeLightGreenLED();
    }

    public void autonomousInit() {
        teleop = false;
        alliance = DriverStation.getAlliance();
        shoot.setAlliance(alliance);
        shoot.enableLimelightGreenLED();
        hang.zero();
    }

    public void teleopInit() {
        teleop = true;
        alliance = DriverStation.getAlliance();
        shoot.setAlliance(alliance);
        shoot.enableLimelightGreenLED();
        hang.zero();
        shoot.forceDisableAlignment(false);
    }

    public void testInit() {
        teleop = false;
        alliance = DriverStation.getAlliance();
        shoot.setAlliance(alliance);
        shoot.enableLimelightGreenLED();
        hang.zero();
    }

    public void robotPeriodic() {
        handleColor();
    }

    public void disableInit() {
        teleop = false;
        shoot.disableLimeLightGreenLED();
    }

    public void teleopPeriodic(){
        double ySpeed = -controller1.getLeftY();
        double xSpeed = controller1.getLeftX();
        double zRotation = controller1.getRightX();
        ySpeed = Math.abs(ySpeed) >= Constants.CHASSIS_DEADLINE ? ySpeed : 0;
        xSpeed = Math.abs(xSpeed) >= Constants.CHASSIS_DEADLINE ? xSpeed : 0;
        zRotation = Math.abs(zRotation) >= Constants.CHASSIS_DEADLINE ? zRotation : 0;
        drive.driveCartesian(ySpeed+povYFwd+povYBwd, xSpeed, zRotation+povZCW+povZCCW, 0);
    }

    private void handleColor() {
        if (teleop){
            double distance = shoot.getTargetDistance();
            double currentRPM = shoot.getShootMotorVelocity();
            double targetRPM = shoot.getTargetVelocity();
            if (shoot.isForceDisableAlignment() || (distance >= 1.9 && distance <= 2.4)) {
                // green
                if (currentRPM >= 60 && Math.abs(currentRPM-targetRPM)<=70){
                    // green blink
                    blinkCount++;
                    if (blinkCount>=6){
                        blinkCount=0;
                        lastBrightness = 100 - lastBrightness;
                        for (int i = 0; i < hangLightStripBuffer.getLength(); i++) {
                            hangLightStripBuffer.setRGB(i, 0, lastBrightness, 0);
                        }
                    }
                }else{
                    for (int i = 0; i < hangLightStripBuffer.getLength(); i++) {
                        hangLightStripBuffer.setRGB(i, 0, 100, 0);
                    }
                    blinkCount=0;
                    lastBrightness = 0;
                }
            } else if (distance >= 1.8 && distance <= 3.1) {
                // yellow
                if (currentRPM>=60 && Math.abs(currentRPM-targetRPM)<=70){
                    // yellow blink
                    blinkCount++;
                    if (blinkCount>=6){
                        blinkCount=0;
                        lastBrightness = 50 - lastBrightness;
                        for (int i = 0; i < hangLightStripBuffer.getLength(); i++) {
                            hangLightStripBuffer.setRGB(i, lastBrightness, lastBrightness, 0);
                        }
                    }
                }else{
                    for (int i = 0; i < hangLightStripBuffer.getLength(); i++) {
                        hangLightStripBuffer.setRGB(i, 50, 50, 0);
                    }
                    blinkCount = 0;
                    lastBrightness = 0;
                }
            } else {
                // alliance color
                for (int i = 0; i < hangLightStripBuffer.getLength(); i++) {
                    hangLightStripBuffer.setRGB(i, alliance == DriverStation.Alliance.Red ? 150 : 0, 0, alliance == DriverStation.Alliance.Blue ? 150 : 0);
                }
            }
        }else{
            for (int i = 0; i < hangLightStripBuffer.getLength(); i++) {
                hangLightStripBuffer.setRGB(i, alliance == DriverStation.Alliance.Red ? 150 : 0, 0, alliance == DriverStation.Alliance.Blue ? 150 : 0);
            }
        }
        hangLightStrip.setData(hangLightStripBuffer);
    }
}
