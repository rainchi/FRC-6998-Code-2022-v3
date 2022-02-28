// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.AutonomousCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HangSubsystem;
import frc.robot.subsystems.CollectSubsystem;
import frc.robot.subsystems.ShootSubsystem;


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
    private final DriveSubsystem drive = new DriveSubsystem(controller1);
    private final HangSubsystem hang = new HangSubsystem();
    private final ShootSubsystem shoot = new ShootSubsystem(drive);
    private final CollectSubsystem intake = new CollectSubsystem();

    private final SendableChooser<String> pathChooser = new SendableChooser<>();
    private final AutonomousCommandGroup autoCommand = new AutonomousCommandGroup(drive, hang, shoot, pathChooser);

    private final AddressableLED rgbStrip = new AddressableLED(9);
    private final AddressableLEDBuffer buffer = new AddressableLEDBuffer(114);

    private int m_rainbowFirstPixelHue = 0;

    private boolean enableColorLoop = false;

    //Command autoCommand = new InstantCommand();


    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();
        rgbStrip.setLength(buffer.getLength());
        rgbStrip.start();
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
            if (controller1.getBButton() || controller2.getBButton()){
                intake.enableIntake(1);
            }else if(controller1.getXButton() || controller2.getXButton()){
                intake.enableIntake(-1);
            }else{
                intake.disableIntake();
            }
        }, intake));

        // control left and right hang hooks
        hang.setDefaultCommand(new RunCommand(() -> {
            if(hang.isZeroing()) return;
            double value = (controller1.getRightBumper()?1:0) - (controller1.getLeftBumper()?1:0);
            double valueCenter = (controller2.getRightBumper()?0.8:0) - (controller2.getLeftBumper()?1:0);
            if (value>0){
                hang.extendSolenoid();
            }else if(value<0){
                hang.collapseSolenoid();
            }else{
                hang.stopSolenoid();
            }
            hang.setCenterHook(valueCenter);
            double left = controller1.getLeftTriggerAxis();
            double right = controller1.getRightTriggerAxis();
            left = Math.abs(left)>=0.05?1:0;
            right = Math.abs(right)>=0.05?0.8:0;
            hang.setLeftAndRightHook(right-left);

            setColorLoop(value!=0||valueCenter!=0||right!=0||left!=0);

        }, hang));
        // control shoot motors and transfer motor
        shoot.setDefaultCommand(new RunCommand(() -> {
            if(shoot.isZeroing()) return;
            if(controller2.getLeftTriggerAxis()>=0.2){
                shoot.enableShootMotor();
            }else{
                shoot.disableShootMotor();
            }
            if(controller2.getRightTriggerAxis()>=0.2){
                shoot.setTransferMotorSpeed(1);
            }else{
                shoot.stopTransferMotor();
            }
        }, shoot));
    }


    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return autoCommand;
    }

    public void robotInit(){
        drive.zeroYaw();
    }

    public void autonomousInit() {

    }

    public void teleopInit(){
        shoot.zero();
        hang.zero();
        DriverStation.Alliance alliance = DriverStation.getAlliance();
        for (int i =0;i<buffer.getLength();i++){
            buffer.setRGB(i, alliance== DriverStation.Alliance.Red?255:0,0,alliance== DriverStation.Alliance.Blue?255:0);
        }
        rgbStrip.setData(buffer);
    }

    public void testInit(){

    }

    public void robotPeriodic(){
        rainbow();

    }

    private void rainbow() {
        if (!enableColorLoop) return;
        // For every pixel
        for (var i = 0; i < buffer.getLength(); i++) {
            // Calculate the hue - hue is easier for rainbows because the color
            // shape is a circle so only one value needs to precess
            final var hue = (m_rainbowFirstPixelHue + (i * 180 / buffer.getLength())) % 180;
            // Set the value
            buffer.setHSV(i, hue, 255, 128);
        }
        // Increase by to make the rainbow "move"
        m_rainbowFirstPixelHue += 12;
        // Check bounds
        m_rainbowFirstPixelHue %= 180;
        rgbStrip.setData(buffer);
    }

    private void setColorLoop(boolean enable){
        if (enable){
            enableColorLoop = true;
        }else if(enableColorLoop){
            enableColorLoop = false;
            DriverStation.Alliance alliance = DriverStation.getAlliance();
            for (int i =0;i<buffer.getLength();i++){
                buffer.setRGB(i, alliance== DriverStation.Alliance.Red?255:0,0,alliance== DriverStation.Alliance.Blue?255:0);
            }
            rgbStrip.setData(buffer);
        }
    }
}
