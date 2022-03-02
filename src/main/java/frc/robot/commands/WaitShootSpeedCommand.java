package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShootSubsystem;


public class WaitShootSpeedCommand extends CommandBase {
    private final ShootSubsystem shootSubsystem;
    private final double targetRPM;

    public WaitShootSpeedCommand(ShootSubsystem shootSubsystem, double targetRPM) {
        this.shootSubsystem = shootSubsystem;
        this.targetRPM = targetRPM;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.shootSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return Math.abs(shootSubsystem.getShootMotorVelocity()-targetRPM)<200;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
