package frc.robot.commands;


import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveMotorVoltages;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.math.trajectory.constraint.MecanumDriveKinematicsConstraint;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HangSubsystem;
import frc.robot.subsystems.ShootSubsystem;

import java.util.List;

public class AutonomousCommandGroup extends SequentialCommandGroup {

    public AutonomousCommandGroup(DriveSubsystem driveSubsystem, HangSubsystem hangSubsystem, ShootSubsystem shootSubsystem, SendableChooser<String> pathChooser) {
        var autoVoltageConstraint = new MecanumDriveKinematicsConstraint(driveSubsystem.getDriveKinematics(), 0.5);
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(
                        new Translation2d(0.5, 0)
                ),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(1, 0, new Rotation2d(0)),
                // Pass config
                new TrajectoryConfig(0.5, 0.5)
                        // Add kinematics to ensure max speed is actually obeyed
                        .setKinematics(driveSubsystem.getDriveKinematics())
                        // Apply the voltage constraint
                        .addConstraint(autoVoltageConstraint)
        );
        addCommands(new RunTrajectoryCommand(driveSubsystem, trajectory));
    }
}