package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

import java.util.function.Supplier;

import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;


public class RunTrajectoryCommand extends CommandBase {
    private final Timer timer = new Timer();
    private final Trajectory trajectory;
    private final Supplier<Pose2d> pose;
    private final RamseteController follower;
    private final SimpleMotorFeedforward feedforward;
    private final MecanumDriveKinematics kinematics;
    private final Supplier<MecanumDriveWheelSpeeds> speeds;
    private final PIDController frontLeftController;
    private final PIDController frontRightController;
    private final PIDController rearLeftController;
    private final PIDController rearRightController;
    private MecanumDriveWheelSpeeds prevSpeeds;
    private double prevTime;
    private final DriveSubsystem drive;

    public RunTrajectoryCommand(DriveSubsystem driveTrainSubsystem, Trajectory trajectory) {
        this.trajectory = requireNonNullParam(trajectory, "trajectory", "RamseteCommand");
        pose = driveTrainSubsystem::getPose;
        follower = new RamseteController();
        feedforward = new SimpleMotorFeedforward(Constants.FF_CHASSIS_ksVolts,
                Constants.FF_CHASSIS_kvVoltSecondsPerMeter,
                Constants.FF_CHASSIS_kaVoltSecondsSquaredPerMeter);
        kinematics = driveTrainSubsystem.getDriveKinematics();
        speeds = driveTrainSubsystem::getWheelSpeeds;
        frontLeftController = new PIDController(Constants.PID_CHASSIS[0], 0, 0);
        frontRightController = new PIDController(Constants.PID_CHASSIS[0], 0, 0);
        rearLeftController = new PIDController(Constants.PID_CHASSIS[0], 0, 0);
        rearRightController = new PIDController(Constants.PID_CHASSIS[0], 0, 0);
        drive = driveTrainSubsystem;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        prevTime = -1;
        var initialState = trajectory.sample(0);
        prevSpeeds =
                kinematics.toWheelSpeeds(
                        new ChassisSpeeds(
                                initialState.velocityMetersPerSecond,
                                0,
                                initialState.curvatureRadPerMeter * initialState.velocityMetersPerSecond));
        frontLeftController.reset();
        frontRightController.reset();
        rearLeftController.reset();
        rearRightController.reset();
        drive.resetOdometry(trajectory.getInitialPose());
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        double curTime = timer.get();
        double dt = curTime - prevTime;

        if (prevTime < 0) {
            drive.driveVolts(0.0, 0.0, 0.0, 0.0);
            prevTime = curTime;
            return;
        }

        var targetWheelSpeeds =
                kinematics.toWheelSpeeds(
                        follower.calculate(pose.get(), trajectory.sample(curTime)));

        var frontLeftSpeedSetpoint = targetWheelSpeeds.frontLeftMetersPerSecond;
        var frontRightSpeedSetpoint = targetWheelSpeeds.frontRightMetersPerSecond;
        var rearLeftSpeedSetpoint = targetWheelSpeeds.rearLeftMetersPerSecond;
        var rearRightSpeedSetpoint = targetWheelSpeeds.rearRightMetersPerSecond;

        double frontLeftOutput, frontRightOutput, rearLeftOutput, rearRightOutput;

        double frontLeftFeedforward =
                feedforward.calculate(
                        frontLeftSpeedSetpoint, (frontLeftSpeedSetpoint - prevSpeeds.frontLeftMetersPerSecond) / dt);

        double frontRightFeedforward =
                feedforward.calculate(
                        frontRightSpeedSetpoint, (frontRightSpeedSetpoint - prevSpeeds.frontRightMetersPerSecond) / dt);

        double rearLeftFeedforward =
                feedforward.calculate(
                        rearLeftSpeedSetpoint, (rearLeftSpeedSetpoint - prevSpeeds.rearLeftMetersPerSecond) / dt);
        double rearRightFeedforward =
                feedforward.calculate(
                        rearRightSpeedSetpoint, (rearRightSpeedSetpoint - prevSpeeds.rearRightMetersPerSecond) / dt);
        frontLeftOutput =
                frontLeftFeedforward
                        + frontLeftController.calculate(speeds.get().frontLeftMetersPerSecond, frontLeftSpeedSetpoint);

        frontRightOutput =
                frontRightFeedforward
                        + frontRightController.calculate(speeds.get().frontRightMetersPerSecond, frontRightSpeedSetpoint);

        rearLeftOutput =
                rearLeftFeedforward
                        + rearLeftController.calculate(speeds.get().rearLeftMetersPerSecond, rearLeftSpeedSetpoint);

        rearRightOutput =
                rearRightFeedforward
                        + rearRightController.calculate(speeds.get().rearRightMetersPerSecond, rearRightSpeedSetpoint);

        drive.driveVolts(frontLeftOutput, frontRightOutput, rearLeftOutput, rearRightOutput);
        prevSpeeds = targetWheelSpeeds;
        prevTime = curTime;
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(trajectory.getTotalTimeSeconds());
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();

        if (interrupted) {
            drive.driveVolts(0.0, 0.0, 0.0, 0.0);
        }
    }
}
