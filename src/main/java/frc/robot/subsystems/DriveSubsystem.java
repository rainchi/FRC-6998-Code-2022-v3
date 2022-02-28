package frc.robot.subsystems;


import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.math.MathUsageId;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {
    private final CANSparkMax motorFrontLeft = new CANSparkMax(Constants.MOTOR_CHASSIS_FL, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax motorFrontRight = new CANSparkMax(Constants.MOTOR_CHASSIS_FR, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax motorRearLeft = new CANSparkMax(Constants.MOTOR_CHASSIS_RL, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax motorRearRight = new CANSparkMax(Constants.MOTOR_CHASSIS_RR, CANSparkMaxLowLevel.MotorType.kBrushless);

    private final MecanumDrive drive = new MecanumDrive(motorFrontLeft, motorRearLeft, motorFrontRight, motorRearRight);

    private final AHRS navX = new AHRS(SPI.Port.kMXP);

    private final XboxController driveController;

    private final MecanumDriveKinematics driveKinematics = new MecanumDriveKinematics(new Translation2d(0.35, 0.28), new Translation2d(0.35, -0.28), new Translation2d(-0.35, 0.28), new Translation2d(-0.35, -0.28));

    private final MecanumDriveOdometry odometry;

    public DriveSubsystem(XboxController driveController) {
        this.driveController = driveController;
        zeroYaw();
        // Odometry
        odometry = new MecanumDriveOdometry(driveKinematics, navX.getRotation2d());
        // Restore to factory defaults
        motorFrontLeft.restoreFactoryDefaults();
        motorFrontRight.restoreFactoryDefaults();
        motorRearLeft.restoreFactoryDefaults();
        motorRearRight.restoreFactoryDefaults();
        // Set current limit
        motorFrontLeft.setSmartCurrentLimit(30);
        motorFrontRight.setSmartCurrentLimit(30);
        motorRearLeft.setSmartCurrentLimit(30);
        motorRearRight.setSmartCurrentLimit(30);
        // Set idle mode to brake
        motorFrontLeft.setIdleMode(CANSparkMax.IdleMode.kCoast);
        motorFrontRight.setIdleMode(CANSparkMax.IdleMode.kCoast);
        motorRearLeft.setIdleMode(CANSparkMax.IdleMode.kCoast);
        motorRearRight.setIdleMode(CANSparkMax.IdleMode.kCoast);
        // Set invert
        motorFrontLeft.setInverted(Constants.MOTOR_CHASSIS_FL_INVERTED);
        motorFrontRight.setInverted(Constants.MOTOR_CHASSIS_FR_INVERTED);
        motorRearLeft.setInverted(Constants.MOTOR_CHASSIS_RL_INVERTED);
        motorRearRight.setInverted(Constants.MOTOR_CHASSIS_RR_INVERTED);

        setDefaultCommand(new RunCommand(() -> {
            double ySpeed = -driveController.getLeftY();
            double xSpeed = driveController.getLeftX();
            double zRotation = driveController.getRightX();
            ySpeed = Math.abs(ySpeed) >= Constants.CHASSIS_DEADLINE ? ySpeed : 0;
            xSpeed = Math.abs(xSpeed) >= Constants.CHASSIS_DEADLINE ? xSpeed : 0;
            zRotation = Math.abs(zRotation) >= Constants.CHASSIS_DEADLINE ? zRotation : 0;
            driveCartesian(ySpeed, xSpeed, zRotation, navX.getAngle());
        }, this));
    }

    @Override
    public void periodic() {
        odometry.update(navX.getRotation2d(), getWheelSpeeds());
        if (Constants.DEBUG){
            ChassisSpeeds speed = driveKinematics.toChassisSpeeds(getWheelSpeeds());
            SmartDashboard.putNumber("DriveSubsystem Kinematics Vx", speed.vxMetersPerSecond);
            SmartDashboard.putNumber("DriveSubsystem Kinematics Vy", speed.vyMetersPerSecond);
            SmartDashboard.putNumber("DriveSubsystem Kinematics Degrees", Math.toDegrees(speed.omegaRadiansPerSecond));
            Pose2d pose = odometry.getPoseMeters();
            SmartDashboard.putNumber("DriveSubsystem Odometry Px", pose.getX());
            SmartDashboard.putNumber("DriveSubsystem Odometry Py", pose.getY());
            SmartDashboard.putNumber("DriveSubsystem Odometry Rotation", pose.getRotation().getDegrees());
        }
    }

    public void zeroYaw() {
        navX.zeroYaw();
    }


    public AHRS getNavX() {
        return navX;
    }

    public void driveCartesian(double ySpeed, double xSpeed, double zRotation) {
        driveCartesian(ySpeed, xSpeed, zRotation, 0.0);
    }

    public void driveCartesian(double ySpeed, double xSpeed, double zRotation, double gyroAngle) {
        drive.driveCartesian(ySpeed, xSpeed, zRotation, gyroAngle);
    }

    public void driveVolts(double frontLeftVolts, double frontRightVolts, double rearLeftVolts, double rearRightVolts) {
        frontLeftVolts = MathUtil.clamp(frontLeftVolts, -7, 7);
        frontRightVolts = MathUtil.clamp(frontRightVolts, -7, 7);
        rearLeftVolts = MathUtil.clamp(rearLeftVolts, -7, 7);
        rearRightVolts = MathUtil.clamp(rearRightVolts, -7, 7);
        motorFrontLeft.setVoltage(frontLeftVolts);
        motorFrontRight.setVoltage(frontRightVolts);
        motorRearLeft.setVoltage(rearLeftVolts);
        motorRearRight.setVoltage(rearRightVolts);
        drive.feed();
    }

    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    /**
     * Returns the current wheel speeds of the robot.
     *
     * @return The current wheel speeds.
     */
    public MecanumDriveWheelSpeeds getWheelSpeeds() {
        return new MecanumDriveWheelSpeeds(motorFrontLeft.getEncoder().getVelocity() / 60.0 / Constants.CHASSIS_GEARING * Constants.DISTANCE_METER_PER_ROTATION, motorFrontRight.getEncoder().getVelocity() / 60.0 / Constants.CHASSIS_GEARING * Constants.DISTANCE_METER_PER_ROTATION, motorRearLeft.getEncoder().getVelocity() / 60.0 / Constants.CHASSIS_GEARING * Constants.DISTANCE_METER_PER_ROTATION, motorRearRight.getEncoder().getVelocity() / 60.0 / Constants.CHASSIS_GEARING * Constants.DISTANCE_METER_PER_ROTATION);
    }

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetOdometry(Pose2d pose) {
        initializeEncoders();
        odometry.resetPosition(pose, navX.getRotation2d());
    }

    public void initializeEncoders() {
        motorFrontLeft.getEncoder().setPosition(0);
        motorFrontRight.getEncoder().setPosition(0);
        motorRearLeft.getEncoder().setPosition(0);
        motorRearRight.getEncoder().setPosition(0);
    }

    public MecanumDriveKinematics getDriveKinematics() {
        return driveKinematics;
    }

    public double getTurnRate() {
        return navX.getRate();
    }
}

