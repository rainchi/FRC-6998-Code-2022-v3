package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShootSubsystem extends SubsystemBase {
    private boolean zeroing = false;
    private boolean shoot = false;
    private boolean hasTarget = false;
    private double targetDistance = 0;
    private final WPI_TalonFX mainShootMotor = new WPI_TalonFX(Constants.MOTOR_SHOOT_MAIN);
    private final CANSparkMax auxiliaryShootMotor = new CANSparkMax(Constants.MOTOR_SHOOT_AUXILIARY, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final WPI_VictorSPX ballTransferMotor = new WPI_VictorSPX(Constants.MOTOR_SHOOT_TRANSFER);
    private final CANSparkMax rotateMotor = new CANSparkMax(Constants.MOTOR_SHOOT_ROTATE, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax angleMotor = new CANSparkMax(Constants.MOTOR_SHOOT_ANGLE, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final PIDController pid_Rotate = new PIDController(Constants.PID_SHOOT_ROTATE[0], Constants.PID_SHOOT_ROTATE[1], Constants.PID_SHOOT_ROTATE[2]);
    private final PIDController pid_Rotate_Fast = new PIDController(Constants.PID_SHOOT_ROTATE_FAST[0], Constants.PID_SHOOT_ROTATE_FAST[1], Constants.PID_SHOOT_ROTATE_FAST[2]);
    private final PIDController pid_Angle = new PIDController(Constants.PID_SHOOT_ANGLE[0], Constants.PID_SHOOT_ANGLE[1], Constants.PID_SHOOT_ANGLE[2]);
    private final NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");
    private boolean autoAlignment = true;
    private final DriveSubsystem driveSubsystem;

    public ShootSubsystem(DriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;

        mainShootMotor.configFactoryDefault();
        auxiliaryShootMotor.restoreFactoryDefaults();
        rotateMotor.restoreFactoryDefaults();
        angleMotor.restoreFactoryDefaults();
        ballTransferMotor.configFactoryDefault();

        mainShootMotor.setInverted(Constants.MOTOR_SHOOT_MAIN_INVERTED);
        auxiliaryShootMotor.setInverted(Constants.MOTOR_SHOOT_AUXILIARY_INVERTED);
        rotateMotor.setInverted(Constants.MOTOR_SHOOT_ROTATE_INVERTED);
        angleMotor.setInverted(Constants.MOTOR_SHOOT_ANGLE_INVERTED);
        ballTransferMotor.setInverted(Constants.MOTOR_SHOOT_TRANSFER_INVERTED);

        mainShootMotor.setNeutralMode(NeutralMode.Coast);
        auxiliaryShootMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
        rotateMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        angleMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        ballTransferMotor.setNeutralMode(NeutralMode.Coast);

        mainShootMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 39, 0, 0));
        auxiliaryShootMotor.setSmartCurrentLimit(39);
        rotateMotor.setSmartCurrentLimit(39);
        angleMotor.setSmartCurrentLimit(39);

        mainShootMotor.config_kP(0, Constants.PID_SHOOT_MAIN[0]);
        mainShootMotor.config_kI(0, Constants.PID_SHOOT_MAIN[1]);
        mainShootMotor.config_kD(0, Constants.PID_SHOOT_MAIN[2]);
        mainShootMotor.config_kF(0, Constants.F_SHOOT_MAIN);
        auxiliaryShootMotor.getPIDController().setP(Constants.PID_SHOOT_AUXILIARY[0]);
        auxiliaryShootMotor.getPIDController().setI(Constants.PID_SHOOT_AUXILIARY[1]);
        auxiliaryShootMotor.getPIDController().setD(Constants.PID_SHOOT_AUXILIARY[2]);
        auxiliaryShootMotor.getPIDController().setFF(Constants.F_SHOOT_AUXILIARY);

        angleMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 24);
        angleMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, 0);
        angleMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
        angleMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, false);

        mainShootMotor.overrideLimitSwitchesEnable(false);
    }

    @Override
    public void periodic() {
        if (Constants.DEBUG){
            SmartDashboard.putBoolean("ShootSubSystem Zeroing", zeroing);
            SmartDashboard.putBoolean("ShootSubsystem Limit switch", mainShootMotor.isRevLimitSwitchClosed() == 0);
            SmartDashboard.putNumber("ShootSubsystem Angle Encoder", angleMotor.getEncoder().getPosition());
            SmartDashboard.putBoolean("ShootSubsystem Has Target", hasTarget);
            SmartDashboard.putNumber("ShootSubsystem Distance", targetDistance);
        }
        if (zeroing) { // zeroing mode
            rotateMotor.stopMotor();
            if (mainShootMotor.isRevLimitSwitchClosed() == 0) {
                zeroing = false;
                angleMotor.set(0);
                angleMotor.getEncoder().setPosition(0);
                angleMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
            } else {
                if (angleMotor.get() == 0) {
                    double zeroSpeed = 0.3;
                    angleMotor.set(-zeroSpeed);
                }
            }
        } else { // normal mode
            if (autoAlignment) {
                hasTarget = limelight.getEntry("tv").getDouble(0) >= 1.0;
                if (hasTarget) {
                    double tx = limelight.getEntry("tx").getDouble(0);
                    double ty = limelight.getEntry("ty").getDouble(0);

                    double angleToGoalRadians = Math.toRadians(Constants.AUTO_ALIGNMENT_MOUNT_ANGLE + ty);
                    targetDistance = (Constants.AUTO_ALIGNMENT_GOAL_HEIGHT_METER - Constants.AUTO_ALIGNMENT_LENS_HEIGHT_METER) / Math.tan(angleToGoalRadians);


                    double currentRotationOutput = pid_Rotate.calculate(-tx);
                    currentRotationOutput += pid_Rotate_Fast.calculate(driveSubsystem.getTurnRate());
                    rotateMotor.set(currentRotationOutput);
                    // PID閉環控制仰角
                    double currentAngleOutput = pid_Angle.calculate(ty);
                    currentAngleOutput = MathUtil.clamp(currentAngleOutput, -0.3, 0.3);
                    //angleMotor.set(currentAngleOutput);
                } else {
                    rotateMotor.stopMotor();
                    angleMotor.stopMotor();
                }
            }
            if (shoot){
                mainShootMotor.set(ControlMode.Velocity, 4000 / 600.0 / Constants.SHOOT_GEARING * 2048.0);
                auxiliaryShootMotor.getPIDController().setReference(3500, CANSparkMax.ControlType.kVelocity);
            }else{
                mainShootMotor.stopMotor();
                auxiliaryShootMotor.stopMotor();
            }
        }
    }

    public void zero() {
        zeroing = true;
        angleMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, false);
    }

    public void enableShootMotor() {
        shoot = true;
    }

    public void disableShootMotor() {
        shoot = false;
    }

    public void setTransferMotorSpeed(double speedInPercent) {
        ballTransferMotor.set(speedInPercent);
    }

    public void stopTransferMotor() {
        ballTransferMotor.stopMotor();
    }

    public void enableAutoAlignment() {
        autoAlignment = true;
    }

    public void disableAutoAlignment() {
        autoAlignment = false;
    }

    public boolean isAutoAlignmentEnabled() {
        return autoAlignment;
    }

    public double getShootMotorVelocity() {
        return mainShootMotor.getSelectedSensorVelocity() * 600.0 / 2048.0 * 3;
    }

    public boolean isZeroing() {
        return zeroing;
    }
}

