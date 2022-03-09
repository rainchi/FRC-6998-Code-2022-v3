package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShootSubsystem extends SubsystemBase {
    private boolean shoot = false;
    private double speedInRPM = -1;
    private double overrideSpeed = -2;
    private boolean hasTarget = false;
    private double targetDistance = 0;
    private double targetVelocity = 0;
    private double baseSpeed = 2000;
    private double xOffset = 0;
    private final WPI_TalonFX mainShootMotor = new WPI_TalonFX(Constants.MOTOR_SHOOT_MAIN);
    private final CANSparkMax auxiliaryShootMotor = new CANSparkMax(Constants.MOTOR_SHOOT_AUXILIARY, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final WPI_VictorSPX ballTransferMotor = new WPI_VictorSPX(Constants.MOTOR_SHOOT_TRANSFER);
    private final CANSparkMax rotateMotor = new CANSparkMax(Constants.MOTOR_SHOOT_ROTATE, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final PIDController pid_Rotate = new PIDController(Constants.PID_SHOOT_ROTATE[0], Constants.PID_SHOOT_ROTATE[1], Constants.PID_SHOOT_ROTATE[2]);
    private final PIDController pid_Rotate_Fast = new PIDController(Constants.PID_SHOOT_ROTATE_FAST[0], Constants.PID_SHOOT_ROTATE_FAST[1], Constants.PID_SHOOT_ROTATE_FAST[2]);
    private final NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");
    private boolean autoAlignment = true;
    private boolean forceDisableAlignment = false;
    private final DriveSubsystem driveSubsystem;
    private final ColorSensorV3 colorSensor = new ColorSensorV3(I2C.Port.kMXP);
    private final ColorMatch colorMatcher = new ColorMatch();
    private DriverStation.Alliance alliance = DriverStation.Alliance.Invalid;

    public ShootSubsystem(DriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;

        mainShootMotor.configFactoryDefault();
        auxiliaryShootMotor.restoreFactoryDefaults();
        rotateMotor.restoreFactoryDefaults();
        ballTransferMotor.configFactoryDefault();

        mainShootMotor.configClosedloopRamp(0.01);
        auxiliaryShootMotor.setClosedLoopRampRate(0.01);
        rotateMotor.setClosedLoopRampRate(0.01);
        ballTransferMotor.configClosedloopRamp(0.01);

        mainShootMotor.configOpenloopRamp(0.01);
        auxiliaryShootMotor.setClosedLoopRampRate(0.01);
        rotateMotor.setOpenLoopRampRate(0.01);
        ballTransferMotor.configOpenloopRamp(0.01);

        mainShootMotor.setInverted(Constants.MOTOR_SHOOT_MAIN_INVERTED);
        auxiliaryShootMotor.setInverted(Constants.MOTOR_SHOOT_AUXILIARY_INVERTED);
        rotateMotor.setInverted(Constants.MOTOR_SHOOT_ROTATE_INVERTED);
        ballTransferMotor.setInverted(Constants.MOTOR_SHOOT_TRANSFER_INVERTED);

        mainShootMotor.setNeutralMode(NeutralMode.Coast);
        auxiliaryShootMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
        rotateMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        ballTransferMotor.setNeutralMode(NeutralMode.Coast);

        mainShootMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 35, 0, 0));
        auxiliaryShootMotor.setSmartCurrentLimit(20, 20);
        rotateMotor.setSmartCurrentLimit(25);

        mainShootMotor.config_kP(0, Constants.PID_SHOOT_MAIN[0]);
        mainShootMotor.config_kI(0, Constants.PID_SHOOT_MAIN[1]);
        mainShootMotor.config_kD(0, Constants.PID_SHOOT_MAIN[2]);
        mainShootMotor.config_kF(0, Constants.F_SHOOT_MAIN);

        auxiliaryShootMotor.getPIDController().setP(Constants.PID_SHOOT_AUXILIARY[0]);
        auxiliaryShootMotor.getPIDController().setI(Constants.PID_SHOOT_AUXILIARY[1]);
        auxiliaryShootMotor.getPIDController().setD(Constants.PID_SHOOT_AUXILIARY[2]);
        auxiliaryShootMotor.getPIDController().setFF(Constants.F_SHOOT_AUXILIARY);

        mainShootMotor.overrideLimitSwitchesEnable(false);

        colorMatcher.addColorMatch(Constants.BALL_RED);
        colorMatcher.addColorMatch(Constants.BALL_BLUE);
    }

    @Override
    public void periodic() {
        Color detectedColor = colorSensor.getColor();
        ColorMatchResult colorResult = colorMatcher.matchClosestColor(detectedColor);
        if (Constants.DEBUG) {
            SmartDashboard.putBoolean("ShootSubsystem Limit switch", mainShootMotor.isRevLimitSwitchClosed() == 0);
            SmartDashboard.putBoolean("ShootSubsystem Has Target", hasTarget);
            SmartDashboard.putNumber("ShootSubsystem Distance", targetDistance);
            SmartDashboard.putNumber("ShootSubsystem Velocity", getShootMotorVelocity());
            SmartDashboard.putNumber("Red", detectedColor.red);
            SmartDashboard.putNumber("Green", detectedColor.green);
            SmartDashboard.putNumber("Blue", detectedColor.blue);
            SmartDashboard.putNumber("Ball confidence", colorResult.confidence);
        }
        if (forceDisableAlignment){
            if (shoot){
                mainShootMotor.set(ControlMode.Velocity, 3500 / 600.0 / Constants.SHOOT_GEARING * 2048.0);
                auxiliaryShootMotor.getPIDController().setReference(2500, CANSparkMax.ControlType.kVelocity);
            }else{
                mainShootMotor.stopMotor();
                auxiliaryShootMotor.stopMotor();
            }
            if (autoAlignment){
                rotateMotor.stopMotor();
            }
            return;
        }
        if (autoAlignment) {
            hasTarget = limelight.getEntry("tv").getDouble(0) >= 1.0;
            if (hasTarget) {
                double tx = limelight.getEntry("tx").getDouble(0)+xOffset;
                double ty = limelight.getEntry("ty").getDouble(0);

                double angleToGoalRadians = Math.toRadians(Constants.AUTO_ALIGNMENT_MOUNT_ANGLE + ty);
                targetDistance = (Constants.AUTO_ALIGNMENT_GOAL_HEIGHT_METER - Constants.AUTO_ALIGNMENT_LENS_HEIGHT_METER) / Math.tan(angleToGoalRadians);

                double currentRotationOutput = 0;
                currentRotationOutput += pid_Rotate.calculate(-tx);
                currentRotationOutput += pid_Rotate_Fast.calculate(driveSubsystem.getTurnRate());
                rotateMotor.set(currentRotationOutput);
            } else {
                rotateMotor.stopMotor();
            }
        } else if (overrideSpeed != -2) {
            rotateMotor.set(overrideSpeed);
        }
        if (shoot) {
            double rpm = baseSpeed+ targetDistance * 500;
            if (speedInRPM != -1) {
                rpm = speedInRPM;
            }
//            if (colorResult.color == Constants.BALL_BLUE && colorResult.confidence >= 0.9) {
//                //blue ball
//                if (Constants.DEBUG) {
//                    SmartDashboard.putString("Ball Color", "Blue");
//                }
//                if (alliance != DriverStation.Alliance.Blue) {
//                    rpm = 500;
//                }
//            } else if (colorResult.color == Constants.BALL_RED && colorResult.confidence >= 0.9) {
//                // red ball
//                if (Constants.DEBUG) {
//                    SmartDashboard.putString("Ball Color", "Red");
//                }
//                if (alliance != DriverStation.Alliance.Red) {
//                    rpm = 500;
//                }
//            } else {
//                if (Constants.DEBUG) {
//                    SmartDashboard.putString("Ball Color", "Unknown");
//                }
//                rpm = speedInRPM;
//            }
            rpm = MathUtil.clamp(rpm, 500, 6000);
            targetVelocity = rpm;
            mainShootMotor.set(ControlMode.Velocity, rpm / 600.0 / Constants.SHOOT_GEARING * 2048.0);
            auxiliaryShootMotor.getPIDController().setReference(2500, CANSparkMax.ControlType.kVelocity);
        } else {
            mainShootMotor.stopMotor();
            auxiliaryShootMotor.stopMotor();
        }
    }

    public void enableShootMotor() {
        shoot = true;
        this.speedInRPM = -1;
    }

    public void enableShootMotor(double speedInRPM) {
        shoot = true;
        this.speedInRPM = speedInRPM;
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
        return mainShootMotor.getSelectedSensorVelocity() * 600.0 / 2048.0 * Constants.SHOOT_GEARING;
    }

    public double getTargetVelocity() {
        if (forceDisableAlignment){
            return 3500;
        }
        return targetVelocity;
    }

    public void setAlliance(DriverStation.Alliance alliance) {
        this.alliance = alliance;
    }

    public void enableLimelightGreenLED() {
        limelight.getEntry("ledMode").setNumber(1);
        limelight.getEntry("ledMode").setNumber(3);
    }

    public void disableLimeLightGreenLED() {
        limelight.getEntry("ledMode").setNumber(3);
        limelight.getEntry("ledMode").setNumber(1);
    }

    public void overrideRotate(double speed) {
        autoAlignment = false;
        rotateMotor.set(speed);
        overrideSpeed = speed;
    }

    public void cancelOverrideRotate() {
        autoAlignment = true;
        overrideSpeed = -2;
    }

    public void setBaseSpeed (double speed){
        baseSpeed = speed;
    }

    public void setXOffset(double offset){
        xOffset = offset;
    }

    public double getTargetDistance() {
        return targetDistance;
    }

    public void forceDisableAlignment(boolean enable){
        forceDisableAlignment = enable;
    }

    public boolean isForceDisableAlignment(){
        return forceDisableAlignment;
    }
}

