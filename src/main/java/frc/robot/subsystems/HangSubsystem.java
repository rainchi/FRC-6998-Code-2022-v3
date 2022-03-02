package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class HangSubsystem extends SubsystemBase {
    private boolean zeroing = false;
    private boolean locked = false;
    private final CANSparkMax leftHangMotor = new CANSparkMax(Constants.MOTOR_HANG_LEFT, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax rightHangMotor = new CANSparkMax(Constants.MOTOR_HANG_RIGHT, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax centerHangMotor = new CANSparkMax(Constants.MOTOR_HANG_CENTER, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final DigitalInput leftHangLimit = new DigitalInput(Constants.LIMIT_HANG_LEFT);
    private final DigitalInput rightHangLimit = new DigitalInput(Constants.LIMIT_HANG_RIGHT);
    private final DigitalInput centerHangLimit = new DigitalInput(Constants.LIMIT_HANG_CENTER);
    private final DoubleSolenoid solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.SOLENOID_HANG_FORWARD, Constants.SOLENOID_HANG_REVERSE);
    private final Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);

    public HangSubsystem() {
        // Factory reset motors
        leftHangMotor.restoreFactoryDefaults();
        rightHangMotor.restoreFactoryDefaults();
        centerHangMotor.restoreFactoryDefaults();
        // Set motor's invert flag
        leftHangMotor.setInverted(Constants.MOTOR_HANG_LEFT_INVERTED);
        rightHangMotor.setInverted(Constants.MOTOR_HANG_RIGHT_INVERTED);
        centerHangMotor.setInverted(Constants.MOTOR_HANG_CENTER_INVERTED);
        // Set motor's current draw limit
        leftHangMotor.setSmartCurrentLimit(39);
        rightHangMotor.setSmartCurrentLimit(39);
        centerHangMotor.setSmartCurrentLimit(39);
        // Set motor's neutral mode to brake (prevent robot fall down when disable)
        leftHangMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        rightHangMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        centerHangMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        // Set motor's forward soft limit
        leftHangMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, Constants.SOFT_LIMIT_LEFT_MAX);
        rightHangMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward,  Constants.SOFT_LIMIT_RIGHT_MAX);
        centerHangMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, Constants.SOFT_LIMIT_CENTER_MAX);
        leftHangMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
        rightHangMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
        centerHangMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);

        leftHangMotor.getPIDController().setP(Constants.PID_HANG_LEFT[0]);
        rightHangMotor.getPIDController().setP(Constants.PID_HANG_RIGHT[0]);
        centerHangMotor.getPIDController().setP(Constants.PID_HANG_CENTER[0]);

        enableCompressor();
    }

    @Override
    public void periodic() {
        if (Constants.DEBUG){
            SmartDashboard.putBoolean("HangSubsystem Zeroing", zeroing);
            SmartDashboard.putBoolean("HangSubsystem Left Limit Switch", leftHangLimit.get());
            SmartDashboard.putBoolean("HangSubsystem Right Limit Switch", rightHangLimit.get());
            SmartDashboard.putBoolean("HangSubsystem Center Limit Switch", centerHangLimit.get());
            SmartDashboard.putNumber("HangSubsystem Left Encoder", leftHangMotor.getEncoder().getPosition());
            SmartDashboard.putNumber("HangSubsystem Right Encoder", rightHangMotor.getEncoder().getPosition());
            SmartDashboard.putNumber("HangSubsystem Center Encoder", centerHangMotor.getEncoder().getPosition());
        }
        if(zeroing){ // Zeroing Mode
            boolean success = true;
            if(!leftHangLimit.get()){
                leftHangMotor.set(-0.6);
                success = false;
            }else{
                leftHangMotor.set(0);
                leftHangMotor.getEncoder().setPosition(0);
            }
            if(!rightHangLimit.get()){
                rightHangMotor.set(-0.6);
                success = false;
            }else{
                rightHangMotor.set(0);
                rightHangMotor.getEncoder().setPosition(0);
            }
            if(!centerHangLimit.get()){
                centerHangMotor.set(-1);
                success = false;
            }else{
                centerHangMotor.set(0);
                centerHangMotor.getEncoder().setPosition(0);
            }
            if(success){
                zeroing = false;
            }
        }
    }

    public void zero(){
        zeroing = true;
        leftHangMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, false);
        rightHangMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, false);
        centerHangMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, false);
    }

    public boolean isZeroing(){
        return zeroing;
    }


    public void resetEncoders() {
        leftHangMotor.getEncoder().setPosition(0);
        rightHangMotor.getEncoder().setPosition(0);
        centerHangMotor.getEncoder().setPosition(0);
    }

    public RelativeEncoder getLeftHangEncoder() {
        return leftHangMotor.getEncoder();
    }

    public RelativeEncoder getRightHangEncoder() {
        return rightHangMotor.getEncoder();
    }

    public RelativeEncoder getCenterHangEncoder() {
        return centerHangMotor.getEncoder();
    }

    public void stopAll() {
        leftHangMotor.stopMotor();
        rightHangMotor.stopMotor();
        centerHangMotor.stopMotor();
    }

    public void lockHang() {
        stopAll();
        locked = true;
    }

    public void unlockHang() {
        locked = false;
    }

    public boolean isHangLocked() {
        return locked;
    }

    public void enableCompressor() {
        compressor.enableDigital();
    }

    public void disableCompressor() {
        compressor.disable();
    }

    public boolean isCompressorEnabled() {
        return compressor.enabled();
    }

    public DoubleSolenoid getSolenoid(){
        return solenoid;
    }

    public void stopLeftAndRightHook(){
        leftHangMotor.stopMotor();
        rightHangMotor.stopMotor();
    }

    public void setLeftAndRightHook(double speed){
        double leftSpeed = speed;
        double rightSpeed = speed;
        if (leftHangLimit.get() && leftSpeed<0){
            leftSpeed=0;
        }
        if (rightHangLimit.get() && rightSpeed<0){
            rightSpeed=0;
        }
        leftHangMotor.set(leftSpeed);
        rightHangMotor.set(rightSpeed);
    }

    public void extendSolenoid(){
        solenoid.set(DoubleSolenoid.Value.kForward);
    }

    public void collapseSolenoid(){
        solenoid.set(DoubleSolenoid.Value.kReverse);
    }

    public void stopSolenoid(){
        solenoid.set(DoubleSolenoid.Value.kOff);
    }

    public void setCenterHook(double speed){
        if(centerHangLimit.get() && speed<0){
            speed = 0;
        }
        centerHangMotor.set(speed);
    }

    public void stopCenterHook(){
        centerHangMotor.stopMotor();
    }
}

