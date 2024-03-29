package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class CollectSubsystem extends SubsystemBase {
    private final DoubleSolenoid solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.SOLENOID_INTAKE_FORWARD, Constants.SOLENOID_INTAKE_REVERSE);
    private final WPI_TalonSRX intakeMotor = new WPI_TalonSRX(Constants.MOTOR_INTAKE);
    private final NetworkTableInstance nt = NetworkTableInstance.getDefault();
    private boolean intakeEnabled = false;
    public CollectSubsystem() {
        intakeMotor.configFactoryDefault();
        intakeMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 25, 0,0));
        intakeMotor.setInverted(Constants.MOTOR_INTAKE_INVERTED);
    }

    public void enableIntake(boolean reverse){
        solenoid.set(DoubleSolenoid.Value.kForward);
        double speed = reverse?-1:1;
        intakeMotor.set(speed);
        intakeEnabled = true;
    }
    public void disableIntake(){
        solenoid.set(DoubleSolenoid.Value.kReverse);
        intakeMotor.set(0);
        intakeEnabled = false;
    }

    public boolean isIntakeEnabled() {
        return intakeEnabled;
    }
}

