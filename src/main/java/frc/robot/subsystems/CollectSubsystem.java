package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class CollectSubsystem extends SubsystemBase {
    private final DoubleSolenoid solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.SOLENOID_INTAKE_FORWARD, Constants.SOLENOID_INTAKE_REVERSE);
    private final WPI_TalonSRX intakeMotor = new WPI_TalonSRX(Constants.MOTOR_INTAKE);
    public CollectSubsystem() {
        intakeMotor.configFactoryDefault();
        intakeMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 39, 0,0));
        intakeMotor.setInverted(Constants.MOTOR_INTAKE_INVERTED);
    }

    public void enableIntake(double speed){
        solenoid.set(DoubleSolenoid.Value.kForward);
        intakeMotor.set(speed);
    }
    public void disableIntake(){
        solenoid.set(DoubleSolenoid.Value.kReverse);
        intakeMotor.set(0);
    }
}

