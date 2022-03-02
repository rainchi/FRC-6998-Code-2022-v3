package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class CollectSubsystem extends SubsystemBase {
    private final DoubleSolenoid solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.SOLENOID_INTAKE_FORWARD, Constants.SOLENOID_INTAKE_REVERSE);
    private final WPI_TalonSRX intakeMotor = new WPI_TalonSRX(Constants.MOTOR_INTAKE);
    private final NetworkTableInstance nt = NetworkTableInstance.getDefault();
    private DriverStation.Alliance alliance;
    private boolean intakeEnabled = false;
    public CollectSubsystem() {
        intakeMotor.configFactoryDefault();
        intakeMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 39, 0,0));
        intakeMotor.setInverted(Constants.MOTOR_INTAKE_INVERTED);
    }

    public void setAlliance(DriverStation.Alliance alliance){
        this.alliance = alliance;
    }

    public void enableIntake(double speed){
        solenoid.set(DoubleSolenoid.Value.kForward);
        intakeMotor.set(speed);
        intakeEnabled = true;
    }
    public void disableIntake(){
        solenoid.set(DoubleSolenoid.Value.kReverse);
        intakeMotor.set(0);
        intakeEnabled = false;
    }

    public double getBallX(){
        double x = -1;
        if (alliance == DriverStation.Alliance.Blue){
            x = nt.getTable("vision/balls").getEntry("bx").getDouble(-1);
            nt.getTable("vision/balls").getEntry("bx").setNumber(-1);
        }else if(alliance == DriverStation.Alliance.Red){
            x =  nt.getTable("vision/balls").getEntry("rx").getDouble(-1);
            nt.getTable("vision/balls").getEntry("rx").setNumber(-1);
        }
        return x;
    }

    public double getBallY(){
        if (alliance == DriverStation.Alliance.Blue){
            return nt.getTable("vision/balls").getEntry("by").getDouble(-1);
        }
        if(alliance == DriverStation.Alliance.Red){
            return nt.getTable("vision/balls").getEntry("ry").getDouble(-1);
        }
        return -1;
    }

    public boolean isIntakeEnabled() {
        return intakeEnabled;
    }
}

