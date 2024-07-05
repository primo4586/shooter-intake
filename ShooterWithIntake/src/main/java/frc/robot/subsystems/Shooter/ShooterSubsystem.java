

package frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Intake.IntakeSubsystem;

public class ShooterSubsystem extends SubsystemBase implements shooterConstants{ 
  /** Creates a new ShooterSubsystem. */
  private TalonFX _upMotor;
  private TalonFX _downMotor;
  private MotionMagicVelocityVoltage mm = new MotionMagicVelocityVoltage(0,MOTION_MAGIC_ACCELERATION,true,0.0,0,false,false,false);
  private double targetSpeed = 0;

  public static ShooterSubsystem instance;
  public static ShooterSubsystem getInstance() {
    if (instance == null) {
      instance = new ShooterSubsystem();
    }
    return instance;
  }


  private ShooterSubsystem() {
     _upMotor = new TalonFX(UPMOTOR_ID, Constants.CAN_BUS_NAME);
     _downMotor = new TalonFX(DOWNMOTOR_ID, Constants.CAN_BUS_NAME);

    configs();
  }

  public Command setShooterSpeed(double speed){
    targetSpeed = speed;
    return runOnce(() -> {_upMotor.setControl(mm.withVelocity(speed));
    _downMotor.setControl(mm.withVelocity(speed));});
  }
  
public Command stopMotor(){
  return runOnce(() -> {_upMotor.set(0);
    _downMotor.set(0);});
}

public boolean isAtVelocity(){
  return Math.abs(_upMotor.getVelocity().getValue()-targetSpeed)<MINIMUM_ERROR && Math.abs(_downMotor.getVelocity().getValue()-targetSpeed)<MINIMUM_ERROR;
}

  private void configs(){
    // declaring Configs
        TalonFXConfiguration upConfigs = new TalonFXConfiguration();
        TalonFXConfiguration downConfigs = new TalonFXConfiguration();
        MotionMagicConfigs shooterMM = new MotionMagicConfigs();

        // giving motion magic values
        shooterMM.MotionMagicCruiseVelocity =  MOTION_MAGIC_CRUISE_VELOCITY;
        shooterMM.MotionMagicAcceleration =  MOTION_MAGIC_ACCELERATION;
        shooterMM.MotionMagicJerk =  MOTION_MAGIC_JERK;
        upConfigs.MotionMagic = shooterMM;
        downConfigs.MotionMagic = shooterMM;

        // giving PID values
        upConfigs.Slot0.kP =  UP_KP;
        upConfigs.Slot0.kD =  UP_KD;
        upConfigs.Slot0.kS =  UP_KS;
        upConfigs.Slot0.kV =  UP_KV;
        upConfigs.Slot0.kA =  UP_KA;

        downConfigs.Slot0.kP =  DOWN_KP;
        downConfigs.Slot0.kD =  DOWN_KD;
        downConfigs.Slot0.kS =  DOWN_KS;
        downConfigs.Slot0.kV =  DOWN_KV;
        downConfigs.Slot0.kA =  DOWN_KA;

        // max voltage for m_shooterMotor
        upConfigs.Voltage.PeakForwardVoltage =  PEAK_FORWARD_VOLTAGE;
        upConfigs.Voltage.PeakReverseVoltage =  PEAK_REVERSE_VOLTAGE;

        downConfigs.Voltage.PeakForwardVoltage =  PEAK_FORWARD_VOLTAGE;
        downConfigs.Voltage.PeakReverseVoltage =  PEAK_REVERSE_VOLTAGE;

        upConfigs.Feedback.SensorToMechanismRatio =  GEAR_RATIO;
        downConfigs.Feedback.SensorToMechanismRatio =  GEAR_RATIO;

        upConfigs.CurrentLimits.SupplyCurrentLimit = 40;
        upConfigs.CurrentLimits.SupplyCurrentThreshold = 50;
        upConfigs.CurrentLimits.SupplyTimeThreshold = 0.1;
        upConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;

        downConfigs.CurrentLimits.SupplyCurrentLimit = 40;
        downConfigs.CurrentLimits.SupplyCurrentThreshold = 50;
        downConfigs.CurrentLimits.SupplyTimeThreshold = 0.1;
        downConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;

        _upMotor.setNeutralMode(NeutralModeValue.Coast);
        _downMotor.setNeutralMode(NeutralModeValue.Coast);

        upConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        downConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        /* Speed up signals for better characterization data */
        BaseStatusSignal.setUpdateFrequencyForAll(1000, _upMotor.getVelocity());
        BaseStatusSignal.setUpdateFrequencyForAll(1000, _downMotor.getVelocity());

        _upMotor.optimizeBusUtilization();
        _downMotor.optimizeBusUtilization();

        // Checking if _upMotor apply configs
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = _upMotor.getConfigurator().apply(upConfigs);
            if (status.isOK())
                break;
        }
                
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
