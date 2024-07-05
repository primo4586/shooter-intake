// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import java.lang.constant.Constable;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase implements intakeConstants {
  private TalonFX _motor;
  private DigitalInput _opticSensor;
  private TorqueCurrentFOC currentFOC = new TorqueCurrentFOC(0);
  public static IntakeSubsystem instance;

  public static IntakeSubsystem getInstance() {
    if (instance == null) {
      instance = new IntakeSubsystem();
    }
    return instance;
  }

  /** Creates a new IntakeSubsystem. */
  private IntakeSubsystem() {
    _motor = new TalonFX(MOTOR_ID, Constants.CAN_BUS_NAME);
    _opticSensor = new DigitalInput(OPTICSENSOR_ID);

    configs();
  }

  public Command setCurrent(double current) {
    return runOnce(() -> _motor.setControl(currentFOC.withOutput(current)));
  }

  public boolean getOpticSensorValue() {
    return _opticSensor.get();
  }

  public Command intakeCommand() {
    return runEnd(() -> setCurrent(INTAKE_CURRENT), () -> _motor.stopMotor()).until(() -> getOpticSensorValue());
  }
  public Command feedShooterCommand(){
    return runEnd(() -> setCurrent(FEED_SHOOTER_CURRENT), () -> _motor.stopMotor()).withTimeout(FEED_SHOOTER_TIME);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  private void configs() {
    TalonFXConfiguration motorConfigs = new TalonFXConfiguration();
    motorConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
    motorConfigs.CurrentLimits.SupplyCurrentLimit = CURRENT_LIMIT;
    motorConfigs.CurrentLimits.SupplyCurrentThreshold = CURRENT_THRESHOLD;
    motorConfigs.CurrentLimits.SupplyTimeThreshold = TIME_THRESHOLD;
    motorConfigs.MotorOutput.NeutralMode = NEUTRALMODE_VALUE;
    motorConfigs.MotorOutput.Inverted = MOTOR_DIRECTIONS;

    // upload configs to motor
    StatusCode statusCode = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; i++) {
      _motor.getConfigurator().apply(motorConfigs);
      if (statusCode.isOK())
        break;
    }
    if (!statusCode.isOK())
      System.out.println("Intake could not apply config, error code:" + statusCode.toString());
  }
}
