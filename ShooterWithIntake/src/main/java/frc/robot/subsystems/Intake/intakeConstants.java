package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public interface intakeConstants {
    int MOTOR_ID = 0; //TODO: find id
    int OPTICSENSOR_ID =0; //TODO: find id

    // configs constants
    double CURRENT_LIMIT = 40;
    double CURRENT_THRESHOLD = 0.2;
    double TIME_THRESHOLD = 100;
    NeutralModeValue NEUTRALMODE_VALUE = NeutralModeValue.Brake; 

    InvertedValue MOTOR_DIRECTIONS = InvertedValue.CounterClockwise_Positive;

    double INTAKE_CURRENT = 30;
    double FEED_SHOOTER_CURRENT = 30;
    double FEED_SHOOTER_TIME = 0.5;
}
