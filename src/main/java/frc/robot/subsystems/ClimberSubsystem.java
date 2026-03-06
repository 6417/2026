package frc.robot.subsystems;

import java.awt.Robot;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.signals.GainSchedBehaviorValue;
import com.ctre.phoenix6.signals.GainSchedKpBehaviorValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.fridowpi.motors.FridoFalcon500v6;
import frc.fridowpi.motors.FridoServoMotor;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class ClimberSubsystem extends SubsystemBase {
    private final FridoFalcon500v6 climberMotor;
    private final Servo servoHatchet;
    private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0.0);
    private final MotorOutputConfigs motorConfigs = new MotorOutputConfigs();
    private final ClosedLoopGeneralConfigs closedLoopGeneralConfigs = new ClosedLoopGeneralConfigs();
    private final Slot0Configs motionMagicSlot1 = new Slot0Configs();
    private final Slot1Configs motionMagicSlot2 = new Slot1Configs();

    public boolean isHatchetEngaged = false;

    public ClimberSubsystem() {
        // Initialize motor and apply configuration from Constants.
        servoHatchet = new FridoServoMotor(0);
        climberMotor = new FridoFalcon500v6(Constants.Climber.motorId);
        climberMotor.setInverted(Constants.Climber.motorInverted);
        climberMotor.setIdleMode(Constants.Climber.idleMode);

        // Configure Motion Magic and PID slots (extend/retract).
        reconfigure();

        // Start with a known encoder reference.
        climberMotor.setEncoderPosition(Constants.Climber.resetEncoderPosition);

        servoHatchet.setBoundsMicroseconds(2200, 1499, 1500, 1501, 800);
    }

    @Override
    public void periodic() {
        Logger.recordOutput("/Climber/ServoStatusAngle", servoHatchet.getAngle());
        Logger.recordOutput("/Climber/HatchetEngaged", isHatchetEngaged);
        Logger.recordOutput("/Climber/ClimberEncoderTicks", climberMotor.getEncoderTicks());
    }

    public void setManualPercent(double percent) {
        // Manual open-loop control for testing or encoder zeroing.
        climberMotor.set(MathUtil.clamp(percent, -1.0, 1.0));
    }

    public void stop() {
        climberMotor.stopMotor();
    }

    public void resetEncoder() {
        // Reset encoder position to the configured zero.
        climberMotor.setEncoderPosition(Constants.Climber.resetEncoderPosition);
    }

    public double getAmperage() {
        // Used for stall detection during zeroing.
        return climberMotor.asTalonFX().getStatorCurrent().getValueAsDouble();
    }

    public void setPositionTop() {
        // Use slot 0 for extend (out) with outward motion constraints.
        motionMagicRequest.Position = Constants.Climber.highPosition;
        motionMagicRequest.Slot = 1;
        climberMotor.asTalonFX().setControl(motionMagicRequest);
    }

    public boolean isClimberAtPosition(double position) {
        return (climberMotor.getEncoderTicks() >= position);
    }

    public boolean isMotorBlockedDetectionByVelocity(double velocityThreshold) {
        return (climberMotor.asTalonFX().getVelocity().getValueAsDouble() < velocityThreshold);
    }

    public boolean isMotorBlockedDetectionByAmperage(double threshold) {
        return climberMotor.getAppliedAmps() >= threshold;
    }

    public void startHoming() {
        climberMotor.set(Constants.Climber.homingSpeed);
    }
    
    public void endHoming() {
        stop();
        climberMotor.setEncoderPosition(0);
    }
    
    public void enableServoHatchet() {
        servoHatchet.setAngle(85);
        System.out.println(" ############# Enabled Servo Hatchet ###############");
        isHatchetEngaged = true;
        // servo engaged, climber cannot move anymore
    }

    public void disableServoHatchet() {
        System.out.println(" ############# Disabled Servo Hatchet ###############");
        servoHatchet.setAngle(115);
        isHatchetEngaged = false;
        // the climber can now move freely
    }

    private void reconfigure() {
        motionMagicSlot1.kP = Constants.Climber.kPSlot1;
        motionMagicSlot1.kI = Constants.Climber.kISlot1;
        motionMagicSlot1.kD = Constants.Climber.kDSlot1;
        motionMagicSlot1.kS = Constants.Climber.kSSlot1;
        motionMagicSlot1.kV = Constants.Climber.kVSlot1;
        motionMagicSlot1.kA = Constants.Climber.kASlot1;
        motionMagicSlot1.kG = Constants.Climber.kGSlot1;
        motionMagicSlot1.GainSchedBehavior = GainSchedBehaviorValue.UseSlot2;
        motionMagicSlot1.GravityType = GravityTypeValue.Elevator_Static;
        motionMagicSlot1.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;

        motionMagicSlot2.kP = Constants.Climber.kPSlot2;
        motionMagicSlot2.kI = Constants.Climber.kISlot2;
        motionMagicSlot2.kD = Constants.Climber.kDSlot2;
        motionMagicSlot2.kS = Constants.Climber.kSSlot2;
        motionMagicSlot2.kV = Constants.Climber.kVSlot2;
        motionMagicSlot2.kA = Constants.Climber.kASlot2;
        motionMagicSlot2.kG = Constants.Climber.kGSlot2;
        motionMagicSlot2.GainSchedBehavior = GainSchedBehaviorValue.Inactive;
        motionMagicSlot2.GravityType = GravityTypeValue.Elevator_Static;
        motionMagicSlot2.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;

        closedLoopGeneralConfigs.GainSchedErrorThreshold = Constants.Climber.gainSchedErrorThreshold;
        closedLoopGeneralConfigs.GainSchedKpBehavior = GainSchedKpBehaviorValue.Continuous;

        climberMotor.asTalonFX().getConfigurator().apply(motionMagicSlot1);
        climberMotor.asTalonFX().getConfigurator().apply(motionMagicSlot2);
        climberMotor.asTalonFX().getConfigurator().apply(closedLoopGeneralConfigs);
    }
}
