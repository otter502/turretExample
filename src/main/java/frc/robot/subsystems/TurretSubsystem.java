package frc.robot.subsystems;

import static frc.robot.Constants.kStandardDt;
import static frc.robot.Constants.TurretConstants.kConstraints;
import static frc.robot.Constants.TurretConstants.kFeedForwardCalc;
import static frc.robot.Constants.TurretConstants.kGearRatio;
import static frc.robot.Constants.TurretConstants.kMaxAngle;
import static frc.robot.Constants.TurretConstants.kMinAngle;
import static frc.robot.Constants.TurretConstants.kMotorConfig;
import static frc.robot.Constants.TurretConstants.kMotorID;
import static frc.robot.Constants.TurretConstants.kMotorType;
import static frc.robot.Constants.TurretConstants.kSim;
import static frc.robot.Constants.TurretConstants.kStartingAngle;
import static frc.robot.Constants.TurretConstants.kThreshold;

import java.util.InputMismatchException;
import java.util.Optional;
import java.util.Random;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants.TurretPreset;
import frc.robot.subsystems.MotorSimulationSubsystem.PhysicalSimWrapper;
import frc.robot.subsystems.MotorSimulationSubsystem.SimSparkConstants;
import frc.robot.subsystems.MotorSimulationSubsystem.SimulatedSubsystem;
import frc.robot.util.ModeSwitchHandler.ModeSwitchInterface;

public class TurretSubsystem extends SubsystemBase implements SimulatedSubsystem, ModeSwitchInterface{
    
    //#region variables
    private final SparkMax mMotor;
    private final RelativeEncoder mEncoder;

    private final TrapezoidProfile mMotionProfileCalculator;

    private final SparkClosedLoopController mCLController;


    private Rotation2d mSetpoint;
    private State mCurrState;

    //#endregion

    public TurretSubsystem() {
        super();

        mMotor = new SparkMax(kMotorID, kMotorType);
        mMotor.configure(kMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        mEncoder = mMotor.getEncoder();

        mEncoder.setPosition(convertAngleToRaw(kStartingAngle));

        mMotionProfileCalculator = new TrapezoidProfile(kConstraints);

        mCLController = mMotor.getClosedLoopController();

        resetMechanism();
    }

    public void resetMechanism(){
        var currentPosition = getRawPosition();
        mSetpoint = currentPosition;
        mCurrState = new State(convertAngleToRaw(currentPosition), 0.0);
    }

    //#region Command methods

    private void incrementAngle(Rotation2d delta){
        // the reason we use this instead of .plus() is because .plus() constraints the result to [-pi, pi]
        mSetpoint = Rotation2d.fromDegrees(mSetpoint.getDegrees() + delta.getDegrees());
    }

    private void setSetpoint(Rotation2d newAngle){
        mSetpoint = newAngle;
    }

    private void setMechanismAngle(Rotation2d angle){
        mEncoder.setPosition(convertAngleToRaw(angle));
        resetMechanism();
    }


    //#endregion

    //#region lambda commands
    // these are methods that return commands

    public Command manualMode(Rotation2d delta){
        return this.runEnd(() -> {
            incrementAngle(delta);
        }, () -> {
            resetMechanism();
        });
    }

    public Command setSetpointCommand(Rotation2d newSetpoint){
        return this.runOnce(() -> setSetpoint(newSetpoint));
    }

    public Command presetCommand(TurretPreset preset){
        return setSetpointCommand(preset.presetSetpoint);
    }

    public Command setMechanismAngleCommand(Rotation2d newAngle){
        return this.runOnce(() -> setMechanismAngle(newAngle));
};

    //#endregion

    //#region conditions

    public boolean atTarget(){
        return  MathUtil.isNear(
            mSetpoint.getDegrees(), 
            getRawPosition().getDegrees(), 
            kThreshold.getDegrees(), -180, 180
        );
    }

    //#endregion

    @Override
    public void periodic() {
        mSetpoint = optimizeAngle(mSetpoint, getRawPosition());

        mSetpoint = safetyZoneUpdate(mSetpoint);

        updateMotor();

        updateUserOutputs();

    }

    //#region Periodic methods

    /**
     * this function modifies the setpoint to be safe
     * for this turret it wraps the output, (this assumes the turret can rotate infinitely)
     * @param setpoint unsafe setpoint
     * @return safe setpoint
     */
    private Rotation2d safetyZoneUpdate(Rotation2d setpoint) {
        return Rotation2d.fromDegrees(
            MathUtil.clamp(
                mSetpoint.getDegrees(),
                kMaxAngle.getDegrees(), 
                kMinAngle.getDegrees()
            )
        );
    }

    /**
     * this function runs the motor's closed loop and motion profile update cycle
     */
    private void updateMotor(){
        mCurrState = mMotionProfileCalculator
            .calculate(
                kStandardDt, 
                mCurrState , 
                new State(convertAngleToRaw(mSetpoint), 0.0)
            );

        mCLController.setReference(
            mCurrState.position, 
            ControlType.kPosition, 
            ClosedLoopSlot.kSlot0, 
            kFeedForwardCalc.calculate(mCurrState.velocity)
        );
    }


    /**
     * this function updates the subsystem's logs as well as shuffleboard
     */
    private void updateUserOutputs(){

    }
    
    //#endregion

    //#region interfaces

    @Override
    public void onModeSwitch() {
        resetMechanism();
    }

    @Override
    public Optional<SimSparkConstants[]> getSparkSimConstants() {
        return Optional.of(new SimSparkConstants[]{
            new SimSparkConstants(
                mMotor, 
                new PhysicalSimWrapper(kSim),
                DCMotor.getNEO(1),
                kGearRatio,
                (s) -> {}
            )
        });
    }

    //#endregion

    //#region Utility functions
    /**
     * this function is used to ensure the conversion between the raw encoder value and the angle is consistent
     * @return a rotation2d with 0 degrees facing the front of the robot with positive being CCW
     */
    private Rotation2d getRawPosition(){
        return rawToAngle(mEncoder.getPosition());
    }

    /**
     * this function is used to ensure the conversion between the raw encoder value and the angle is consistent
     * @return a rotation2d with 0 degrees facing the front of the robot with positive being CCW
     */
    private Rotation2d rawToAngle(double angle){
        return Rotation2d.fromRotations(angle);
    }

    /**
     * this should the inverse of {@link #rawToAngle() getPosition()}
     * 
     * @param angle the intended angle of the system
     * @return the raw value to be used for the motor
     */
    private double convertAngleToRaw(Rotation2d angle){
        return angle.getRotations();
    }

    /**
     * This wraps a rotation2d to be between -pi and pi
     */
    private Rotation2d getWrappedAngle(Rotation2d unWrapped){
        return unWrapped.plus(Rotation2d.kZero);
    }

    //#region angle Optimizer (specific to turret)

    /**
     * 
     * @param setpoint the target robot angle (between -pi and pi)
     * @param guess the current angle, the returned angle will be the closest to the guess with the same robot angle
     * @return
     */
    private static Rotation2d optimizeAngle(Rotation2d setpoint, Rotation2d guess){
        var wrapDiffDeg = (getWrappedDiff(setpoint.getDegrees(), guess.getDegrees(), 0, 360));

        var shortDistTar = guess.getDegrees() + wrapDiffDeg; //goto angle via shortest path
        var longDistTar = guess.getDegrees() + wrapDiffDeg - Math.signum(guess.getDegrees()) * 360; //goto angle via longest path

        var maxDeg = kMaxAngle.getDegrees();
        var minDeg = kMinAngle.getDegrees();

        if (shortDistTar >= minDeg && shortDistTar <= maxDeg) {
            return Rotation2d.fromDegrees(shortDistTar);
        } else {
            if (longDistTar >= minDeg && longDistTar <= maxDeg){
                return Rotation2d.fromDegrees(longDistTar);
            } else{
                System.out.println("ERROR");
                return guess;
            }
        }
    }


    private static double getWrappedDiff(double reference, double target, double min, double max){
        reference = MathUtil.inputModulus(reference, min, max);
        target = MathUtil.inputModulus(target, min, max);
        var baseDist = (reference - target);
        if (Math.abs(baseDist) < Math.abs(max - min) / 2) {
            return baseDist;
        }
        
        var refSideDist = Math.min(Math.abs(reference - max), Math.abs(reference - min));
        var tarSideDist = Math.min(Math.abs(target - max), Math.abs(target - min));
        
        if (Math.abs(reference - max) < Math.abs(reference - min)) {
            //closer to the maximum, so the difference is positive
            refSideDist *= 1;
        } else {
            //closer to the minimum side
            refSideDist *= -1;
        }
    
        if (Math.abs(target - max) < Math.abs(target - min)) {
            //closer to the maximum, so the difference is positive
            tarSideDist *= 1;
        } else {
            //closer to the minimum side
            tarSideDist *= -1;
        }
    
    
        return refSideDist - tarSideDist;
    };
    
    //#endregion


    //#endregion

}
