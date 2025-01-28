package frc.robot.util;


import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;

public final class ModeSwitchHandler {
    public interface ModeSwitchInterface {
        /**
         * this method will trigger whenever the robot is enabled, or switches mode (ie from autonomous to teleoperated) 
         */
        default void onModeSwitch(){}

        /**
         * this method will trigger whenever the robot is disabled
         */
        default void onDisable(){}
    }

    public static void EnableModeSwitchHandler(ModeSwitchInterface... modeSwitches) {

        RobotModeTriggers.disabled().onTrue(Commands.runOnce(() -> {
            for(var index : modeSwitches){
                index.onDisable();
            }
        }).ignoringDisable(true));

        RobotModeTriggers.teleop()
            .or(RobotModeTriggers.autonomous())
            .or(RobotModeTriggers.test())
            .onTrue(
                Commands.runOnce(() -> {
                    for(var index : modeSwitches){
                        index.onModeSwitch();
                    }}
                ).ignoringDisable(true)
            );
    }
}
