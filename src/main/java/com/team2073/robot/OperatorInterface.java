package com.team2073.robot;

import com.team2073.robot.Command.Drive.DriveCommand;
import com.team2073.robot.Subsystems.Drive.DrivetrainSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

public class OperatorInterface {
    private static ApplicationContext appCTX = ApplicationContext.getInstance();
    private Joystick controller = appCTX.getController();
    private DrivetrainSubsystem drivetrain = appCTX.getDrivetrainSubsystem();


    private JoystickButton a = new JoystickButton(controller, 1);
    private JoystickButton b = new JoystickButton(controller, 2);

    public OperatorInterface(){
    }

    public void init(){
//        b.whileHeld(new DriveCommand(drivetrain, controller));
//        a.whileHeld(new HI());
    }
}
