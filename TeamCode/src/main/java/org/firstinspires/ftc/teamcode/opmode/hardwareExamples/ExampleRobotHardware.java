package org.firstinspires.ftc.teamcode.opmode.hardwareExamples;

import org.firstinspires.ftc.teamcode.opmode.RobotHardware;

import java.util.ArrayList;

/**
 * A class that has all of the bare minimum requirements you need to create a robotHardware extension.
 * As the naming convention for these classes is [Name]RobotHardware, you can copy this code in
 *   and replace every occurence of the word Example with that [Name]
**/
public abstract class ExampleRobotHardware extends RobotHardware {
    public enum ExampleMotorName{
        DRIVE_FRONT_LEFT ("Drive_Front_Left"),
        DRIVE_FRONT_RIGHT ("Drive_Front_Right"),
        DRIVE_BACK_LEFT ("Drive_Back_Left"),
        DRIVE_BACK_RIGHT ("Drive_Back_Right");

        private String myName;
        private MotorName myMotorName;
        ExampleMotorName(String name) {
            myName = name;
            myMotorName = new RobotHardware.MotorName(name);
        }
        String getName()
        {
            return myName;
        }
        MotorName getMotorName()
        {
            return myMotorName;
        }

    }
    public enum ExampleServoName
    {
        JEWEL_DROP ("Servo_Jewel_Drop"),
        JEWEL_HIT ("Servo_Jewel_Hit");
        private String myName;
        private ServoName myServoName;
        ExampleServoName(String name)
        {
            myName = name;
            myServoName = new RobotHardware.ServoName(name);
        }
        String getName()
        {
            return myName;
        }
        ServoName getServoName()
        {
            return myServoName;
        }
    }
    @Override
    public ArrayList<MotorName> getMotors() {
        ArrayList<MotorName> names = new ArrayList<MotorName>();
        for(ExampleRobotHardware.ExampleMotorName name : ExampleRobotHardware.ExampleMotorName.values())
        {
            names.add(name.getMotorName());
        }
        return names;
    }

    @Override
    public ArrayList<ServoName> getServos() {
        ArrayList<ServoName> names = new ArrayList<ServoName>();
        for(ExampleRobotHardware.ExampleServoName name : ExampleRobotHardware.ExampleServoName.values())
        {
            names.add(name.getServoName());
        }
        return names;
    }
}
