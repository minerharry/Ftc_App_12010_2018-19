package org.firstinspires.ftc.teamcode.opmode.hardwareExamples;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.opmode.RobotHardware;

import java.util.ArrayList;

/**
 * A class that has all of the bare minimum requirements you need to create a robotHardware extension.
 * As the naming convention for these classes is [Name]RobotHardware, you can copy this code in
 * and replace every occurence of the word Example with that [Name]
 **/
public abstract class ExampleRobotHardware extends RobotHardware {

    /**Hardware Enums**/
    /*
        The following two enums are examples of Hardware Enumerators you can use in your extension of RobotHardware to
        set up the hardware and confirm that your hardware object names are always correct for your config files;
        as long as you reference the enum name (and necessary .get[Object]Name()) you can change the String input
        to each enumerator object and it will change it for the whole code
     */
    //Motor name class for all motors
    public enum ExampleMotorName{
        DRIVE_FRONT_LEFT ("Drive_Front_Left"),
        DRIVE_FRONT_RIGHT ("Drive_Front_Right"),
        DRIVE_BACK_LEFT ("Drive_Back_Left"),
        DRIVE_BACK_RIGHT ("Drive_Back_Right");

        private String myName;
        private MotorName myMotorName;
        ExampleMotorName(String name) {
            myName = name;
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

    //Servo Name enum for all servos
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


    //Any extra objects can go here; as long as there is an appropriate [Object]Name class in RobotHardware, you can use the above model
    //and replace any occurrence of the word Motor/Servo with the [Object]'s name from the RobotHardware class

    /** Alternative Xml-Based Hardware Enums **/
    /*
        These are a set of alternate versions of the two enumerators above that use an Xml-Based system instead of
        simply defining each name in the class. The functionality of the alternate enum should be identical, except for the fact
        that the myName + my[Object]Name fields will not be defined until the function initRobot[Object]s(HardwareMap m) is called,
        which should be put in the init function (overriden example below) for all hardware enums with defined values. Although
        at the time of writing this there does not seem to be any reason or ability to use those fields before the init function / statically,
        be warned that it will cause an error. The argument to each enum is now the R id for the xml string value containing the name
        of the hardware object, which should be in the form of R.string.[stringName]. Make sure that the R object that you import
        is org.firstinspires.ftc.teamcode.R (example import statement below) so that you are referencing the values folder in TeamCode.
     */

    /**Example import statement:
     * import org.firstinspires.ftc.teamcode.R;
     */
    /**
     * Example enums:
     *
    public enum ExampleMotorName {
        DRIVE_FRONT_LEFT(R.string.backLeft),
        DRIVE_FRONT_RIGHT(R.string.backRight),
        DRIVE_BACK_LEFT(R.string.frontLeft),
        DRIVE_BACK_RIGHT(R.string.frontRight);
        private int myNameID;
        private String myName;
        private MotorName myMotorName;

        ExampleMotorName(int nameID) {
            myNameID = nameID;
        }

        public void initRobot(HardwareMap map) {
            myName = map.appContext.getResources().getString(myNameID);
            myMotorName = new RobotHardware.MotorName(myName);
        }

        public static void initRobotMotors(HardwareMap map) {
            for (ExampleMotorName name : ExampleMotorName.values()) {
                name.initRobot(map);
            }
        }

        String getName() {
            if (myName == null) {
                throw new NullPointerException("Error: " + this.getDeclaringClass().toString() + " Exception - Name not initialized from XML, make sure initRobot[Object]s() method was called during init()");
            }
            return myName;
        }

        MotorName getMotorName() {
            if (myName == null) {
                throw new NullPointerException("Error: " + this.getDeclaringClass().toString() + " Exception - Name not initialized from XML, make sure initRobot[Object]s() method was called during init()");
            }
            return myMotorName;
        }
    }

    public enum ExampleServoName {
        J(R.string.intakeServoLeft);
        private int myNameID;
        private String myName;
        private ServoName myServoName;

        ExampleServoName(int nameID) {
            myNameID = nameID;
        }

        public void initRobot(HardwareMap map) {
            myName = map.appContext.getResources().getString(myNameID);
            myServoName = new ServoName(myName);
        }

        String getName() {
            if (myName == null) {
                throw new NullPointerException("Error: " + this.getDeclaringClass().toString() + " Exception - Name not initialized from XML, make sure initRobot[Object]s() method was called during init()");
            }
            return myName;
        }

        ServoName getServoName() {
            if (myName == null) {
                throw new NullPointerException("Error: " + this.getDeclaringClass().toString() + " Exception - Name not initialized from XML, make sure initRobot[Object]s() method was called during init()");
            }
            return myServoName;
        }

        public static void initRobotServos(HardwareMap map) {
            for (ExampleServoName name : ExampleServoName.values()) {
                name.initRobot(map);
            }
        }
    }
    **/

    /**
     * Example Init Method
     **/

    @Override
    public void init() {
        //super.init();
        //ExampleMotorName.initRobotServos(hardwareMap);
        //ExampleServoName.initRobotServos(hardwareMap);
    }


    /**
     * Hardware Access Classes
     **/
    /*
        Because the hardware enumerators are specific to each game/extending class, there is one accessor method for each function
        that returns for the use of RobotHardware the list of all of the Motor/Servo/ObjectNames that are stored in the enumerator
        objects themselves. If you want the hardware objects of that type to be usable, you need one method following the pattern of the below
        methods. To make a new method for a non-prewritten hardware type, you can simply replace every occurrence of the word Motor/Servo
        with whatever hardware type word, as long as all of the other enums/functions are provided. If the hardware type is not supported
        by robot hardware, you'll have to do the same thing to the existing functions as described in the class
     */
    @Override
    public ArrayList<MotorName> getMotors() {
        ArrayList<MotorName> names = new ArrayList<MotorName>();
        for (ExampleRobotHardware.ExampleMotorName name : ExampleRobotHardware.ExampleMotorName.values()) {
            names.add(name.getMotorName());
        }
        return names;
    }

    @Override
    public ArrayList<ServoName> getServos() {
        ArrayList<ServoName> names = new ArrayList<ServoName>();
        for (ExampleRobotHardware.ExampleServoName name : ExampleRobotHardware.ExampleServoName.values()) {
            names.add(name.getServoName());
        }
        return names;
    }
}
