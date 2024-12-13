package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;

public class Robot_Hardware {

    public DigitalChannel ExtendLimit, LiftDown;

    // Define our Motors
    public DcMotorEx leftFront, leftBack, rightFront, rightBack, motorLiftLeft, motorLiftRight, motorExtendLeft, motorExtendRight;

    // Define your servo's here
    public Servo flipper, claw, wrist, lamps;

    public ColorSensor sensorColor;
    public DistanceSensor sensorDistance;

    public IMU imu;

    // We define local variables to hold objects that belong to the LinearOpMode
    // passed into initHardware()
    // Define local GamePads
    Gamepad gamepad1;
    Gamepad gamepad2;

    Telemetry telemetry;


    // Define a local HardwareMap
    HardwareMap hardwareMap;

    // Save the opMode passed into initHardware
    LinearOpMode opMode;

    // Is this an autonomous OpMode?
    boolean autonomous = false;

    // Is this a teleOp OpMode?
    boolean teleOp = false;

    public void initHardware(LinearOpMode opModeIn) {

        // Determine if this is an autonomous OpMode or teleOp
        autonomous = opModeIn.getClass().getAnnotation(Autonomous.class) != null;
        teleOp = opModeIn.getClass().getAnnotation(TeleOp.class) != null;
        opMode = opModeIn;

        this.hardwareMap = opModeIn.hardwareMap;
        this.telemetry = opModeIn.telemetry;

        initHardware(opModeIn.hardwareMap, opModeIn.gamepad1, opModeIn.gamepad2);
    }

    /**
     * Initialize all of the hardware for the robot
     *
     * @param hardwareMapIn This is a new HardwareMap created in the opMode
     * @param gamepad1In    This is the nbr 1 gamepad Object
     * @param gamepad2In    This is the nbr 2 gamepad Object
     **/
    public void initHardware(HardwareMap hardwareMapIn, Gamepad gamepad1In, Gamepad gamepad2In) {
        // Save reference to Hardware map
        hardwareMap = hardwareMapIn;
        // Save references to the GamePads
        gamepad1 = gamepad1In;
        gamepad2 = gamepad2In;

        // Get our motors from the HardwareMap. This is where configuration
        // values are stored when you configure the robot.
        leftFront = setupDriveMotor("leftFront", DcMotorEx.Direction.REVERSE);
        leftBack = setupDriveMotor("leftBack", DcMotorEx.Direction.REVERSE);
        rightFront = setupDriveMotor("rightFront", DcMotorEx.Direction.FORWARD);
        rightBack = setupDriveMotor("rightBack", DcMotorEx.Direction.FORWARD);
        motorLiftLeft = setupDriveMotor("motorLiftLeft", DcMotorEx.Direction.REVERSE);
        motorLiftRight = setupDriveMotor("motorLiftRight", DcMotorEx.Direction.FORWARD);
        motorExtendLeft = setupDriveMotor("motorExtendLeft", DcMotorEx.Direction.FORWARD);
        motorExtendRight = setupDriveMotor("motorExtendRight", DcMotorEx.Direction.REVERSE);

        //Setup your digital channels
        LiftDown = hardwareMap.get(DigitalChannel.class,"LiftHome");
        //ExtendLimit = hardwareMap.get(DigitalChannel.class,"LiftHome");

        //Color and Distance Sensor are one in the same!
        sensorColor = hardwareMap.get(ColorSensor.class, "ColorSense");
        sensorDistance = hardwareMap.get(DistanceSensor.class, "ColorSense");

        imu = hardwareMap.get(IMU.class, "imu");

        flipper = hardwareMap.servo.get("flipper");
        claw = hardwareMap.servo.get("claw");
        lamps = hardwareMap.servo.get("lamps");
        wrist = hardwareMap.servo.get("wrist");

        motorLiftLeft.setTargetPosition(0);
        motorLiftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLiftLeft.setPower(1);

        motorLiftRight.setTargetPosition(0);
        motorLiftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLiftRight.setPower(1);

        motorExtendLeft.setTargetPosition(0);
        motorExtendLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorExtendLeft.setPower(1);

        motorExtendRight.setTargetPosition(0);
        motorExtendRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorExtendRight.setPower(1);

        telemetry.addLine("P l e a s e   w a i t . . .");
        telemetry.addData("setting up driver motor behaviors...", " ");
        telemetry.update();

        lamps.setPosition(0.5);

        // Tell the motors not to coast. When we quit applying power it should stop.
        setDriveZeroPowerBehaviors(DcMotor.ZeroPowerBehavior.BRAKE);

        // Start with motor speeds of zero
        setDriveMotorSpeeds(0);

        // Retrieve the IMU from the hardware map
        initializeImu();
        imu.resetYaw();

        // Set all hubs to use the AUTO Bulk Caching mode for faster encoder reads
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        telemetry.update();
    }

    private DcMotorEx setupDriveMotor(String deviceName, DcMotorEx.Direction direction) {
        DcMotorEx aMotor = hardwareMap.get(DcMotorEx.class, deviceName);
        aMotor.setDirection(direction);
        if (autonomous) {
            aMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);  // Reset Encoders to zero
        }
        aMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        aMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);  // Requires motor encoder cables to be hooked up.
        return aMotor;
    }

    public void setDriveZeroPowerBehaviors(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        leftFront.setZeroPowerBehavior(zeroPowerBehavior);
        leftBack.setZeroPowerBehavior(zeroPowerBehavior);
        rightFront.setZeroPowerBehavior(zeroPowerBehavior);
        rightBack.setZeroPowerBehavior(zeroPowerBehavior);
        motorLiftLeft.setZeroPowerBehavior(zeroPowerBehavior);
        motorLiftRight.setZeroPowerBehavior(zeroPowerBehavior);
        motorExtendLeft.setZeroPowerBehavior(zeroPowerBehavior);
        motorExtendRight.setZeroPowerBehavior(zeroPowerBehavior);
    }

    public void setDriveMotorSpeeds(double speed) {
        leftFront.setPower(speed);
        leftBack.setPower(speed);
        rightFront.setPower(speed);
        rightBack.setPower(speed);
        motorLiftLeft.setPower(speed);
        motorLiftRight.setPower(speed);
        motorExtendLeft.setPower(speed);
        motorExtendRight.setPower(speed);
    }

    public void initializeImu() {

        // Tell the software how the Control Hub is mounted on the robot to align the IMU XYZ axes correctly
        RevHubOrientationOnRobot orientationOnRobot =
                new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                        RevHubOrientationOnRobot.UsbFacingDirection.LEFT);

        // Set all hubs to use the AUTO Bulk Caching mode for faster encoder reads
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        imu.initialize(new IMU.Parameters(orientationOnRobot));
    }

}
