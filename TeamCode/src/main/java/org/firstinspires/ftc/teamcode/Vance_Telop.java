package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;


@TeleOp(name="Driver Mode", group="Robot")
public class Vance_Telop extends LinearOpMode {
    Robot_Hardware robot = new Robot_Hardware();  //Initialize the hardware

    double x =0;

    @Override
    public void runOpMode() throws InterruptedException {
        //hsv Values is an array that will hold the hue, saturation and value information!
        float hsvValues[] = {0F, 0F, 0F};

        //values is a reference to the hsv values array!
        final float values [] = hsvValues;

        // sometimes it helps to multiply the raw RGB values with a scale factor
        // to amplify/attentuate the measured values.
        final double SCALE_FACTOR = 255;

        // get a reference to the RelativeLayout so we can change the background
        // color of the Robot Controller app to match the hue detected by the RGB sensor.
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

        waitForStart();

            robot.lamps.setPosition(0.5);
            sleep(500);
            robot.lamps.setPosition(0);
            sleep(500);
            robot.lamps.setPosition(0.5);
            sleep(500);
            robot.lamps.setPosition(0);

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // This is currently inverted!!

            if (robot.motorLiftLeft.getCurrentPosition() > 1400)
            {
                x = -gamepad1.left_stick_x * 20000; // Counteract imperfect strafing this is front to back weight bias
            }else{

                x = -gamepad1.left_stick_x * 10;
            }

            double rx = gamepad1.right_stick_x * 16; // Invert this if needed!!


            ///////////////////////// Color Detection Control Section ////////////////////////////

            // convert the RGB values to HSV values.
            // multiply by the SCALE_FACTOR.
            // then cast it back to int (SCALE_FACTOR is a double)
            Color.RGBToHSV((int) (robot.sensorColor.red() * SCALE_FACTOR),
                    (int) (robot.sensorColor.green() * SCALE_FACTOR),
                    (int) (robot.sensorColor.blue() * SCALE_FACTOR),
                    hsvValues);



            ///////////////////////// Overdrive Control Section ////////////////////////////


            float Overdrive; // makes robot go fast!!!
            if (gamepad1.right_bumper == false) {
                Overdrive = 0.35F;
            } else {
                Overdrive = 0.8F;
            }
            // ^^^ Gamepad 1





            ///////////////////////// Lifter Section ////////////////////////////

            float Lifta; // lift up
            if (gamepad2.dpad_up == true && robot.motorLiftLeft.getCurrentPosition() <= 2700)
            {
                Lifta = 1F; // Goes up!
            }
            else if (gamepad2.dpad_down == true && (robot.LiftDown.getState() == false))
            {
                Lifta = -1F; //Goes down until limit switch met!
            }
            else if (gamepad2.dpad_down == false && gamepad1.dpad_up == false &&robot.LiftDown.getState() == true)
            {
                Lifta = 0F; // Stops holding power if arm is down!
            }
            else
            {
                Lifta = 0.001F; //Holds in place if arm is up!
            }



            ///////////////////////// Extender Section ////////////////////////////

            float Extenda; // Extends the arm!
            if (gamepad2.dpad_right == true && (robot.motorExtendLeft.getCurrentPosition() <= 1400) && (robot.motorLiftLeft.getCurrentPosition() <1000))
            {
                Extenda = 1F; // Goes up!
            }
            else if (gamepad2.dpad_right == true && (robot.motorExtendLeft.getCurrentPosition() <= 2100) && (robot.motorLiftLeft.getCurrentPosition() >1000))
            {
                Extenda = 1F;
            }
            else if (gamepad2.dpad_left == true && robot.motorExtendLeft.getCurrentPosition() >= 5)
            {
                Extenda = -1F; //goes down
            }
            else if (gamepad2.dpad_right == false && gamepad2.dpad_left == false && robot.LiftDown.getState() == true)
            {
                Extenda = 0F; //No need to hold when arm is down
            }
            else
            {
                Extenda = 0.001F; // Stops and holds in place!
            }

            ///////////////////////// Flipper Section ////////////////////////////


            float flippera = 0;
            if (gamepad2.a) {  // flips the claw down larger value = towards floor default = 0.81
                robot.flipper.setPosition(0.42);
            } if (gamepad2.y) {  // flips the claw up lower value = towards robot default = 0.50
                robot.flipper.setPosition(0);
            }

            ///////////////////////// Claw Section ////////////////////////////


            float clawa = 0;
            if (gamepad2.right_trigger > 0) {  // closes the claw
                robot.claw.setPosition(0);
            } if (gamepad2.left_trigger > 0) {  // opens the claw
                robot.claw.setPosition(0.3);
            }


            ///////////////////////// Color & Lamps Section ////////////////////////////

            if (robot.sensorColor.red() > 300 && (robot.sensorColor.red() > robot.sensorColor.green())) { //sets lamps to red so long as green isnt predominant for yellow blocks
                robot.lamps.setPosition(0.28);
            } else if (robot.sensorColor.blue() > 300) { //sets lamps to blue
                robot.lamps.setPosition(0.611);
            } else if (robot.sensorColor.green() > 300)  //sets lamps to yellow
                robot.lamps.setPosition(0.388);
            else {
                robot.lamps.setPosition(0);
            }

            ///////////////////////// Wrist Section ////////////////////////////

            float wrista = 0;
            if (gamepad2.b) { //rotates the wrist to face the front
                robot.wrist.setPosition(-0.10);
            } if (gamepad2.x) { //rotates the wrist to face the rear
                robot.wrist.setPosition(0.625);
            }

            if ((clawa < 0.1 && wrista < 0.1) && (gamepad2.dpad_left && robot.motorExtendLeft.getCurrentPosition() > 600))
            {
                robot.wrist.setPosition(0);
            }

            ///////////////////////// Servo Variable Output Section ////////////////////////////


            double k;
            {
                k = robot.claw.getPosition();
            }

            double k2;
            {
                k2 = robot.flipper.getPosition();
            }

            double k3;
            {
                k3 = robot.wrist.getPosition();
            }

            ///////////////////////// Display on Station Section ////////////////////////////

            //feedback data below for on driver station!
            telemetry.addData("Distance (cm)",
                    String.format(Locale.US, "%.02f", robot.sensorDistance.getDistance(DistanceUnit.CM)));
            telemetry.addData("Alpha", robot.sensorColor.alpha());
            telemetry.addData("Red  ", robot.sensorColor.red());
            telemetry.addData("Green", robot.sensorColor.green());
            telemetry.addData("Blue ", robot.sensorColor.blue());
            telemetry.addData("Hue", hsvValues[0]);
            telemetry.addData("Overdrive", Overdrive);
            telemetry.addData("Claw", k);
            telemetry.addData("Flipper", k2);
            telemetry.addData("Wrist", k3);
            telemetry.addData("Lift Command", Lifta);
            telemetry.addData("Lifter Left", robot.motorLiftLeft.getCurrentPosition());
            telemetry.addData("Lifter Right", robot.motorLiftRight.getCurrentPosition());
            telemetry.addData("Extender Left", robot.motorExtendLeft.getCurrentPosition());
            telemetry.addData("Extender Right", robot.motorExtendRight.getCurrentPosition());
            telemetry.addData("Lift Down", robot.LiftDown.getState());

            // change the background color to match the color detected by the RGB sensor.
            // pass a reference to the hue, saturation, and value array as an argument
            // to the HSVToColor method.
            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));

                    telemetry.update();

                    ///////////////////////// DC motor driving Section ////////////////////////////


                    // Denominator is the largest motor power (absolute value) or 1
                    // This ensures all the powers maintain the same ratio, but only when
                    // at least one is out of the range [-1, 1]
                    double denominator;
                    {
                        denominator = (Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1));
                    }
                    double frontLeftPower = ((y + x + rx) * Overdrive) / denominator;
                    double RearLeftPower = ((y - x + rx) * Overdrive) / denominator;
                    double frontRightPower = ((y - x - rx) * Overdrive) / denominator;
                    double RearRightPower = ((y + x - rx) * Overdrive) / denominator;
                    double LiftMotorPower = Lifta;
                    double ExtendMotorPower = Extenda;

                    ///////////////////////// DC Motor Power Set Section ////////////////////////////

                    robot.leftFront.setPower(frontLeftPower);
                    robot.leftBack.setPower(RearLeftPower);
                    robot.rightFront.setPower(frontRightPower);
                    robot.rightBack.setPower(RearRightPower);
                    robot.motorLiftLeft.setPower(LiftMotorPower);
                    robot.motorLiftRight.setPower(LiftMotorPower);
                    robot.motorExtendLeft.setPower(ExtendMotorPower);
                    robot.motorExtendRight.setPower(ExtendMotorPower);

                    ///////////////////////// For Fun Section ////////////////////////////

                    telemetry.addData("Status", "RUN BOB RUN!");

                    //telemetry.update();
                }
            });
        }
    }
}
