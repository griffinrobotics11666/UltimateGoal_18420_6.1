/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.HardwareMap;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;


/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Driver Control", group="Iterative Opmode")
//@Disabled
public class DriverControl extends OpMode {
    HardwareRobot robot = new HardwareRobot();

    Orientation angleExpansion;
    Orientation angleControl;



    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime switchTime = new ElapsedTime();



    //set up variables for the button presses
    boolean debug = false, debugChanged = false;
    boolean inPickupPos = true, isXchanged;
    int shootyRotationCount = 0;
    boolean isUpChanged, isDownChanged;
    boolean isOpen = true, isOpenChanged = false;
    boolean isRoofRaised = true, isYchanged= false; // true: arm is up, false: arm is down
    boolean shootyIsRunning = false, shootyIsRunningChanged = false;
    boolean isShooting = false, shootyIsShootingChanged = false;
    boolean isBchanged = false;
    boolean isLBchanged = false;

    double motorPower = 0;

    // Setup a variable for each drive wheel to save power level for telemetry
    double rearLeftPower;
    double rearRightPower;
    double frontLeftPower;
    double frontRightPower;

    //SETS MAX AND MIN POSITIONS FOR SERVOS
    private static final double CLAW_SERVO_OPEN_POS     =  0.55;
    private static final double CLAW_SERVO_CLOSE_POS     =  0.35;

    private static final double SHOOTY_ROTATION_FLAT_POS     =  0.64;
    private static final double SHOOTY_ROTATION_LAUNCH_LOW     =  0.19;
    private static final double SHOOTY_ROTATION_LAUNCH_HIGH = 0.13;
    private static double CALCULATED_SHOOTY_ROTATION_LAUNCH = 0.13;


    private static final double CLAW_ROTATION_SERVO_PICKUP     =  0.35;
    private static final double CLAW_ROTATION_SERVO_DROP     =  0.49;
    private static final double CLAW_ROTATION_WOBBLE       =  0.20;


    private static final double SHOOTY_BOI_SERVO_SHOOT_POS     =  0.64;
    private static final double SHOOTY_BOI_SERVO_LOAD_POS     =  0.47;

    private static final double DEBUG_INCREMENT = 0.0005; //amt to increase servo in debug (slower)

    private enum grabbingState{armUp, dropRing, armDown}
    private grabbingState state = grabbingState.armDown;
    int armDelay = 2000;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        robot.init(hardwareMap);

        robot.imuControl.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        robot.imuExpansion.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        //set DC Motor drive modes
        robot.rearLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rearRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.shootyMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //set arm Motor to run with encoder
        robot.armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.shootyBoi.setPosition(SHOOTY_BOI_SERVO_LOAD_POS);
        robot.clawRotationServo.setPosition(0.68);
        robot.clawServo.setPosition(0.35);
        robot.shootyRotation.setPosition(0.89);



        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Calculated launch pos", CALCULATED_SHOOTY_ROTATION_LAUNCH);

    }


    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();

        if(robot.voltage.getVoltage() > 13.8){
            CALCULATED_SHOOTY_ROTATION_LAUNCH = 0.15;
        } else if (robot.voltage.getVoltage() > 13.5){
            CALCULATED_SHOOTY_ROTATION_LAUNCH = 0.14;
        } else if(robot.voltage.getVoltage() > 13.15){
            CALCULATED_SHOOTY_ROTATION_LAUNCH = 0.13;
        } else if(robot.voltage.getVoltage() > 12.9){
            CALCULATED_SHOOTY_ROTATION_LAUNCH = 0.12;
        } else if(robot.voltage.getVoltage() > 12.78){
            CALCULATED_SHOOTY_ROTATION_LAUNCH = 0.11;
        } else if(robot.voltage.getVoltage() >= 12.65){
            CALCULATED_SHOOTY_ROTATION_LAUNCH = 0.10;
        } else if(robot.voltage.getVoltage() < 12.65){
            CALCULATED_SHOOTY_ROTATION_LAUNCH = 0.09;
        }
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        if(!debug) {    //runs until debug is true

            if (gamepad1.back && !debugChanged) {   //toggles debug
                debug = true;
                debugChanged = true;
            } else if (!gamepad1.back) {
                debugChanged = false;
            }

            if (gamepad1.left_bumper && !isLBchanged) {   //moves shoty rotation to drop the wobble goal
                robot.clawRotationServo.setPosition(CLAW_ROTATION_WOBBLE);
                debugChanged = true;
            } else if (!gamepad1.left_bumper) {
                isLBchanged = false;
            }

            if (gamepad1.x && !isXchanged) {   //toggles pickup and drop position for rotation
                robot.clawRotationServo.setPosition(inPickupPos ? CLAW_ROTATION_SERVO_DROP : CLAW_ROTATION_SERVO_PICKUP);
                inPickupPos = !inPickupPos;
                isXchanged = true;
            } else if (!gamepad1.x) {
                isXchanged = false;
            }

            if (gamepad1.a && !isOpenChanged) {   //toggles open and close of claw servo
                robot.clawServo.setPosition(isOpen ? CLAW_SERVO_CLOSE_POS : CLAW_SERVO_OPEN_POS);
                isOpen = !isOpen;
                isOpenChanged = true;
            } else if (!gamepad1.a) {
                isOpenChanged = false;
            }

//            if(gamepad1.dpad_left) {
//                robot.shootyRotaion.setPosition(robot.shootyRotaion.getPosition() - 0.005);
//                if(robot.shootyRotaion.getPosition() < 0){
//                    robot.shootyRotaion.setPosition(0);
//                }
//            } else if(gamepad1.dpad_right) {
//                robot.shootyRotaion.setPosition(robot.shootyRotaion.getPosition() + 0.005);
//                if(robot.shootyRotaion.getPosition() > 1){
//                    robot.shootyRotaion.setPosition(1);
//                }
//            }

            if (gamepad1.dpad_up && !isUpChanged) {   //toggles position of shooty rotation servo UP
                shootyRotationCount++;
                if (shootyRotationCount > 2) {
                    shootyRotationCount = 2;
                }
                robot.shootyRotation.setPosition(shootyRotationCount == 1 ? SHOOTY_ROTATION_LAUNCH_LOW : CALCULATED_SHOOTY_ROTATION_LAUNCH);
                isUpChanged = true;
            } else if (!gamepad1.dpad_up) {
                isUpChanged = false;
            }

            if (gamepad1.dpad_down && !isDownChanged) {   //toggles position of shooty rotation servo DOWN
                shootyRotationCount--;
                if (shootyRotationCount < 0) {
                    shootyRotationCount = 0;
                }
                robot.shootyRotation.setPosition(shootyRotationCount == 1 ? SHOOTY_ROTATION_LAUNCH_LOW : SHOOTY_ROTATION_FLAT_POS);
                isDownChanged = true;
            } else if (!gamepad1.dpad_down) {
                isDownChanged = false;
            }

//            if(gamepad1.dpad_up){ //to manually adjust value of shooty motor speed
//                motorPower += 0.01;
//                if(motorPower >1){
//                    motorPower = 1;
//                }
//                robot.shootyMotor.setPower(motorPower);
//            } else if (gamepad1.dpad_down){
//                motorPower -= 0.01;
//                if(motorPower < 0){
//                    motorPower = 0;
//                }
//                robot.shootyMotor.setPower(motorPower);
//            }

            if (gamepad1.y && !isYchanged) {   //changes value of isRoofRaised
                isYchanged = true;
                if (isRoofRaised) {   //MOVE ARM MOTOR DOWN
                    if (!robot.touchyKid.getState()) {   //checks if limit switch is closed
                        robot.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        robot.armMotor.setTargetPosition(5375);
                        robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        robot.armMotor.setPower(1);
                        if (robot.armMotor.getCurrentPosition() >= 5375) {
                            robot.armMotor.setPower(0.0);
                        }
                        state = grabbingState.armUp;
                    }
                } else if (!isRoofRaised) {   //MOVE ARM MOTOR UP
                    robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.armMotor.setTargetPosition(0);
                    robot.armMotor.setPower(1);
                    if (!robot.touchyKid.getState()) {
                        robot.armMotor.setPower(0.0);
                    }
                    state = grabbingState.armDown;
                }
                isRoofRaised = !isRoofRaised;
            }
            else if (!gamepad1.y) {
                isYchanged = false;
            }


            if (gamepad1.right_bumper && !shootyIsShootingChanged) {   //shoots disc
                robot.shootyBoi.setPosition(SHOOTY_BOI_SERVO_SHOOT_POS);
                double pressTime = runtime.milliseconds();
                while (runtime.milliseconds() - pressTime < 250) {  //wait until the shooty servo has fully moves forward.
                    telemetry.addData("Wait ", "true");
                    telemetry.update();
                }
                robot.shootyBoi.setPosition(SHOOTY_BOI_SERVO_LOAD_POS);
                isShooting = !isShooting;
                shootyIsShootingChanged = true;
            } else if (!gamepad1.right_bumper) {
                shootyIsShootingChanged = false;
            }

            if (gamepad1.start && !shootyIsRunningChanged) {   //toggles turning on and off shooty motor
                robot.shootyMotor.setPower(shootyIsRunning ? 0 : 0.59);
                shootyIsRunning = !shootyIsRunning;
                shootyIsRunningChanged = true;
            } else if (!gamepad1.start) {
                shootyIsRunningChanged = false;
            }
            /*if (gamepad1.b && !isBchanged) {
                isBchanged = true;
                grabDepositRing();

            } else if (!gamepad1.b) {
                isBchanged = false;
            }
            */
            switch(state){
                case armUp: {
                    if (gamepad1.b && switchTime.milliseconds() >= armDelay) {
                        switchTime.reset();
                        robot.armMotor.setTargetPosition(0);
                        robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        robot.armMotor.setPower(1);
                        robot.clawRotationServo.setPosition(CLAW_ROTATION_SERVO_DROP);
                        robot.shootyRotation.setPosition(SHOOTY_ROTATION_FLAT_POS);
                        state = grabbingState.dropRing;
                    }
                    break;
                }

                case dropRing: {
                    if (switchTime.milliseconds() >= armDelay) {
                        switchTime.reset();
                        robot.clawServo.setPosition(CLAW_SERVO_OPEN_POS);
                        state = grabbingState.armDown;
                    }
                    break;
                }

                case armDown: {
                    if (gamepad1.b && switchTime.milliseconds() >= 500) {
                        switchTime.reset();
                        robot.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        robot.armMotor.setTargetPosition(5375);
                        robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        robot.armMotor.setPower(1);
                        robot.shootyRotation.setPosition(SHOOTY_ROTATION_LAUNCH_LOW);
                        state = grabbingState.armUp;
                    }
                    break;
                }

            }




            //left stick
            double drive  =  gamepad1.left_stick_y; //note: for the future, the Y direction should be negated and not the x direction
            double strafe = -gamepad1.left_stick_x;
            //right stick
            double turn = -gamepad1.right_stick_x;

            //calculates power
            rearLeftPower    = Range.clip(drive - strafe + turn, -1, 1) ;
            rearRightPower   = Range.clip(drive + strafe - turn, -1, 1) ;
            frontLeftPower = Range.clip(drive + strafe + turn, -1, 1) ;
            frontRightPower = Range.clip(drive - strafe - turn, -1, 1) ;

            // Send calculated power to rear wheels
            robot.rearLeftDrive.setPower(rearLeftPower);
            robot.rearRightDrive.setPower(rearRightPower);
            robot.frontLeftDrive.setPower(frontLeftPower);
            robot.frontRightDrive.setPower(frontRightPower);

//            telemetry.addData("color_sensor_red", robot.colorSensor.red());
//            telemetry.addData("color_sensor_green", robot.colorSensor.green());
//            telemetry.addData("color_sensor_blue", robot.colorSensor.blue());
//            telemetry.addData("distance_sensor_cm", robot.distanceSensor.getDistance(DistanceUnit.CM));


            // Show the elapsed game time.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Is Open", ": " + isOpen);
            telemetry.addData("Is roof raised", ": " + isRoofRaised);
            telemetry.addData("Is arm motor busy", ""  + robot.armMotor.isBusy());
            telemetry.addData("Arm Motor Encoder Position", robot.armMotor.getCurrentPosition());
            telemetry.addData("Is Touching sensor", ": " + !robot.touchyKid.getState());
            telemetry.addData("Shooty Rotation servo Position: ", "%5.2f", robot.shootyRotation.getPosition());
            telemetry.addData("motorPower: " , "" + motorPower);
            telemetry.addData("State", ": " + state);
            telemetry.addData("Voltage", ": " + robot.voltage.getVoltage());
            telemetry.addData("SwitchTime", ": " + switchTime.toString());

            telemetry.update();

        } else {    //runs if debug = true
            telemetry.addData("Debug mode: " ,"enabled.");

            //claw servo
            if(gamepad1.b) {
                robot.clawServo.setPosition(robot.clawServo.getPosition() + DEBUG_INCREMENT);

            } else if(gamepad1.a) {
                robot.clawServo.setPosition(robot.clawServo.getPosition() - DEBUG_INCREMENT);
                if(robot.clawServo.getPosition() <= 0.35){
                    robot.clawServo.setPosition(0.35);
                }
            }
            telemetry.addData("Claw servo Position: ", "%5.2f", robot.clawServo.getPosition());

            //rotation servo
            if(gamepad1.y) {
                robot.clawRotationServo.setPosition(robot.clawRotationServo.getPosition() + DEBUG_INCREMENT);
            } else if(gamepad1.x) {
                robot.clawRotationServo.setPosition(robot.clawRotationServo.getPosition() - DEBUG_INCREMENT);
            }
            telemetry.addData("Claw Rotation servo Position: ", "%5.2f", robot.clawRotationServo.getPosition());

            //shooty boi servo
//            if(gamepad1.right_bumper) {
//                robot.shootyBoi.setPosition(robot.shootyBoi.getPosition() + DEBUG_INCREMENT);
//            } else if (gamepad1.left_bumper) {
//                robot.shootyBoi.setPosition(robot.shootyBoi.getPosition() - DEBUG_INCREMENT);
//            }
//            telemetry.addData("Shooty Boi servo Position: ", "%5.2f", robot.shootyBoi.getPosition());


            //shooty rotation servo
            if(gamepad1.dpad_left) {
                robot.shootyRotation.setPosition(robot.shootyRotation.getPosition() - DEBUG_INCREMENT);
                if(robot.shootyRotation.getPosition() < 0){
                    robot.shootyRotation.setPosition(0);
                }
            } else if(gamepad1.dpad_right) {
                robot.shootyRotation.setPosition(robot.shootyRotation.getPosition() + DEBUG_INCREMENT);
                if(robot.shootyRotation.getPosition() > 1){
                    robot.shootyRotation.setPosition(1);
                }
            }
            telemetry.addData("Shooty Rotation servo Position: ", "%5.2f", robot.shootyRotation.getPosition());

            //arm Motor
            if(gamepad1.dpad_up){
                robot.armMotor.setPower(-.5);
            }
            else if (gamepad1.dpad_down){
                robot.armMotor.setPower(.5);
            }
            else{
                robot.armMotor.setPower(0);
            }

            telemetry.addData("Arm Motor ", "Position: %7d", robot.armMotor.getCurrentPosition());
            telemetry.addData("touchyKid", robot.touchyKid.getState());
            telemetry.addData("Current Angle", readDoubleAngle());

            if(gamepad1.start && !shootyIsRunningChanged){   //toggles turning on and off shooty motor
                robot.shootyMotor.setPower(shootyIsRunning ? 0 : 0.59);
                shootyIsRunning = !shootyIsRunning;
                shootyIsRunningChanged = true;
            } else if (!gamepad1.start) {
                shootyIsRunningChanged = false;
            }

            if(gamepad1.back && !debugChanged){   //toggles debug
                debug = false;
                debugChanged = true;
            } else if (!gamepad1.back) {
                debugChanged = false;
            }
            telemetry.update();
        }
    }

    public String readAngle() {
        angleControl = robot.imuControl.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        angleExpansion = robot.imuExpansion.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return "Expansion: " + String.valueOf(angleExpansion.firstAngle) + "\nAngle: Control: " + String.valueOf(angleControl.firstAngle);
    }

    public double readDoubleAngle() {
        angleControl = robot.imuControl.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angleControl.firstAngle;
    }

    public void grabDepositRing(){
        /*TODO Figure out why it is skipping most of the first lines of code.
        make two button presses--one for rotating shooty the other for raising
        (consider using a state machine)*/
        robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.armMotor.setTargetPosition(0);
        robot.armMotor.setPower(1);
        if(!robot.touchyKid.getState()) {
            robot.armMotor.setPower(0.0);
        }
        robot.shootyRotation.setPosition(SHOOTY_ROTATION_FLAT_POS);
        robot.clawRotationServo.setPosition(CLAW_ROTATION_SERVO_DROP);
        robot.clawServo.setPosition(CLAW_SERVO_OPEN_POS);
        robot.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.armMotor.setTargetPosition(5375);
        robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.armMotor.setPower(1);
        if (robot.armMotor.getCurrentPosition() >= 5375) {
            robot.armMotor.setPower(0.0);
        }
    }
    public void strafe(double distance, double speed) {
        int newLeftFrontTarget;
        int newRightBackTarget;
        int newRightFrontTarget;
        int newLeftBackTarget;
        double strafeScale =(10000.0/98.0) * (24.0/27.0) * (24.0/27.0);

        wheelSetMode(1);

        newLeftFrontTarget = robot.frontLeftDrive.getCurrentPosition() + (int) (distance * strafeScale);
        newLeftBackTarget = robot.rearLeftDrive.getCurrentPosition() - (int) (distance * strafeScale);
        newRightFrontTarget = robot.frontRightDrive.getCurrentPosition() - (int) (distance * strafeScale);
        newRightBackTarget = robot.rearRightDrive.getCurrentPosition() + (int) (distance * strafeScale);

        robot.frontLeftDrive.setTargetPosition(newLeftFrontTarget);
        robot.rearLeftDrive.setTargetPosition(newLeftBackTarget);
        robot.frontRightDrive.setTargetPosition(newRightFrontTarget);
        robot.rearRightDrive.setTargetPosition(newRightBackTarget);

        robot.frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rearLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rearRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.frontLeftDrive.setPower(Math.abs(speed));
        robot.frontRightDrive.setPower(Math.abs(speed));
        robot.rearLeftDrive.setPower(Math.abs(speed)); //-
        robot.rearRightDrive.setPower(Math.abs(speed)); //-

        while (robot.rearLeftDrive.isBusy() && robot.frontLeftDrive.isBusy() && robot.frontLeftDrive.isBusy() && robot.rearRightDrive.isBusy()){

        }
    }
    public void wheelSetMode(int mode){
        robot.rearLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rearRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if(mode == 1){
            robot.rearLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rearRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            robot.rearLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rearRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        if(mode == 2){
            robot.rearLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.rearRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }


    }
    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        //TODO uncomment this section if you guys want the robot to automatically raise the arm when it turns off so we dont have to deal with manually moving it & ensure it starts upright
//        if(!isRoofRaised){   //MOVE ARM MOTOR UP
//            robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            robot.armMotor.setTargetPosition(0);
//            robot.armMotor.setPower(1);
//            if(!robot.touchyKid.getState()){
//                robot.armMotor.setPower(0.0);
//            }
//        }
    }



            }