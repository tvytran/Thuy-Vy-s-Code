/*Copyright (c) 2021 FIRST. All rights reserved.
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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Basic: Omni Linear OpMode", group="Linear Opmode")

public class ControlDrive extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    //DeclareOpMode members for the two arm motors
    private DcMotor armMotor = null;
    private DcMotor armMotor2 = null;

    //Declare OpMode members for the servos
    private Servo servo = null;


    static final double INCREMENT   = 0.01;     // amount to slew servo each CYCLE_MS cycle
    static final int    CYCLE_MS    =   50;     // period of each cycle
    static final double MAX_POS     =  1.0;     // Maximum rotational position
    static final double MIN_POS     =  0.0;     // Minimum rotational position

    static final double STRAFE_SPEED = 0.5;
    double position = 0;

    @Override
    public void runOpMode() {

        //initializing motors and servos
        initialize();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            /*====================================
                       WHEEL MOTOR
             =====================================*/

            runDriver();


            /*=====================================
                        ARM MOTOR
            =======================================*/
            //Go down
            goingDown();

            //Go up
            goingUp();

            /*=====================================
                        CLAW STUFF
            ========================================*/
            //open
            openClaw();
            //close
            closeClaw();


            /*
            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.update();
            */

        }
    }

    // Strafe left with the Mecanum wheels.
    public void strafeLeft() {
        // Set the speed of the left front and left back wheels to the strafe speed.
        leftFrontDrive.setPower(STRAFE_SPEED);
        leftBackDrive.setPower(STRAFE_SPEED);

        // Set the speed of the right front and right back wheels to the negative of the strafe speed.
        rightFrontDrive.setPower(-STRAFE_SPEED);
        rightBackDrive.setPower(-STRAFE_SPEED);
    }

    // Strafe right with the Mecanum wheels.
    public void strafeRight() {
        // Set the speed of the left front and left back wheels to the negative of the strafe speed.
        leftFrontDrive.setPower(-STRAFE_SPEED);
        leftBackDrive.setPower(-STRAFE_SPEED);

        // Set the speed of the right front and right back wheels to the strafe speed.
        rightFrontDrive.setPower(STRAFE_SPEED);
        rightBackDrive.setPower(STRAFE_SPEED);
    }


    //==============================================================================
    //initialize stuff
    public void initWheelMotor()
    {
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "leftBackDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBackDrive");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
    }
    public void initArmMotor()
    {
        armMotor = hardwareMap.get(DcMotor.class, "armMotor");
        armMotor2 = hardwareMap.get(DcMotor.class, "armMotor2");
    }
    public void initServo()
    {

        servo = hardwareMap.get(Servo.class, "servo");
    }
    public void initialize()
    {
        initWheelMotor();
        initArmMotor();
        initServo();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        runtime.reset();
    }
    //====================================================================================

    //Open Claw
    public void openClaw()
    {
        if(gamepad1.right_trigger == 1.0)
        {
            // Keep stepping up until we hit the max value
            servo.setPosition(MAX_POS);

        }
    }
    //close claw
    public void closeClaw()
    {

        if(gamepad1.right_bumper)
        {
            servo.setPosition(MIN_POS);
        }
    }


    //============================================================================================



    //run manually
    public void runDriver()
    {
        double max;

        // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
        double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
        double lateral =  gamepad1.left_stick_x;
        double yaw     =  gamepad1.right_stick_x;

        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        double leftFrontPower  = axial + lateral + yaw;
        double rightFrontPower = axial - lateral - yaw;
        double leftBackPower   = axial - lateral + yaw;
        double rightBackPower  = axial + lateral - yaw;

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower  /= max;
            rightFrontPower /= max;
            leftBackPower   /= max;
            rightBackPower  /= max;
        }

        // This is test code:
        //
        // Uncomment the following code to test your motor directions.
        // Each button should make the corresponding motor run FORWARD.
        //   1) First get all the motors to take to correct positions on the robot
        //      by adjusting your Robot Configuration if necessary.
        //   2) Then make sure they run in the correct direction by modifying the
        //      the setDirection() calls above.
        // Once the correct motors move in the correct direction re-comment this code.

            /*
            leftFrontPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
            leftBackPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
            rightFrontPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
            rightBackPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad
            */


        //Send calculated power to wheels
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }


    
    //============================================================================================
    //elevator



    //Going up
    public void goingUp() {
        if(gamepad1.left_bumper)
        {
            armMotor.setDirection(DcMotor.Direction.REVERSE);
            armMotor2.setDirection(DcMotor.Direction.FORWARD);
            armMotor.setPower(1);
            armMotor2.setPower(1);
        }
        else if(gamepad1.left_bumper == false)
        {
            armMotor.setPower(0);
            armMotor2.setPower(0);
        }
    }

    //Going down
    public void goingDown() {
        if(gamepad1.left_trigger == 1.0){
            armMotor.setDirection(DcMotor.Direction.FORWARD);
            armMotor2.setDirection(DcMotor.Direction.REVERSE);
            armMotor.setPower(1);
            armMotor2.setPower(1);

        }
        else if(gamepad1.left_trigger == 0)
        {
            armMotor.setPower(0);
            armMotor2.setPower(0);
        }
    }



    //=====================================================================================================


}
