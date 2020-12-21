/* Copyright (c) 2019 FIRST. All rights reserved.
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

package UltimateGoal_RobotTeam.OpModes.Test.Prototypes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

import UltimateGoal_RobotTeam.HarwareConfig.HardwareRobotMulti;
import UltimateGoal_RobotTeam.OpModes.BasicOpMode;
import UltimateGoal_RobotTeam.OpModes.TeleOp.BasicTeleOp;

/**
 * This 2020-2021 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the Ultimate Goal game elements.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
@TeleOp(name = "ImageRecog tfod test", group = "Test")

public class ImageRecogTest extends BasicTeleOp {


    @Override
    public void runOpMode() {

        initializeTeleOp();

        waitForStart();

        runCode();

    }

    @Override
    public void initializeTeleOp() {
        runtime.reset();
        telemetry.addLine("NOT READY DON'T PRESS PLAY");
        telemetry.update();
        telemetry.setAutoClear(false);//allow all the lines of telemetry to remain during initialization

        // configure the robot needed - for this demo only need DriveTrain
        // configArray has True or False values for each subsystem HW element
        //
        /** configArray is arranged as
         * [0] = DriveTrain
         * [1] = Shooter
         * [2] = Conveyor
         * [3] = WobbleArm
         * [4] = Collector
         * [5] = ImageRecog
         * items that are 1 = true will be configured to the robot
         */
        // HW ELEMENTS *****************    DriveTrain  Shooter  Conveyor	WobbleArm	Collector   ImageRecog
        boolean[] configArray = new boolean[]{ false, 	false, 	false, 		false, 		false,      true};

        robotUG = new HardwareRobotMulti(this, configArray,false);
        // READ HASHMAP FILE
        readOrWriteHashMap();

        //Indicate initialization complete and provide telemetry
        telemetry.addData("Status: ", "Initialized");
        telemetry.addData(">", "Press Play to start");
        telemetry.update();

        telemetry.setAutoClear(true);//revert back to telemetry.update clearing prior display

    }
    public void runCode() {
// run until the end of the match (driver presses STOP)
        int methodSelect = 0;
        while (opModeIsActive()) {
            while (methodSelect == 0 && opModeIsActive()) {

                // Run image recognition number of loops, show results, determine to continue
                String ringType = robotUG.imageRecog.viewRings(this, 100);
                telemetry.addLine("------------------------------------");
                telemetry.addData("Loop Completed", "Review data and repeat?");

                pressAToContinue();

                methodSelect = switchMethod(methodSelect);

                idle();
            }
            while (methodSelect == 1 && opModeIsActive()) {

                // Run image recognition timed loops, show results, determine to continue
                String ringType = robotUG.imageRecog.viewRingsTimed(this, 2.0);
                telemetry.addLine("------------------------------------");
                telemetry.addData("Loop Completed", "Review data and repeat?");

                pressAToContinue();

                methodSelect = switchMethod(methodSelect);

                idle();
            }
        }
        robotUG.imageRecog.shutdown();
    }
    public int switchMethod(int mSelect){
        while ((!gamepad1.x && !gamepad1.y)  && opModeIsActive()){
            telemetry.addLine("%%%%%%%%%%%%%%%%%%%%%%%%%%%%");
            telemetry.addData("Method Select", "Press 'X' to run same, Press 'Y' to change");
            telemetry.addLine("%%%%%%%%%%%%%%%%%%%%%%%%%%%%");
            telemetry.update();
        }
        if(gamepad1.y){
            if(mSelect>0){
                mSelect -=1;
            }
            else {
                mSelect +=1;

            }
            telemetry.addLine(" ");
            telemetry.addData("Method Changed", "Method Select value = %d", mSelect);
            telemetry.update();
        }
        else {
            telemetry.addLine(" ");
            telemetry.addData("Method NOT Changed", "Method Select value = %d", mSelect);
            telemetry.update();
        }
        sleep(2000);//wait so screen can be read

        return mSelect;
    }
}
