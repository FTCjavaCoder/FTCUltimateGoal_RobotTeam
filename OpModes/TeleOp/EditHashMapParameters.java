package UltimateGoal_RobotTeam.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="EditHashMapParameters", group="TeleOp")

public class EditHashMapParameters extends BasicTeleOp {

    @Override
    public void runOpMode() {

        telemetry.addLine("Status: Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        fileWasRead = cons.readFromPhone(hashMapFile, this);
        telemetry.addData("File Was Read?","%s", fileWasRead);

        if (!fileWasRead) {

            cons.defineParameters();
            cons.writeToPhone(hashMapFile, this);

            fileWasRead = cons.readFromPhone(hashMapFile, this);
            telemetry.addData("Created File, file Was Read?","%s", fileWasRead);
        }
        pressAToContinue();

        cons.editHashMap(this);

        cons.writeToPhone(hashMapFile,this);

        telemetry.addLine("HashMap editing complete");
        telemetry.update();
        sleep(1000);
    }
}