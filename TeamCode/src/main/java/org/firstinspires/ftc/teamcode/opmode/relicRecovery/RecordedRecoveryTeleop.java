package org.firstinspires.ftc.teamcode.opmode.relicRecovery;

import android.content.Context;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.base.BlackBox;
import org.firstinspires.ftc.teamcode.base.Color;
import org.firstinspires.ftc.teamcode.opmode.RobotHardware;

import java.io.FileOutputStream;

/**
 * Recorded teleop mode.
 * This mode records the hardware which can later be played back in autonomous.
 * Select the manual control mode by changing the parent class.
 */
public class RecordedRecoveryTeleop extends RelicRecoveryManual {
    @TeleOp(name="cb.Recorded.PlaybackAuto", group="chargerbots")
    public static class RecordedRecoveryTeleopPlaybackAuto extends RecordedRecoveryTeleop {
        @Override public void init() {
            filename = "PlaybackAuto";
            super.init();
        }
    }

    @TeleOp(name="cb.Recorded.Red.Center", group="chargerbots")
    public static class RecordedRecoveryTeleopRedCenter extends RecordedRecoveryTeleop {
        @Override public void init() {
            filename = getStartPositionName(Color.Ftc.RED,
                                            StartPosition.FIELD_CENTER);
            super.init();
        }
    }

    @TeleOp(name="cb.Recorded.Red.Corner", group="chargerbots")
    public static class RecordedRecoveryTeleopRedCorner extends RecordedRecoveryTeleop {
        @Override public void init() {
            filename = getStartPositionName(Color.Ftc.RED,
                                            StartPosition.FIELD_CORNER);
            super.init();
        }
    }

    @TeleOp(name="cb.Recorded.Blue.Center", group="chargerbots")
    public static class RecordedRecoveryTeleopBlueCenter extends RecordedRecoveryTeleop {
        @Override public void init() {
            filename = getStartPositionName(Color.Ftc.BLUE,
                                            StartPosition.FIELD_CENTER);
            super.init();
        }
    }

    @TeleOp(name="cb.Recorded.Blue.Corner", group="chargerbots")
    public static class RecordedRecoveryTeleopBlueCorner extends RecordedRecoveryTeleop {
        @Override public void init() {
            filename = getStartPositionName(Color.Ftc.BLUE,
                                            StartPosition.FIELD_CORNER);
            super.init();
        }
    }

    /**
     * Extends teleop initialization to start a recorder.
     */
    public void init() {
        super.init();
        startTime = -1;
        try {
            outputStream = hardwareMap.appContext.openFileOutput(
                    filename, Context.MODE_PRIVATE);
            recorder = new BlackBox.Recorder(hardwareMap, outputStream);
        } catch (Exception e) {
            e.printStackTrace();
            requestOpModeStop();
        }
    }

    /**
     * Extends teleop control to record hardware after loop.
     */
    public void loop() {
        super.loop();
        if (startTime == -1) {
            startTime = time;
        }
        double elapsed = time - startTime;
        telemetry.addData("Recording File", filename);
        telemetry.addData("Elapsed", elapsed);

        try {
            for (RelicMotorName m : RelicMotorName.values()) {
                recorder.record(m.getName(), elapsed);
            }
            for (RelicServoName s : RelicServoName.values()) {
                recorder.record(s.name(), elapsed);
            }
        } catch (Exception e) {
            e.printStackTrace();
            requestOpModeStop();
        }
    }

    /**
     * Closes the file to flush recorded data.
     */
    public void stop() {
        super.stop();

        try {
            recorder = null;
            outputStream.close();
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    // The filename base to write to.
    protected String filename;
    // The output file stream.
    private FileOutputStream outputStream;
    // The hardware recorder.
    private BlackBox.Recorder recorder;
    // Start time of recording.
    private double startTime;
}
