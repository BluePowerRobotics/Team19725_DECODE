package org.firstinspires.ftc.teamcode.utility;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;

import java.io.IOException;

public class RoadRunnerFileWriter {
    public class FileWriterAction implements Action{
        private MecanumDrive drive;
        public FileWriterAction(MecanumDrive driveRC){
            this.drive = driveRC;
        }

        @Override
        public boolean run(@NonNull com.acmerobotics.dashboard.telemetry.TelemetryPacket telemetryPacket) {
            try (java.io.FileWriter writer = new java.io.FileWriter("/sdcard/FIRST/pose.txt")) {
                writer.write(drive.localizer.getPose().position.x + "," +
                        drive.localizer.getPose().position.y + "," +
                        drive.localizer.getPose().heading.toDouble());
            } catch (IOException e) {
                throw new RuntimeException(e);
            }
            return false; // 每帧都执行，不结束
        }
    }
    public Action WriteFile(MecanumDrive driveRC) {
        return new FileWriterAction(driveRC);
    }


}
