package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.DriveSubsystem.Alignment;
//test


public class AlignToTargetCommand extends Command {
    private final DriveSubsystem driveSubsystem;
    private final Alignment alignment;

    public AlignToTargetCommand(DriveSubsystem subsystem, Alignment alignment) {
        driveSubsystem = subsystem;
        this.alignment = alignment;
        addRequirements(driveSubsystem);
    }

    @Override
    public void execute() {
        driveSubsystem.alignToTarget(alignment);
    }

    @Override
    public boolean isFinished() {
        double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0.0);
        double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0.0);
        return Math.abs(tx) < DriveSubsystem.ALIGN_THRESHOLD && Math.abs(ty) < DriveSubsystem.ALIGN_THRESHOLD;
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.drive(0, 0, 0);
    }
}
