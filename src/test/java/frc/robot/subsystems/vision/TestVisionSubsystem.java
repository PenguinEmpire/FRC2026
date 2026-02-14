package frc.robot.subsystems.vision;

//import static org.mockito.Mockito.mock;

import org.junit.jupiter.api.Test;

import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.vision.VisionInterface;


public class TestVisionSubsystem {
    @Test
    public void testCurrentID() {
        MyVisionTester myTester = new MyVisionTester();
        myTester.FiducialID = 1;
        VisionSubsystem subsystem = new VisionSubsystem(myTester);
        subsystem.updateOffsets();
        assert(subsystem.getCurrentID()==-1);
    }


    @Test
    public void testChangedCurrentID() {
        MyVisionTester myTester = new MyVisionTester();
        myTester.FiducialID = 10;
        myTester.hasID = true;
        VisionSubsystem subsystem = new VisionSubsystem(myTester);
        subsystem.updateOffsets();
        assert(subsystem.getCurrentID()==10);
        assert(subsystem.Fiducial3DOffsetX == -0.597);
        assert(subsystem.Fiducial3DOffsetY == 0);
        assert(subsystem.Fiducial3DOffsetZ == 0);
    }

    @Test
    public void testResetTarget() {
        MyVisionTester myTester = new MyVisionTester();
        myTester.FiducialID = 1;
        myTester.hasID = true;
        VisionSubsystem subsystem = new VisionSubsystem(myTester);
        subsystem.updateOffsets();
        assert(subsystem.getCurrentID()==1);
        myTester.hasID = false;
        myTester.FiducialID = 2;
        subsystem.updateOffsets();
        assert(subsystem.getCurrentID()==-1);
    }
} 