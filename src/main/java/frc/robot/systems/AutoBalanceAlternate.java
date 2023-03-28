package frc.robot.systems;
import frc.robot.systems.ElevFourbar.Setpoint;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.AutoModes;

// Alternate way to try auto balance -- EG
public class AutoBalanceAlternate {
    private static final double RAMMING_SPEED = -0.75;
    private static final double ENCROACHING_SPEED = 0.2;
    private static final double DRIVE_TO_RAMP = 350;
    private static final double PITCH_RATE_THRESHOLD = 5;
    private static final double PITCH_THRESHOLD = 3;    

    private Timer stepTimer = new Timer();
    
    public static int Balance_Step = 0;
    int start_time;

    public AutoBalanceAlternate(int startTime)
    {
        start_time = startTime;
        stepTimer.reset();
    }
    public boolean run()
    {
        double pitch = Pigeon.getPitch();
        double pitch_rate = Pigeon.getChangePerSecond().pitchPerSecond;

        switch (Balance_Step) {
            case 0: //Initialize at desired start time
                if(AutoModes.autoBalanceTimer.get()>start_time){
                    Balance_Step++;
                    stepTimer.reset();
                    stepTimer.start();                   
                }
                break;
            case 1: //Drive toward and up ramp a total distance, then stop when distance is reached
                SwerveDrive.runUncurved(0.0, RAMMING_SPEED, 0.0);
                if(Math.abs(SwerveDrive.getModuleInfo(1).getPosition().distanceMeters)>=DRIVE_TO_RAMP)
                {
                    SwerveDrive.runUncurved(0.0, 0.0, 0.0);
                    Balance_Step++;
                    stepTimer.reset();
                    stepTimer.start();
                }
                else if(stepTimer.get()>3.0)  //If taking too long, give up all AutoBalance
                    Balance_Step = -1;
                break;
            case 2: //Try to balance                
                if(stepTimer.get()>5.0){ //If taking too long, stop robot and go to next step
                    SwerveDrive.runUncurved(0.0, 0.0, 0.0);
                    Balance_Step ++;
                }  
                else if(Math.abs(pitch_rate) > PITCH_RATE_THRESHOLD) // If pitch rate is high, stop robot
                    SwerveDrive.runUncurved(0.0, 0.0, 0.0);
                else if(pitch > PITCH_THRESHOLD) // if pitch is positive, drive towards balance point
                    SwerveDrive.runUncurved(0.0, ENCROACHING_SPEED, 0.0);
                else if(pitch < -PITCH_THRESHOLD) // if pitch is negative, drive towards balance point
                    SwerveDrive.runUncurved(0.0, -ENCROACHING_SPEED, 0.0);
                else // Robot thinks we are balanced -- pitch rate is low and pitch is under threshold, so we're done
                    Balance_Step=4;
                break;
            case 3: // This is reached if attempts to balance timed out.  Last ditch effort: set arm to Mid position to change CG
                if(pitch > PITCH_THRESHOLD)
                {
                    SwerveDrive.runUncurved(0.0, 0.0, 0.0); //Make ure we're stopped
                    ElevFourbar.autoRun(Setpoint.MID_SCORING);    
                }                    
                Balance_Step++;
            case 4:
                SwerveDrive.runUncurved(0.0, 0.0, 0.0); //Make sure we're stopped!
                return true;

            default:
                break;
        }
        return false;
    }
}

