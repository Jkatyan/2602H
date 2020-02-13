# 2602H *JScript*
This branch was created solely for algorithm testing away from the main program.

Currently using:
      
      -Okapi 2D Motion Profiling
      
      -IMU Turns
      
      -PID / Slew Linear Motions
      
Coming soon:

      -IMU assisted straight line driving
      
      -PID + IMU Turns
      
      -Auto Parallel Park / Stack (Ultrasonics -> 2D Motion Profile)
      
      
 
Function documentation:
      
      //Okapi 2D Motion Profiling
      //Step 1) In void competition_initialize() create a path.
      
            void competition initialize(){
                  profileController->generatePath(
                  {{0_ft, 0_ft, 0_deg}, {1_ft, 0_ft, 0_deg}}, "A");
            } 
            
       //Step 2) To follow a path, in void autonomous() set a target.
       
            void autonomous(){
                  profileController->setTarget("A"); //setTarget("A", true, true); to follow path backwards.
                  profileController->waitUntilSettled();
                  profileController->removePath("A"); //remove path once motion is complete.
            }
