# 2602H *JScript*
This branch was created solely for algorithm testing away from the main program.

Currently testing:
      
      -Okapi 2D Motion Profiling
      
      -IMU Turns
 
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
