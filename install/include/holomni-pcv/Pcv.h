#ifndef _Pcv_h_
#define _Pcv_h_

#include <stdio.h>
#include <math.h>
#include <string>
#include <iostream>
#include <fstream>
#include "Ctrl.h"
#include "yaml-cpp/yaml.h"
#define MSG  printf

class PCV
{
 public:
  PCV( YAML::Node& doc, int step_freq );

 
 ~PCV()
 {   
   if (ctrl != NULL)
   {     
    delete ctrl;     
   }
   ctrl = NULL;   
 }
 
 Pcv_Status Step(Pcv_Command pcv_cmd);
 void UpdateGoal(Pcv_Command pcv_cmd);  
      
  private:    
    Pcv_Params params;
    Float ENC_REV2CNT;
     Ctrl * ctrl;
};

#endif // _Pcv_h_