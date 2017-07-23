// gcc -g -Wall -lholomni_pcv -I./pr/include -I./ -ansi -pedantic pcv_test.cpp -o pcv_test

// gcc -g -Wall -lyaml-cpp -I./pr/include -I./ -ansi -pedantic pcv_test.cpp Pcv.cpp Filter.cpp Vehicle.cpp Caster.cpp Ctrl.cpp Traj3.cpp pr/src/matrix/PrMatrix.cpp pr/src/matrix/PrVector.cpp  -o pcv_test

#include "Pcv.h"
#include <mcheck.h> /* Header file to include mtrace related functions */
#include <stdlib.h>
#include <yaml-cpp/yaml.h>
int main()
{
  //mtrace(); /* This starts memory tracing. */
  printf("Starting PCV Test..\n");
#ifdef YAMLCPP_03
  std::ifstream fin("/home/meka/mekabot/m3qa/robot_config/mb1/pcv_meka_b1r1.yml");
  YAML::Parser parser(fin);
  YAML::Node doc;
  parser.GetNextDocument(doc);
#else
  std::string fin("/home/meka/mekabot/m3qa/robot_config/mb1/pcv_meka_b1r1.yml");
  YAML::Node doc = YAML::LoadFile(fin);
#endif
  PCV * pcv = new PCV(doc, 1000);
  
  delete pcv;
  
  pcv = NULL;
  
  pcv = new PCV(doc, 1000);
  
  delete pcv;
    
  
  printf("Exiting PCV Test..\n");
  
 return EXIT_SUCCESS;
}