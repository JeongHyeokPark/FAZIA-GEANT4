#include "globals.hh"
#include <sstream>

#include "G4RunManager.hh"
#include "G4VisExecutive.hh"
#include "G4UImanager.hh"
#include "G4UIExecutive.hh"
#include "G4ParticleDefinition.hh"
#include "G4IonTable.hh"
#include "G4ProcessManager.hh"
#include "G4UIterminal.hh"
#include "G4UItcsh.hh"

#include "DetectorConstruction.hh"
#include "PrimaryGeneratorAction.hh"
#include "RunAction.hh"
#include "SteppingAction.hh"
#include "EventAction.hh"

#include "FTFP_INCLXX.hh"
#include "FTFP_BERT.hh"

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

using namespace std;

int main(int argc, char** argv)
{
  G4bool bSaveKinematics = 1; // 0: Don't save kinemaitcs, 1: Save
  G4double kinEnergy = 70; // MeV unit
  G4double rotationDeg = 30.;

  auto bIsMac = std::system( "lsb_release >> /dev/null" );
  G4int ranSeed = strtod(argv[1], NULL);
  long seed = time(0)*gethostid() * ranSeed;
  if( bIsMac ) seed = time(0) * ranSeed;
  CLHEP::HepRandom::setTheSeed( seed );

  if(argc <= 1)
  {
    cout << "PLEASE USE THE FOLLOWING COMMAND TO RUN THE EXECUTABLE" << endl;
    cout << "./example N" << endl;
    cout << "OR" << endl;
    cout << "./example N run.mac" << endl;
    return -1;
  }

  G4String fileNumber = argv[1];

  G4RunManager* runManager = new G4RunManager;
  runManager -> SetUserInitialization( new DetectorConstruction( rotationDeg ) );
  //runManager -> SetUserInitialization(new FTFP_INCLXX());
  runManager -> SetUserInitialization(new FTFP_BERT());
  runManager -> SetUserAction(new RunAction(fileNumber, bSaveKinematics));
  runManager -> SetUserAction(new PrimaryGeneratorAction((G4double) kinEnergy));
  runManager -> SetUserAction(new EventAction());
  runManager -> SetUserAction(new SteppingAction(bSaveKinematics));
  runManager -> Initialize();


  G4VisManager* visManager = new G4VisExecutive;
  visManager -> Initialize();
  G4UImanager* UImanager = G4UImanager::GetUIpointer();



  if(argc > 2)
  {
    G4String command = "/control/execute ";
    G4String fileName = argv[2];
    UImanager -> ApplyCommand(command+fileName);
  }
  else if(argc <= 2)
  {
    //G4UIExecutive* ui = new G4UIExecutive(argc, argv);
    auto ui = new G4UIterminal(new G4UItcsh);
    UImanager -> ApplyCommand("/control/execute vis.mac");
    ui -> SessionStart();
    delete ui;
  }

  delete visManager;
  delete runManager;
  return 0;
}
