#ifndef DETECTORCONSTRUCTION_HH
#define DETECTORCONSTRUCTION_HH

#include "G4VUserDetectorConstruction.hh"
#include "G4NistManager.hh"
#include "G4Material.hh"
#include "globals.hh"

#include "G4SDManager.hh"
#include "G4MultiFunctionalDetector.hh"
#include "G4PSEnergyDeposit.hh"

#include "G4AssemblyVolume.hh"     // 여러 논리볼륨을 묶어 하나처럼 배치하는 컨테이너
#include "G4Transform3D.hh"        // 위치/회전을 하나의 변환으로 묶는 클래스
#include "G4RotationMatrix.hh"     // 회전 행렬




using namespace CLHEP;

class G4VPhysicalVolume;
class G4LogicalVolume;

class DetectorConstruction : public G4VUserDetectorConstruction
{
  public:
    DetectorConstruction( G4double rotDeg );
    virtual ~DetectorConstruction();

    virtual G4VPhysicalVolume* Construct();

  private:
    G4LogicalVolume *logicWorld;
    G4LogicalVolume *logicDetector;

    G4double fRotDeg = 0;

    G4NistManager* nist = nullptr;

    void SetUpTarget();
    void SetUpDump();
    void SetUpCollimator();
    void SetUpDetector();
    	
};

#endif
