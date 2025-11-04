#include "SteppingAction.hh"

#include "G4Event.hh"
#include "G4RunManager.hh"
#include "G4SystemOfUnits.hh"
#include "G4EventManager.hh"
#include "G4ParticleDefinition.hh"

// ===== [추가] 누적용 컨테이너/헤더 =====
#include <unordered_map>
#include "G4VProcess.hh"
#include "G4TouchableHistory.hh" // G4TouchableHandle 사용 시 안전

SteppingAction::SteppingAction( G4bool bSaveKinematics )
  : G4UserSteppingAction()
{
  fSaveKinematics = bSaveKinematics;
}

SteppingAction::~SteppingAction()
{
}

void SteppingAction::UserSteppingAction(const G4Step* step)
{
  analysisManager = G4AnalysisManager::Instance();
  eventAction = (EventAction*) G4EventManager::GetEventManager()->GetUserEventAction();

  // 이번 이벤트에서 gold target과 kinematics를 이뤘는지 아닌지 확인하는 flag를 가져온다
  G4bool isGoldTarget = eventAction->GetGoldTarget();
  // Stepping Action 에서 결정 된 gold target kinematics를 만들었는지 확인하는 용도
  G4bool localGoldTarget = false;

  // Saving Data (기존 변수들 유지)
  G4int eventID = G4RunManager::GetRunManager() -> GetCurrentEvent() -> GetEventID();
  G4int parentID = step -> GetTrack() -> GetParentID();
  G4int trackID = step -> GetTrack() -> GetTrackID();
  G4int pdg = step -> GetTrack() -> GetDefinition() -> GetPDGEncoding();
  G4double edep = step -> GetTotalEnergyDeposit();

  G4ThreeVector mom = step->GetTrack()->GetMomentum();

  G4int trackFlag = 0;
  if( eventAction->GetTrackFlag()->size() ) 
  {
    auto it = eventAction->GetTrackFlag()->find(trackID);
    if (it != eventAction->GetTrackFlag()->end()) 
      trackFlag = eventAction->GetTrackFlag()->at(trackID);
  }



  // 가장 먼저 kinematics가 일어났는지, 확인한다
  // 내가 처음 쏴주는 양성자일때
  //if( parentID==0 && pdg==2212 )
  {
    const std::vector< const G4Track* >* secondaryTracks = step->GetSecondaryInCurrentStep();
    int numOfSecondary = secondaryTracks->size();


    if( numOfSecondary!=0 )
    {
      const G4Track* secondTrack = secondaryTracks->at(0);
      auto proc = secondTrack->GetCreatorProcess();
      G4String processName = proc->GetProcessName();
      G4double parentA = step->GetTrack()->GetDefinition()->GetAtomicMass();
      G4double parentZ = step->GetTrack()->GetDefinition()->GetAtomicNumber();


      // Elastic Scattering은 모두 hadElastic임
      if( processName=="hadElastic" )
      {
        // X의 에너지와 secondary의 저장
        // 트랙이 하나만 생성 됐으니까, 지금 X 과 생성된 secondary 하나만 저장
        G4ThreeVector protonMom = step->GetTrack()->GetMomentum();
        G4ThreeVector targetMom = secondTrack->GetMomentum();

        G4int targetA = secondTrack->GetDefinition()->GetAtomicMass();
        G4int targetZ = secondTrack->GetDefinition()->GetAtomicNumber();

        G4double protonMass = step->GetTrack()->GetDefinition()->GetPDGMass();
        G4double targetMass = secondTrack->GetDefinition()->GetPDGMass();
        G4double protonKe = step->GetTrack()->GetKineticEnergy();
        G4double targetKe = secondTrack->GetKineticEnergy();

        if( targetZ==79 && targetA==197 ) localGoldTarget = true;

        bool localAirTarget = false;
        if( targetZ==6 || 
            targetZ==7 || 
            targetZ==8 || 
            targetZ==18 ) localAirTarget = true;

        if( localGoldTarget || localAirTarget )
        {
          //G4int secondTrackID = secondTrack->GetTrackID();
          G4int secondTrackID = trackID + 1; // Temp implementation

          // 1: beam proton->Au
          // 2: beam proton->Air
          // 3: beam proton->Au->X->Air
          if( localGoldTarget ) 
            eventAction->GetTrackFlag()->insert( std::pair<G4int,G4int> (secondTrackID,1) );
          if( localAirTarget ) 
          {
            eventAction->GetTrackFlag()->insert( std::pair<G4int,G4int> (secondTrackID,2) );
            if( trackFlag==1 ) eventAction->GetTrackFlag()->insert( std::pair<G4int,G4int> (secondTrackID,3) );
          }

          if( fSaveKinematics )
          {
            analysisManager->FillNtupleIColumn( 0, eventID );
            analysisManager->FillNtupleIColumn( 1, targetZ );
            analysisManager->FillNtupleIColumn( 2, targetA );
            analysisManager->FillNtupleIColumn( 3, 2212 );
            analysisManager->FillNtupleDColumn( 4, protonMom.x() );
            analysisManager->FillNtupleDColumn( 5, protonMom.y() );
            analysisManager->FillNtupleDColumn( 6, protonMom.z() );
            analysisManager->FillNtupleDColumn( 7, protonMass );
            analysisManager->FillNtupleDColumn( 8, protonKe );
            analysisManager->FillNtupleIColumn( 9, 0 );
            analysisManager -> AddNtupleRow();


            analysisManager->FillNtupleIColumn( 0, eventID );
            analysisManager->FillNtupleIColumn( 1, targetZ );
            analysisManager->FillNtupleIColumn( 2, targetA );
            analysisManager->FillNtupleIColumn( 3, secondTrack->GetDefinition()->GetPDGEncoding() );
            analysisManager->FillNtupleDColumn( 4, targetMom.x() );
            analysisManager->FillNtupleDColumn( 5, targetMom.y() );
            analysisManager->FillNtupleDColumn( 6, targetMom.z() );
            analysisManager->FillNtupleDColumn( 7, targetMass );
            analysisManager->FillNtupleDColumn( 8, targetKe );
            analysisManager->FillNtupleIColumn( 9, 0 );
            analysisManager -> AddNtupleRow();
          }
        }
      }

      // protonInelastic 말고, 모든 Inelastic을 찾으면,
      if( processName.contains("Inelastic") )
      {
        G4int targetZ = 0;
        G4int targetA = 0;
        //for( G4int i=0; i<numOfSecondary; i++ )
          for( G4int i=numOfSecondary-1; i>=0; i-- )
        {
          const G4Track* daughterTrack = secondaryTracks->at(i);
          G4double daughterA = daughterTrack->GetDefinition()->GetAtomicMass();
          G4double daughterZ = daughterTrack->GetDefinition()->GetAtomicNumber();

          targetZ += daughterZ;
          targetA += daughterA;
        }
        //targetZ--; // To compensate the proton number
        //targetA--; // To compensate the proton number
        targetZ -= parentZ; // To compensate the X number
        targetA -= parentA; // To compensate the X number

        if( targetZ==79 && targetA==197 ) localGoldTarget = true;

        bool localAirTarget = false;
        if( targetZ==6 || 
            targetZ==7 || 
            targetZ==8 || 
            targetZ==18 ) localAirTarget = true;

        if( localGoldTarget || localAirTarget )
        {
          // secondary 나오는 트랙들의 정보 저장
          //for( G4int i=0; i<numOfSecondary; i++ )
          for( G4int i=numOfSecondary-1; i>=0; i-- )
          {
            const G4Track* daughterTrack = secondaryTracks->at(i);
            //G4int daughterTrackID = daughterTrack->GetTrackID();
            G4int daughterTrackID = trackID + i + 1; // Temp Implementation

            // 1: beam proton->Au
            // 2: beam proton->Air
            // 3: beam proton->Au->X->Air
            if( localGoldTarget ) 
              eventAction->GetTrackFlag()->insert( std::pair<G4int,G4int> (daughterTrackID,1) );
            if( localAirTarget ) 
            {
              eventAction->GetTrackFlag()->insert( std::pair<G4int,G4int> (daughterTrackID,2) );
              if( trackFlag==1 ) eventAction->GetTrackFlag()->insert( std::pair<G4int,G4int> (daughterTrackID,3) );
            }

            G4ThreeVector daughterMom = daughterTrack->GetMomentum();
            G4double daughterMass = daughterTrack->GetDefinition()->GetPDGMass();
            G4double ke = daughterTrack->GetKineticEnergy();

            if( fSaveKinematics )
            {
              analysisManager->FillNtupleIColumn( 0, eventID );
              analysisManager->FillNtupleIColumn( 1, targetZ );
              analysisManager->FillNtupleIColumn( 2, targetA );
              analysisManager->FillNtupleIColumn( 3, daughterTrack->GetDefinition()->GetPDGEncoding() );
              analysisManager->FillNtupleDColumn( 4, daughterMom.x() );
              analysisManager->FillNtupleDColumn( 5, daughterMom.y() );
              analysisManager->FillNtupleDColumn( 6, daughterMom.z() );
              analysisManager->FillNtupleDColumn( 7, daughterMass );
              analysisManager->FillNtupleDColumn( 8, ke );
              analysisManager->FillNtupleIColumn( 9, 1 );
              analysisManager -> AddNtupleRow();
            }
          }
        }
      }

    }
  }




  // event 에서 선언하는 gold target 이 false 이지만, stepping action 에서
  // true로 판단하면 그 때 부터 이 이벤트는 모두 gold target 이벤트로 이 다음
  // energy deposit 정보들을 모두 모으기 시작한다
  if( isGoldTarget==false && 
      localGoldTarget==true ) eventAction->SetGoldTarget( localGoldTarget );



  //if( eventAction->GetGoldTarget() ) // 만약 gold target event 만 저장하고 싶다면
  //if( !eventAction->GetGoldTarget() ) // 만약 gold target event 가 아닌 것들만 저장하고 싶다면
  {
    // =====각 트랙이 각 검출기에서 잃은 에너지 누적 추가=====
    G4double energy_deposit_step = edep;

    // 스텝 시작점에서 현재 물리/논리 볼륨 얻기
    const G4StepPoint* pre_step_point   = step->GetPreStepPoint();
    G4TouchableHandle  touchable_handle = pre_step_point->GetTouchableHandle();
    G4VPhysicalVolume* current_phys_volume = touchable_handle->GetVolume();  // null 가능
    if (current_phys_volume == 0 || energy_deposit_step <= 0.0) return;

    G4int copyNum = step->GetTrack()->GetVolume()->GetCopyNo();
    if( copyNum==0 || copyNum==1 ) return; // If trajectory deposits in world or target, we return this UserSteppingAction

    G4int telNum = (copyNum - 1000) / 100; // This should be 0~15
    G4int detNum = copyNum % 10;


    if (current_phys_volume != 0 && energy_deposit_step > 0.0) {
      G4LogicalVolume* current_logical_volume = current_phys_volume->GetLogicalVolume();
      G4String         current_logical_name   = current_logical_volume->GetName();

      /*
         if (current_logical_name == "logicSi1_epi") 
         eventAction->AccumulateEdepSi1( energy_deposit_step );
         else if (current_logical_name == "logicSi2_int") 
         eventAction->AccumulateEdepSi2( energy_deposit_step );
         else if (current_logical_name == "logicCsI") 
         eventAction->AccumulateEdepCsI( energy_deposit_step );
       */

      if (detNum == 3) // If the detector is epi
        eventAction->AccumulateEdepSi1( telNum, energy_deposit_step );
      else if (detNum == 6) // If the detector is int
        eventAction->AccumulateEdepSi2( telNum, energy_deposit_step );
      else if (detNum == 7)// If the detector is CsI
        eventAction->AccumulateEdepCsI( telNum, energy_deposit_step );

      if( trackFlag )
      {
        if (detNum == 3) // If the detector is epi
          eventAction->AccumulateEdepSi1( trackFlag, telNum, energy_deposit_step );
        else if (detNum == 6) // If the detector is int
          eventAction->AccumulateEdepSi2( trackFlag, telNum, energy_deposit_step );
        else if (detNum == 7)// If the detector is CsI
          eventAction->AccumulateEdepCsI( trackFlag, telNum, energy_deposit_step );
      }
    }
    // =================================================================
  }


}

