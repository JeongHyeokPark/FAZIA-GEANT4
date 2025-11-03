#ifndef EventAction_h
#define EventAction_h 1

#include "G4UserEventAction.hh"
#include "G4ThreeVector.hh"
#include "globals.hh"
//#include "g4root.hh"

#include <map>

class G4Event;

class EventAction : public G4UserEventAction
{
  public: // Without description

    EventAction();
    virtual ~EventAction();

    virtual void BeginOfEventAction(const G4Event*);
    virtual void   EndOfEventAction(const G4Event*);

    void SetGoldTarget( G4bool val ) { isGoldTarget = val; };
    G4bool GetGoldTarget() { return isGoldTarget; };


    void AccumulateEdepSi1( G4int det, G4double val ) { edepSi1[0][det] += val; };
    void AccumulateEdepSi2( G4int det, G4double val ) { edepSi2[0][det] += val; };
    void AccumulateEdepCsI( G4int det, G4double val ) { edepCsI[0][det] += val; };

    void AccumulateEdepSi1( G4int flag, G4int det, G4double val ) { edepSi1[flag][det] += val; };
    void AccumulateEdepSi2( G4int flag, G4int det, G4double val ) { edepSi2[flag][det] += val; };
    void AccumulateEdepCsI( G4int flag, G4int det, G4double val ) { edepCsI[flag][det] += val; };

    std::map< G4int, G4int > GetTrackFlag() { return fTrackFlag; };

  private:
    EventAction & operator=(const EventAction &right);
    EventAction(const EventAction&);

    G4bool isGoldTarget = false;

    G4double edepSi1[4][16] = {0};
    G4double edepSi2[4][16] = {0};
    G4double edepCsI[4][16] = {0};

    std::map< G4int, G4int > fTrackFlag;
};
#endif
