#ifndef NxQMDParameters_hh
#define NxQMDParameters_hh

#include "globals.hh"

class NxQMDParameters 
{
      static G4ThreadLocal NxQMDParameters* parameters;
     
      NxQMDParameters();
   public:
      ~NxQMDParameters();
      static NxQMDParameters* GetInstance()
      {
         if ( parameters == NULL ) parameters = new NxQMDParameters(); 
         return parameters;
      }


      // GroundStateNucleus
      G4double Get_wl() { return wl; };
      G4double Get_cl() { return cl; };
      G4double Get_hbc() { return hbc; };
      G4double Get_rho0() { return rho0; };
      G4double Get_gamm() { return gamm; };
      G4double Get_cpw() { return cpw; };
      G4double Get_cph() { return cph; };
      G4double Get_epsx() { return epsx; };
      G4double Get_cpc() { return cpc; };
      G4double Get_cs() { return cs; };
      G4double Get_c0() { return c0; };
      G4double Get_c0p() { return c0p; };
      G4double Get_clp() { return clp; };
      G4double Get_c3() { return c3; };
      G4double Get_c3p() { return c3p; };
      G4double Get_csp() { return csp; };
      G4double Get_cdp() { return cdp; };

   protected:
      G4double wl;
      G4double cl;
      G4double hbc; 
      G4double rho0;
      G4double gamm; 

/*
      G4double rho0;
      G4double t0;

      G4double aaa;

*/
      // MeanField 
      G4double c0, c3, cs;

      // GroundStateNucleus
      G4double cpw; 
      G4double cph; 
      G4double epsx;
      G4double cpc;
      G4double c0p , clp , c3p , csp , cdp;

};

#endif
