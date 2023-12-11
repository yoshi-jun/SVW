/*==============================================================================
BSD 2-Clause License
Copyright (c) 2021 Junichi Yoshida
All rights reserved.
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
1. Redistributions of source code must retain the above copyright notice,
  this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
==============================================================================*/
#include "em_phys_pcut.h"

// particles
#include "G4Gamma.hh"
#include "G4Electron.hh"
#include "G4Positron.hh"

// gamma processes
#include "G4ComptonScattering.hh"
#include "G4GammaConversion.hh"
#include "G4PhotoElectricEffect.hh"

// electron/positron processes
#include "G4MscStepLimitType.hh"
#include "G4eMultipleScattering.hh"
#include "G4eIonisation.hh"
#include "G4eBremsstrahlung.hh"
#include "G4eplusAnnihilation.hh"
#include "G4SeltzerBergerModel.hh"
#include "G4Generator2BS.hh"

#include "G4PhysicsListHelper.hh"
#include "G4EmParameters.hh"
#include "G4SystemOfUnits.hh"
#include "G4UserSpecialCuts.hh"
#include "G4ProcessManager.hh"

#include "G4UserSpecialCuts.hh"
#include <cassert>

//------------------------------------------------------------------------------
EmPhysPcut::EmPhysPcut()
{
  defaultCutValue = 1.0 * mm;
  G4EmParameters::Instance()->SetMaxEnergy(1.0 * GeV);
  SetVerboseLevel(1);
}

//------------------------------------------------------------------------------
void EmPhysPcut::ConstructParticle()
{
  G4Gamma::Gamma();
  G4Electron::Electron();
  G4Positron::Positron();
}

//------------------------------------------------------------------------------
void EmPhysPcut::ConstructProcess()
{
  AddTransportation();

  auto ph = G4PhysicsListHelper::GetPhysicsListHelper();

  auto particle_iterator = GetParticleIterator();
  particle_iterator->reset();

  while ((*particle_iterator)()) {

    auto particle = particle_iterator->value();
    auto particle_name = particle->GetParticleName();

    if (particle_name == "gamma") {

      ph->RegisterProcess(new G4PhotoElectricEffect, particle);
      ph->RegisterProcess(new G4ComptonScattering, particle);
      ph->RegisterProcess(new G4GammaConversion, particle);
      // for gamma cut 10keV(energy defined in geometory)
      // ph->RegisterProcess(new G4UserSpecialCuts, particle);

    } else if (particle_name == "e-") {

      auto em_msc = new G4eMultipleScattering();
      em_msc->SetStepLimitType(fMinimal);
      ph->RegisterProcess(em_msc, particle);
      ph->RegisterProcess(new G4eIonisation, particle);

      auto br = new G4eBremsstrahlung();
      auto br_model = new G4SeltzerBergerModel();
      //br_model->SetAngularDistribution(new G4Generator2BS());
      br->SetEmModel(br_model);
      ph->RegisterProcess(br, particle);

      //to energy cut (energy value is defined geom)
      ph->RegisterProcess(new G4UserSpecialCuts(),particle);

    } else if (particle_name == "e+") {

      auto ep_msc = new G4eMultipleScattering();
      ep_msc->SetStepLimitType(fMinimal);
      ph->RegisterProcess(ep_msc, particle);
      ph->RegisterProcess(new G4eIonisation, particle);
      ph->RegisterProcess(new G4eplusAnnihilation, particle);

      auto br = new G4eBremsstrahlung();
      auto br_model = new G4SeltzerBergerModel();
      //br_model->SetAngularDistribution(new G4Generator2BS());
      br->SetEmModel(br_model);
      ph->RegisterProcess(br, particle);

      //to energy cut (energy value is defined geom)
      ph->RegisterProcess(new G4UserSpecialCuts(),particle);
    }

  }

}

//------------------------------------------------------------------------------
void EmPhysPcut::SetCuts()
{
  // G4VUserPhysicsList::SetCuts();
  // setting value gamma is fast and next e-, e+. e-,+ need cut value of gamma
  SetCutValue(0.42 * mm, "gamma");
  // cut value 0.42mm is 189keV electron CSDA range in the water
    // 189 + 511 = 700 keV
  SetCutValue(0.42 * mm, "e-");
  SetCutValue(0.42 * mm, "e+");

  if (verboseLevel > 0) { DumpCutValuesTable(); }
}

//------------------------------------------------------------------------------

