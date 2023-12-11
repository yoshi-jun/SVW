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

#include "G4TouchableHistory.hh"
#include "G4Track.hh"
#include "G4Step.hh"
#include "G4ParticleDefinition.hh"
#include "G4HCofThisEvent.hh"
#include "G4UnitsTable.hh"
#include "G4SystemOfUnits.hh"

#include "error_counter.h"
#include "sensitive_volume.h"

namespace
{
//-----------------------------------------------------------------------------
void PrintInfoPreStep(G4Step* aStep)
{
  auto pre_step_pt = aStep-> GetPreStepPoint()-> GetPosition();
  auto post_step_pt = aStep-> GetPostStepPoint() ->GetPosition();

  auto post_step_materi = aStep ->GetPostStepPoint()-> GetMaterial();
  auto pre_step_materi = aStep-> GetPreStepPoint()-> GetMaterial();

  auto kinetic_e = aStep->GetTrack() ->GetKineticEnergy();


  // std::cout << "kinetic e "
  //           << post_step_pt[0] / cm << "[cm]" << ","
  //           << post_step_pt[1] / cm << "[cm]" << ","
  //           << post_step_pt[2] / cm << "[cm]" << ","
  //           << kinetic_e / MeV << "[MeV]"
  //           << std::endl
  //           << "pre step material is " << std::endl
  //           << pre_step_materi << std::endl;
}
} // end of namespace

//------------------------------------------------------------------------------
SensitiveVolume::SensitiveVolume(G4String name)
  : G4VSensitiveDetector(name)
{
}

//------------------------------------------------------------------------------
SensitiveVolume::~SensitiveVolume()
{
}

//------------------------------------------------------------------------------
void SensitiveVolume::Initialize(G4HCofThisEvent*)
{
  sum_eDep = 0.;
  sum_stepLength =0.;
}

//------------------------------------------------------------------------------
void SensitiveVolume::EndOfEvent(G4HCofThisEvent*)
{

}

//------------------------------------------------------------------------------
bool SensitiveVolume::ProcessHits(G4Step* aStep, G4TouchableHistory*)
{
  //Get Totall Energy Deposit
  auto edep = aStep-> GetTotalEnergyDeposit();

  //Get boxcell number (copyNum)
  auto copyNum_z = aStep-> GetPreStepPoint()-> GetTouchableHandle()
                      -> GetCopyNumber(0);

  auto copyNum_y = aStep-> GetPreStepPoint()-> GetTouchableHandle()
                      -> GetCopyNumber(1);

  auto copyNum_x = aStep-> GetPreStepPoint()-> GetTouchableHandle()
                      -> GetCopyNumber(2);

  //Get position and material
  auto pre_step_pt = aStep-> GetPreStepPoint()-> GetPosition();
  auto post_step_pt = aStep-> GetPostStepPoint()-> GetPosition();

  auto kinetic_e = aStep-> GetTrack()-> GetKineticEnergy();

  //Score pre-step-point and Number of voxel
  auto scorer = DoseCounter::GetInstance();

  scorer-> ScoreDose(copyNum_x, copyNum_y, copyNum_z, edep);
  // scorer-> ScoreDoubleDose(copyNum_x, copyNum_y, copyNum_z, edep);

  // auto error_counter = ErrorCounter::GetInstance();
  // error_counter-> DoseDepthCount(copyNum_z, edep);
  // if (aStep-> IsFirstStepInVolume()) {
  //   scorer -> ScoreCount(copyNum_x, copyNum_y, copyNum_z);
  // }


  return true;
}
