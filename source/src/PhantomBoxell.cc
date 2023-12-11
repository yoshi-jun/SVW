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
#include "G4NistManager.hh"
#include "G4ThreeVector.hh"
#include "G4VPhysicalVolume.hh"
#include "G4VTouchable.hh"

#include "G4SystemOfUnits.hh"
#include "phantom_boxell.h"

namespace
{
  G4Material* materi_ = nullptr;
} // namespace


// --------------------------------------------------------------------------
PhantomBoxcell::PhantomBoxcell(std::string name)
  : nz_{0}, dz_{0.}
{
  auto nist_manager = G4NistManager::Instance();
  ::materi_ = nist_manager-> FindOrBuildMaterial(name);
}

// --------------------------------------------------------------------------
PhantomBoxcell::~PhantomBoxcell()
{
}

// --------------------------------------------------------------------------
G4Material* PhantomBoxcell::ComputeMaterial(G4VPhysicalVolume* physvol,
                                        const int idx,
                                        const G4VTouchable* parent)
{
  return ::materi_;
}

// --------------------------------------------------------------------------
int PhantomBoxcell::GetNumberOfMaterials() const
{
  return 1;
}

// --------------------------------------------------------------------------
G4Material* PhantomBoxcell::GetMaterial(int idx) const
{
  return ::materi_;
}

// --------------------------------------------------------------------------
void PhantomBoxcell::ComputeTransformation(const int idx,
                                       G4VPhysicalVolume* physvol) const
{
  double z = dz_ * ( -nz_ / 2. + idx + 0.5 );
  auto vec = G4ThreeVector(0., 0., z);
  physvol-> SetTranslation(vec);
}
