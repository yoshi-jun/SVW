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
#include<iostream>
#include<vector>

#include "G4Event.hh"
#include "G4ParticleGun.hh"
#include "G4ParticleTable.hh"
#include "G4PrimaryParticle.hh"
#include "G4PrimaryVertex.hh"
#include "G4PhysicalConstants.hh"
#include "G4SystemOfUnits.hh"
#include "Randomize.hh"

#include "primary_generator.h"
#include "json_reader.h"

//------------------------------------------------------------------------------
PencilbeamGenerator::PencilbeamGenerator()
{
}

//------------------------------------------------------------------------------
PencilbeamGenerator::~PencilbeamGenerator()
{
}

//------------------------------------------------------------------------------
void PencilbeamGenerator::GeneratePrimaries( G4Event* anEvent )
{
  //Get Score instance
  auto reader = JsonReader::GetInstance();

  auto particle_name = reader-> GetParticleName();
  auto particle_enegy = reader-> GetParticleEne();
  auto ssd = reader-> GetSSD();
  auto beam_direc = reader-> GetBeamDirec();
  beam_direc = {0., 0., 1.};

  //Get particle-talbe pointer
  G4ParticleTable* particle_table = G4ParticleTable::GetParticleTable();

  std::string aName = particle_name;
  double kinetic_e = particle_enegy * MeV;

  G4ThreeVector direction = {beam_direc[0], beam_direc[1], beam_direc[2]};
  auto particle_code = particle_table-> FindParticle(aName);

  auto primary_particle = new G4PrimaryParticle{};
  primary_particle-> SetKineticEnergy(kinetic_e);
  primary_particle-> SetMomentumDirection(direction);
  primary_particle-> SetParticleDefinition(particle_code);

  double pos_x = 0. * cm;
  double pos_y = 0. * cm;
  double pos_z = - (ssd - 35.0) * cm; // 15cm is half of phantom size 
  double time_zero = 0. * s;

  auto primary_vertex = new G4PrimaryVertex{pos_x, pos_y, pos_z, time_zero};
  // auto primary_vertex = new G4PrimaryVertex{pos_z, pos_x, pos_y, time_zero};
  // auto primary_vertex = new G4PrimaryVertex{pos_x, pos_z, pos_y, time_zero};

  primary_vertex-> SetPrimary(primary_particle);
    
  anEvent-> AddPrimaryVertex(primary_vertex);

}
