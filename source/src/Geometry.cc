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
#include "G4Box.hh"
#include "G4LogicalVolume.hh"
#include "G4NistManager.hh"
#include "G4PVParameterised.hh"
#include "G4PVPlacement.hh"
#include "G4PVReplica.hh"
#include "G4RotationMatrix.hh"
#include "G4SDManager.hh"
#include "G4SystemOfUnits.hh"
#include "G4ThreeVector.hh"
#include "G4Transform3D.hh"
#include "G4Tubs.hh"
#include "G4UserSpecialCuts.hh"
#include "G4UserLimits.hh"
#include "G4VisAttributes.hh"
#include "G4VPhysicalVolume.hh"

#include "geometry.h"
#include "json_reader.h"
#include "phantom_boxell.h"
#include "sensitive_volume.h"
// #include "super_voxel.h"
#include "sv_phantom.h"
//------------------------------------------------------------------------------
Geometry::Geometry()
{}

//------------------------------------------------------------------------------
Geometry::~Geometry()
{}

//------------------------------------------------------------------------------
G4VPhysicalVolume* Geometry::Construct()
{
  // Define materials
  G4NistManager* materi_Man = G4NistManager::Instance();

  auto galactic = materi_Man-> FindOrBuildMaterial("G4_Galactic");
  auto air      = materi_Man-> FindOrBuildMaterial("G4_AIR");
  auto water    = materi_Man-> FindOrBuildMaterial("G4_WATER");

  // Define World
  G4double leng_X_World = 150.0 * cm;
  G4double leng_Y_World = 150.0 * cm;
  G4double leng_Z_World = 150.0 * cm;

  auto pv_world = new G4Box{ "PV_World",
                        leng_X_World/2.0, leng_Y_World/2.0, leng_Z_World/2.0 };
  auto lv_world = new G4LogicalVolume{ pv_world, air, "LV_World" };
  auto world  = new G4PVPlacement{ G4Transform3D(), "World",
                                      lv_world, 0, false, 0};

  //-----------------------------------------------------------------------------
  // Define the number of boxcell
  auto dose_score = DoseCounter::GetInstance();

  auto reader = JsonReader::GetInstance();
  auto voxel_num = reader->GetVoxelNum();
  
  // auto super_voxel = new SuperVoxel("voxel_data.json");
  // auto sv_num = super_voxel-> GetSVNum();
  // super_voxel-> MakeSuperVoxel();

  auto sv_phantom = new SVPhantom("sv_dicom_test/sv_dicom_test_", 30);
  std::vector<int> sv_num = {104, 104, 30};
  sv_phantom-> SetSVNum(sv_num);

  G4int nDiv_X = sv_num[0];
  G4int nDiv_Y = sv_num[1];
  G4int nDiv_Z = sv_num[2];

  // Define the length of Water Box
  G4double leng_X_Box = 20.8 * cm;
  G4double leng_Y_Box = 20.8 * cm;
  G4double leng_Z_Box = 30.0 * cm;

  // Define the length of a Boxcell
  G4double leng_X_Boxcell = leng_X_Box / nDiv_X;
  G4double leng_Y_Boxcell = leng_Y_Box / nDiv_Y;
  G4double leng_Z_Boxcell = leng_Z_Box / nDiv_Z;

  // Define the logical box
  auto pv_solid_box = new G4Box{ "PV_Solid_Box",
                                  leng_X_Box / 2.0,
                                  leng_Y_Box / 2.0,
                                  leng_Z_Box / 2.0 };
  auto lv_solid_box = new G4LogicalVolume{ pv_solid_box, water,
                                            "LV_Solid_Box" };

  // Define boxcell X
  auto pv_boxcell_x = new G4Box{ "PV_Bocell_X",
                                  leng_X_Boxcell / 2.0,
                                  leng_Y_Box 		 / 2.0,
                                  leng_Z_Box 		 / 2.0 };

  auto lv_boxcell_x = new G4LogicalVolume{ pv_boxcell_x, water,
                                            "LV_Boxcell_X" };

  new G4PVReplica{ "Boxcell_X", lv_boxcell_x, lv_solid_box, kXAxis,
                  nDiv_X, leng_X_Boxcell };

  // Define boxcell XY
  auto pv_boxcell_xy = new G4Box{ "PV_Boxcell_XY",
                                  leng_X_Boxcell / 2.0,
                                  leng_Y_Boxcell / 2.0,
                                  leng_Z_Box 		 / 2.0 };

  auto lv_boxcell_xy = new G4LogicalVolume{ pv_boxcell_xy, water,
                                            "LV_Boxcell_XY" };

  new G4PVReplica{ "Boxcell_XY", lv_boxcell_xy, lv_boxcell_x, kYAxis,
                  nDiv_Y, leng_Y_Boxcell };

  // Define boxcell XYZ
  auto pv_boxcell_xyz = new G4Box{ "PV_Boxcell_XYZ",
                                    leng_X_Boxcell / 2.0,
                                    leng_Y_Boxcell / 2.0,
                                    leng_Z_Boxcell / 2.0 };

  auto lv_boxcell_xyz = new G4LogicalVolume{ pv_boxcell_xyz, water,
                                            "LV_Boxcell_XYZ" };

  // new G4PVReplica{ "Boxcell_XYZ", lv_boxcell_xyz, lv_boxcell_xy, 
  //                                 kZAxis, nDiv_Z, leng_Z_Boxcell };

  // auto cell_status = new PhantomBoxcell("G4_WATER");
  // cell_status-> Setdatas(nDiv_Z, leng_Z_Boxcell);
  // new G4PVParameterised("Boxcell_XYZ", lv_boxcell_xyz, lv_boxcell_xy,
  //                           kZAxis, nDiv_Z, cell_status);

  // new G4PVParameterised("SuperVoxel_XYZ", lv_boxcell_xyz, lv_boxcell_xy,
  //                           kZAxis, sv_num[2], super_voxel);

  new G4PVParameterised("SuperVoxel_XYZ", lv_boxcell_xyz,
                            lv_boxcell_xy, kZAxis, sv_num[2], sv_phantom);

  //-----------------------------------------------------------------------------
  // Define the point of Water box
  G4double pos_x_box = 0.0 * cm;
  G4double pos_y_box = 0.0 * cm;
  G4double pos_z_box = 50. * cm;

  //Place the Box to world
  auto pos_box = G4ThreeVector{ pos_x_box,
                                pos_y_box,
                                pos_z_box };

  auto rot_box = G4RotationMatrix{};
  auto lv_box = G4Transform3D{ rot_box, pos_box};
  new G4PVPlacement{ lv_box, "Box", lv_solid_box, world, false, 0};


  //---------LIMIT--------------------------
  G4UserLimits* my_limit = new G4UserLimits();
  my_limit-> SetUserMinEkine(189 * keV);
  // lv_world-> SetUserLimits(my_limit);
  lv_boxcell_xyz-> SetUserLimits(my_limit);
  //----------------------------------------

  std::cout << " End Geometry Construct" << std::endl;
  // Return the physical world
  return world;
}

//------------------------------------------------------------------------------
void Geometry::ConstructSDandField()
{
  std::cout << "Start Construct SDand Field" << std::endl;
  auto mySV = new SensitiveVolume("SensitiveVolume");
  G4SDManager::GetSDMpointer()-> AddNewDetector(mySV);
  SetSensitiveDetector("LV_Boxcell_XYZ", mySV);
  std::cout << "End Construct SDand Field" << std::endl;
}
