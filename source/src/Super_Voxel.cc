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
#include "super_voxel.h"
#include "json.hpp"
#include "json_reader.h"

// --------------------------------------------------------------------------
namespace {

} // end namespace

// --------------------------------------------------------------------------
class G4Material;
class G4VPhysicalVolume;

SuperVoxel::SuperVoxel(std::string f_name) 
{
  std::fstream fin(f_name);

	if (!fin) {
    std::cout << "Read error Super Voxel data" << std::endl;
    std::exit(EXIT_FAILURE);
  }

  nlohmann::json js;

  fin >> js;

  auto reader = JsonReader::GetInstance();
  auto voxel_num = reader->GetVoxelNum();

  auto sv_x = js["sv_x"].get<int>();
  auto sv_y = js["sv_y"].get<int>();
  auto sv_z = js["sv_z"].get<int>();
  
  sv_size_ = {sv_x, sv_y, sv_z};

  auto data_fname = js["fname"].get<std::string>();
  
  for (int i=0; i < voxel_num[2] ; i++) {
    std::string f;
    f = data_fname + std::to_string(i) + ".json";
    // std::cout << f << std::endl;
    
    std::fstream f_data(f);
    nlohmann::json js_d;
    
    f_data >> js_d;
    auto data = js_d["data"].get<std::vector<double>>();

    for (double d: data) {
      dicom_data_.push_back(d);
    } 
    
  }

  sv_data_.resize(sv_x * sv_y * sv_z, 0.0);
  
  // auto voxel_num = reader->GetVoxelNum();
  sv_len_ = {30.0 / sv_size_[0], 
             30.0 / sv_size_[1],
             30.0 / sv_size_[2]};
}

// --------------------------------------------------------------------------
void SuperVoxel::MakeSuperVoxel()
{
  auto reader = JsonReader::GetInstance();
  auto vox_num = reader-> GetVoxelNum();
  // std::cout << vox_num[0] << vox_num[1] << vox_num[2] << std::endl; 
  for (int k=0; k < vox_num[2]; k++) {
    for (int j=0; j < vox_num[1]; j++) {
      for (int i=0; i < vox_num[0]; i++) {
        
        auto idx = i + j * sv_size_[0] + k * sv_size_[1] * sv_size_[2];

        auto idx_sv = (i % sv_size_[0]) + (j % sv_size_[1]) * sv_size_[0]
                      + (k % sv_size_[2]) * sv_size_[0] * sv_size_[1];
        
        if (sv_data_[idx_sv] < dicom_data_[idx]) {
          sv_data_[idx_sv] = dicom_data_[idx];
        }
      }
    }
  }

  // for (double data : sv_data_) {
  //   std::cout << data << std::endl;
  // }

  // define Materials list 
  auto nist_manager = G4NistManager::Instance();
  auto max_elem = *std::max_element(dicom_data_.begin(),dicom_data_.end()); 

  for (int d=0;d < max_elem; d++) {
    auto mat = nist_manager->BuildMaterialWithNewDensity(std::to_string(d),
                                                         "G4_WATER", 
                                                         d*0.1);
    // std::cout << mat << std::endl;
    sv_materi_.push_back(mat);
  }

}

// --------------------------------------------------------------------------
G4Material* SuperVoxel::ComputeMaterial(G4VPhysicalVolume* physvol,
                                        const int idx,
                                        const G4VTouchable* parent)
{
  auto reader = JsonReader::GetInstance();
  auto voxel_num = reader->GetVoxelNum(); 

  auto z = idx % voxel_num[2];
  auto y = (idx - z * voxel_num[0] * voxel_num[1]) % voxel_num[1];
  auto x = idx - y * voxel_num[0] - z * voxel_num[0] * voxel_num[1];

  auto idx_sv = (x % sv_size_[0]) + (y % sv_size_[1]) * sv_size_[0]
                    + (z % sv_size_[2]) * sv_size_[0] * sv_size_[1];


  
  auto material = sv_materi_[sv_data_[idx_sv]];

  auto nist_manager = G4NistManager::Instance();
  auto mat = nist_manager-> FindOrBuildMaterial("G4_WATER");
  
  return mat;
}

// --------------------------------------------------------------------------
int SuperVoxel::GetNumberOfMaterials() const 
{
  return sv_data_.size();
}

// --------------------------------------------------------------------------
G4Material* SuperVoxel::GetMaterial(int idx) const
{
  auto reader = JsonReader::GetInstance();
  auto voxel_num = reader->GetVoxelNum(); 

  auto z = idx % voxel_num[2];
  auto y = (idx - z * voxel_num[0] * voxel_num[1]) % voxel_num[1];
  auto x = idx - y * voxel_num[0] - z * voxel_num[0] * voxel_num[1];

  auto idx_sv = (x % sv_size_[0]) + (y % sv_size_[1]) * sv_size_[0]
                    + (z % sv_size_[2]) * sv_size_[0] * sv_size_[1];

  auto material = sv_materi_[sv_data_[idx_sv]];

  return material;
}

// --------------------------------------------------------------------------
void SuperVoxel::ComputeTransformation(const int idx,
                                    G4VPhysicalVolume* physvol) const 
{
  auto reader = JsonReader::GetInstance();
  auto dz = reader->GetVoxelLen()[2];

  double z = sv_len_[2] * ( -sv_size_[2] / 2. + idx + 0.5);
  auto vec = G4ThreeVector(0., 0., z);
  physvol-> SetTranslation(vec);
}
