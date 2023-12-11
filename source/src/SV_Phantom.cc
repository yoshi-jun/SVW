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

#include "sv_phantom.h"

#include <fstream>

//--------------------------------------------------------------------------
SVPhantom::SVPhantom(std::string file_name, int max_z)
{
  for (int i=0; i < max_z; i++) {
    std::string fname = file_name + std::to_string(i) + ".csv";
    
    std::ifstream fin(fname);

    if (!fin) {
      std::cout << "The file named "<< fname << "is NOT found" << std::endl;

      std::exit(EXIT_SUCCESS);
    }
    std::string data;

    std::cout << fname << std::endl;

    while (getline(fin, data)) {    
    
    std::istringstream stream(data);
    std::string tmp = "";

      while(getline(stream, tmp, ',')) {
        
        // std::cout << tmp << std::endl;

        auto density = 1. + std::stod(tmp) * 2/1000 ; 
        sv_elements_.push_back(density);

        // std::cout << density << std::endl;
      }
      
    }
    
  }
  std::cout << "size of SV elements is " << sv_elements_.size() << std::endl;

  auto nist_manager = G4NistManager::Instance();
  
  auto v = sv_elements_;
  std::sort(v.begin(), v.end());
  auto it = unique(v.begin(),v.end());
  v.erase(it, v.end());

  for (double elem : v) {
    std::cout << elem << "," << std::endl;
    auto mat = nist_manager->BuildMaterialWithNewDensity(std::to_string(elem), 
                                                  "G4_WATER", elem);
    sv_mat_.push_back(mat);

    std::cout << mat << std::endl;
  }
}
// --------------------------------------------------------------------------
G4Material* SVPhantom::ComputeMaterial(G4VPhysicalVolume* physvol,
                                        const int idx,
                                        const G4VTouchable* parent)
{
  auto ix = parent->GetReplicaNumber(0);
  auto iy = parent->GetReplicaNumber(1);

  auto num = ix + iy * 104 + idx * 104 * 104;

  std::cout << "Compute material " << idx << ", " 
            << ix << ", " << iy << ", " << num << ", "
            << sv_elements_[num] << std::endl;
  

  auto nist_manager = G4NistManager::Instance();
  auto mat = nist_manager->FindOrBuildMaterial(std::to_string(sv_elements_[num]));

  std::cout << mat << std::endl; 
  return mat;
}

// --------------------------------------------------------------------------
int SVPhantom::GetNumberOfMaterials() const
{
  return sv_mat_.size();
}

// --------------------------------------------------------------------------
G4Material* SVPhantom::GetMaterial(int idx) const
{
  auto nist_manager = G4NistManager::Instance();
  auto mat = nist_manager->FindOrBuildMaterial(std::to_string(sv_elements_[idx]));
  return mat;  
}

// --------------------------------------------------------------------------

void SVPhantom::ComputeTransformation(const int idx,
                                    G4VPhysicalVolume* physvol) const
{
  std::cout << "compute transformation, idx = " << idx << std::endl;
  double z =  1. * ( -30. / 2. + idx + 1. );
  auto vec = G4ThreeVector(0., 0., z);
  std::cout << vec << std::endl;
  physvol-> SetTranslation(vec);
}