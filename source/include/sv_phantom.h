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
#ifndef SV_PHANTOM_
#define SV_PHANTOM_

#include "G4Material.hh"
#include "G4VNestedParameterisation.hh"

class G4Material;
class G4PhysicalVolume;

class SVPhantom : public G4VNestedParameterisation {

public:
  SVPhantom(std::string f_name, int max_z);
  ~SVPhantom()=default;

  void MakeSVPhantom();
  
  G4Material* ComputeMaterial(G4VPhysicalVolume* physvol,
                                      const int idx,
                                      const G4VTouchable* parent = 0) override;

  int GetNumberOfMaterials() const override;

  G4Material* GetMaterial(int idx) const override;

  void ComputeTransformation(const int idx,
                                     G4VPhysicalVolume* physvol) const override;

  void SetSVNum(std::vector<int> sv_num);
  std::vector<int> GetSVNum() const;
  
  
private:
  std::vector<int> sv_size_;
  std::vector<double> sv_elements_;
  std::vector<G4Material*> sv_mat_;
};

#endif

//==============================================================================
inline void SVPhantom::SetSVNum(std::vector<int> sv_num) 
{
  if (sv_num.size() != 3) {
    std::cout << "Size of SV is NOT 3" << std::endl;
    std::exit(EXIT_SUCCESS);
  }

  sv_size_ = sv_num;

}

// --------------------------------------------------------------------------
inline std::vector<int> SVPhantom::GetSVNum() const
{
  return sv_size_;
}