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
#ifndef JSON_READER_H
#define JSON_READER_H

#include <iostream>
#include <vector>

class JsonReader {

public:
  ~JsonReader() = default;

  JsonReader(const JsonReader&) = delete;
  JsonReader& operator=(const JsonReader&) = delete;

  static JsonReader* GetInstance();

  void SetEventNum(int num);
  int GetEventNum();

  void SetChooser(std::string beam);
  std::string GetChoosed() const;

  void SetVoxelNum(int n_x, int n_y, int n_z);
  std::vector<int> GetVoxelNum() const;

  void SetVoxelLen(double l_x, double l_y, double l_z);
  std::vector<double> GetVoxelLen() const;

  void SetBeamDirec(double b_x, double b_y, double b_z);
  std::vector<double> GetBeamDirec() const;
  
  void SetParticleName(std::string name);
  std::string GetParticleName() const;

  void SetParticleEne(double val);
  double GetParticleEne() const;

  void SetSSD(double val);
  double GetSSD() const;

  void SetFileName(std::string name);
  std::string GetFileName() const;

private:

  JsonReader();

  std::string beam_flag_;

  std::string particle_;

  std::string file_name_;

  double kine_energy_;

  std::vector<int> voxel_num_;
  std::vector<double> voxel_len_;

  std::vector<double> beam_direc_;

  double ssd_;

  int e_num_;

};
#endif

//=============================================================================
inline JsonReader::JsonReader()
{
  voxel_num_.resize(3, 0);
  voxel_len_.resize(3, 0.0);
  beam_direc_.resize(3, 0.0);
}

//-----------------------------------------------------------------------------
inline JsonReader* JsonReader::GetInstance()
{
  static JsonReader the_chooser;

  return &the_chooser;
}

//-----------------------------------------------------------------------------
inline void JsonReader::SetEventNum(int num) 
{
  e_num_ = num;
}

//-----------------------------------------------------------------------------
inline int JsonReader::GetEventNum() 
{
  return e_num_;
}
inline void JsonReader::SetChooser(std::string beam)
{
  beam_flag_ = beam;
}

//-----------------------------------------------------------------------------
inline std::string JsonReader::GetChoosed() const
{
  return beam_flag_;
}

//-----------------------------------------------------------------------------
inline void JsonReader::SetVoxelNum(int n_x, int n_y, int n_z)
{
  voxel_num_[0] = n_x;
  voxel_num_[1] = n_y;
  voxel_num_[2] = n_z;
}

//-----------------------------------------------------------------------------
inline std::vector<int> JsonReader::GetVoxelNum() const
{
  return voxel_num_;
}

//-----------------------------------------------------------------------------
inline void JsonReader::SetVoxelLen(double l_x, double l_y, double l_z)
{
  voxel_num_[0] = l_x;
  voxel_num_[1] = l_y;
  voxel_num_[2] = l_z;
}

//-----------------------------------------------------------------------------
inline void JsonReader::SetBeamDirec(double b_x, double b_y, double b_z)
{
  beam_direc_[0] = b_x;
  beam_direc_[1] = b_y;
  beam_direc_[2] = b_z;
}

//-----------------------------------------------------------------------------
inline std::vector<double> JsonReader::GetBeamDirec() const
{
  return beam_direc_;
}

//-----------------------------------------------------------------------------
inline std::vector<double> JsonReader::GetVoxelLen() const
{
  return voxel_len_;
}

//-----------------------------------------------------------------------------
inline void JsonReader::SetParticleName(std::string name)
{
  particle_ = name;
}

//-----------------------------------------------------------------------------
inline std::string JsonReader::GetParticleName() const
{
  return particle_;
}

//-----------------------------------------------------------------------------
inline void JsonReader::SetParticleEne(double val)
{
  kine_energy_ = val;
}

//-----------------------------------------------------------------------------
inline double JsonReader::GetParticleEne() const
{
  return kine_energy_;
}

//-----------------------------------------------------------------------------
inline void JsonReader::SetSSD(double val)
{
  ssd_ = val;
}

//-----------------------------------------------------------------------------
inline double JsonReader::GetSSD() const
{
  return ssd_;
}

//-----------------------------------------------------------------------------
inline void JsonReader::SetFileName(std::string name)
{
  file_name_ = name;
}

//-----------------------------------------------------------------------------
inline std::string JsonReader::GetFileName() const
{
  return file_name_;
}