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
#ifndef STATICAL_ERROR_H_
#define STATICAL_ERROR_H_

#include <fstream>
#include <vector>

#include "G4Threading.hh"
#include "G4EventManager.hh"
#include "G4SystemOfUnits.hh"

class ErrorCounter{
private:
  ErrorCounter() = default;

public:
  ~ErrorCounter() = default;

  ErrorCounter(const ErrorCounter&) = delete;
  ErrorCounter& operator= (const ErrorCounter&) = delete;

  static ErrorCounter* GetInstance();

  void Initialize(int x, int y, int z, int t_max);

  void DoseDepthCount(int z, double dose);

  void Sqaring();

  void MargeAllThreads();

  void CalculateStaticalError(int e_num);

  void SaveStaticalError();

private:
  std::vector<double> dose_sqaring_;
  std::vector<double> dose_sum_;

  std::vector<double> dose_buffer_;

  std::vector<double> dose_ave_;
  std::vector<double> dose_sqr_;


  std::vector<double> statical_error_;

  int nx_;
  int ny_;
  int nz_;
  int nt_;
  int events_;
};

#endif

//=============================================================================
inline ErrorCounter* ErrorCounter::GetInstance() {
  static ErrorCounter error_counter;

  return &error_counter;
}

//-----------------------------------------------------------------------------
inline void ErrorCounter::Initialize(int x, int y, int z, int t_max) {

  nx_ = x;
  ny_ = y;
  nz_ = z;
  nt_ = t_max;

  int num = nz_ * nt_;
  
  dose_sum_.resize(num, 0.0);
  dose_sqaring_.resize(num, 0.0);
  dose_buffer_.resize(num, 0.0);
}
//-----------------------------------------------------------------------------
inline void ErrorCounter::DoseDepthCount(int z, double dose) {
  auto tid = G4Threading::G4GetThreadId();
  int id = z + tid * nz_;

  dose_buffer_[id] += dose;

}

//-----------------------------------------------------------------------------
inline void ErrorCounter::Sqaring() {
  
  auto tid = G4Threading::G4GetThreadId();
  
  for (int k = 0; k < nz_; k++) {
    auto id = k + tid * nz_;

    auto sqaring = dose_buffer_[id] * dose_buffer_[id];
    
    dose_sum_[id] += dose_buffer_[id];
    dose_sqaring_[id] += sqaring;
    dose_buffer_[id] = 0.;
  }

}

//-----------------------------------------------------------------------------
inline void ErrorCounter::MargeAllThreads() {
  
  for (int k = 0; k < nz_; k++) {
    
    double ave = 0.;
    double sqr = 0.;
    
    for (int t = 0; t < nt_; t++) {
      
      int idx = k + t * nz_;

      ave += dose_sum_[idx];
      sqr += dose_sqaring_[idx];

    } 
    
    dose_ave_.push_back(ave);
    dose_sqr_.push_back(sqr);
  }
}

//-----------------------------------------------------------------------------
inline void ErrorCounter::CalculateStaticalError(int e_num) {
  events_ = e_num;
  
  for (int k = 0; k < nz_ ; k++) {
    
    auto dose_ave = dose_ave_[k] / events_;

    auto error = sqrt( sqrt(dose_sqr_[k] / events_
                            - dose_ave * dose_ave) / events_);
    
    statical_error_.push_back(error);

    G4cout << k << "," << error / MeV << G4endl;
    
  }
}

//-----------------------------------------------------------------------------
inline void ErrorCounter::SaveStaticalError() {
  
  std::ostringstream fname;
  std::ostringstream fname_sqring_x;
  std::ostringstream fname_average_x;

  fname << "statical_error_" << events_ << ".csv";
  fname_sqring_x << "sqring_x_" << events_ << ".csv";
  fname_average_x << "average_x_" << events_ << ".csv";

  std::ofstream error_file(fname.str(), std::ios::out);
  std::ofstream sqring_file(fname_sqring_x.str(), std::ios::out);
  std::ofstream average_file(fname_average_x.str(), std::ios::out);

  error_file << "z,cerror" << std::endl; 
  sqring_file << "z,dose" << std::endl;
  average_file << "z,dose" << std::endl;

  for (int k = 0; k < nz_; k++) {

    error_file << k << "," << statical_error_[k] / MeV << std::endl;
    sqring_file << k << "," << dose_sqr_[k] / MeV << std::endl;
    average_file << k << "," << dose_ave_[k] / MeV << std::endl;
  
  }

}