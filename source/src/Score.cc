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
#include <fstream>
#include <iostream>
#include <thread>

#include "G4SystemOfUnits.hh"
#include "G4Threading.hh"
#include "dose_counter.h"

//------------------------------------------------------------------------------
void DoseCounter::Initialize(int x, int y, int z, int t_max, char* mode)
{
  nx_ = x; ny_ = y; nz_ = z;
  thmax_ = t_max;
  mode_ = mode;

  // For score dose * dose 
  double_dose_.resize(x * y * z * t_max, 0.0);
  particle_count_.resize(x * y * z * t_max, 0.00000001);

  if (mode_ == "mt") {

    dose_lis_.resize(x * y * z * t_max, 0.0);
    total_dose_.resize(t_max, 0.0);

    std::cout << "initialized mode mt" << std::endl;

  } else if (mode_ == "mutex") {

    dose_lis_.resize(x * y * z, 0.0);
    total_dose_.resize(1,0.0);

    std::cout << "initialized mode mutex" << std::endl;

  } else if (mode_ == "spin") {

    dose_lis_.resize(x * y * z, 0.0);
    total_dose_.resize(1, 0);

    flag_.store(false);

    std::cout << "initialized mode spin" << std::endl;

  } else if (mode_ == "atomic") {
    
    for (int i = 0; i < x * y * z; i++) {
      // at_dose_counter_.push_back(0.0);
    }
    total_dose_.resize(1, 0.0);
  
  } else {
    std::cout << "The mode setted is rong" << std::endl;
    std::cout << "mode_ " << mode_ << std::endl;
    std::exit(EXIT_FAILURE);
  }
}
//-----------------------------------------------------------------------------
DoseCounter* DoseCounter::GetInstance()
{
  static DoseCounter dose_counter;

  return &dose_counter;
}

//------------------------------------------------------------------------------
void DoseCounter::ScoreDose(int i, int j, int k, double dose)
{
  if (mode_ == "mt") {
    auto tid = G4Threading::G4GetThreadId();
    int idx = i + j * nx_ + k * nx_ * ny_ + tid * nx_ * ny_ * nz_;
    dose_lis_[idx] += dose;
  
  } else if(mode_ == "mutex") {
    if (dose != 0) {
      mtx_.lock();
      int idx = i + j * nx_ + k * nx_ * ny_;
      dose_lis_[idx] += dose;
      mtx_.unlock();
    }    
  
  } else if (mode_ == "spin") {
    if (dose != 0) {
    hoge_ = dose;
    SpinLock();
    int idx = i + j * nx_ + k * nx_ * ny_;
    dose_lis_[idx] += dose;
    SpinUnlock();
    }

  } else if (mode_ == "atomic") {
    int idx = i + j * nx_ + k * nx_ * ny_;

    // at_dose_counter_[idx] += dose;
  }
}

//------------------------------------------------------------------------------
void DoseCounter::ScoreDoseSpin(int i, int j, int k, double dose)
{
  if (dose != 0) {
    hoge_ = dose;
    SpinLock();
    int idx = i + j * nx_ + k * nx_ * ny_;
    dose_lis_[idx] += dose;
    SpinUnlock();
  }
}

//------------------------------------------------------------------------------
void DoseCounter::ScoreDoseMutex(int i, int j, int k, double dose)
{
  if (dose != 0) {
    mtx_.lock();
    int idx = i + j * nx_ + k * nx_ * ny_;
    dose_lis_[idx] += dose;
    mtx_.unlock();
  }
}

//------------------------------------------------------------------------------
void DoseCounter::ScoreDoseAtomic(int i, int j, int k, double dose)
{
  int idx = i + j * nx_ + k * nx_ * ny_;

  // at_dose_counter_[idx] += dose;
}

//------------------------------------------------------------------------------
void DoseCounter::SaveDose(const char* filename) const
{
  std::ofstream dose_file_(filename, std::ios::out);

  std::vector<double> result;
  result.resize(nx_ * ny_ * nz_, 0);

  dose_file_ << "tid,ix,iy,iz,dose" << std::endl;

  for (int k = 0; k < nz_; k++) {
    for (int j = 0; j < ny_; j++) {
      for (int i = 0; i < nx_; i++) {

        auto idx_t = i+ j * nx_ + k * nx_ * ny_;

        // Sum of all thread results
        for (int t = 0; t < thmax_; t++) {

          auto idx_all =  idx_t + t * nx_ * ny_ * nz_;

          result[idx_t] += dose_lis_[idx_all];

          dose_file_ << t << "," << i << "," << j << ","<< k << ","
                     << dose_lis_[idx_all] / MeV  <<std::endl;

        }
      }
    }
  }

  dose_file_.close();
}

//------------------------------------------------------------------------------
void DoseCounter::Marge()
{
  total_dose_lis_.resize(nx_ * ny_ * nz_, 0.);

  auto t_max = G4Threading::GetNumberOfRunningWorkerThreads();

  for (int k = 0; k < nz_; k++) {
    for (int j = 0; j < ny_; j++) {
      for (int i = 0; i < nx_; i++) {

        int idx = i + j * nx_ + k * nx_ * ny_;

        for (int t = 0; t < t_max; t++) {

          int idx_t = idx + t * nx_ * ny_ * nz_;

          total_dose_lis_[idx] += dose_lis_[idx_t];

        }
      }
    }
  }

}

//------------------------------------------------------------------------------
void DoseCounter::SaveTotalDose(const char* filename)
{
  
  std::ofstream dose_file(filename, std::ios::out);

  dose_file << "ix,iy,iz,dose" << std::endl;

  if (mode_ == "mt") {
    
    Marge();
    
    for (int k = 0; k < nz_; k++) {
      for (int j = 0; j < ny_; j++) {
        for (int i = 0; i < nx_; i++) {
          int idx = i + j * nx_ + k * nx_ * ny_;

          dose_file << i << "," << j << "," << k << ","
                  << total_dose_lis_[idx] / MeV << std::endl;

        }
      }
    }
  } else if (mode_ == "mutex" || mode_ == "spin") {

    for (int k = 0; k < nz_; k++) {
      for (int j = 0; j < ny_; j++) {
        for (int i = 0; i < nx_; i++) {
          int idx = i + j * nx_ + k * nx_ * ny_;

          dose_file << i << "," << j << "," << k << ","
                  << dose_lis_[idx] / MeV << std::endl;

        }
      }
    }
  } else if (mode_ == "atomic") {
    
    for (int k = 0; k < nz_; k++) {
      for (int j = 0; j < ny_; j++) {
        for (int i = 0; i < nx_; i++) {
          int idx = i + j * nx_ + k * nx_ * ny_;

          // dose_file << i << "," << j << "," << k << ","
          //         << at_dose_counter_[idx] / MeV << std::endl;

        }
      }
    }
  }
  dose_file.close();

}

//-----------------------------------------------------------------------------
void DoseCounter::SaveRands(std::string rand) const
{
  std::cout << rand << std::endl;
  if (mode_ == "mt") {

    std::ofstream rand_fp("rand-mt.txt", std::ios::out);
    rand_fp << rand << std::endl;
    rand_fp.close();
  }else if (mode_ == "atom") {

    std::ofstream rand_fp("rand-atom.txt", std::ios::out);
    rand_fp << rand << std::endl;
    rand_fp.close();
  }else if (mode_ == "mutex") {

    std::ofstream rand_fp("rand-mutex.txt", std::ios::out);
    rand_fp << rand << std::endl;
    rand_fp.close();
  }
}

//-----------------------------------------------------------------------------
void DoseCounter::CountDose(double dose, int tid)
{
  if (mode_ == "mt") {
    total_dose_[tid] += dose;

  } else if (mode_ == "mutex") {
    mtx_.lock();
    total_dose_[0] += dose;
    mtx_.unlock();

  } else if (mode_ == "atomic") {
    // std::atomic<double> after ver.c++20
    // hoge_ += dose;
  
  } else if (mode_ == "spin") {
    SpinLock();
    total_dose_[0] += dose;
    SpinUnlock();

  }
}

//-----------------------------------------------------------------------------
void DoseCounter::ScoreDoubleDose(int i, int j, int k, double dose)
{
  
  auto tid = G4Threading::G4GetThreadId();
  auto idx = i + j * nx_ + k * nx_ * ny_ + tid * nx_ * ny_ * nz_;
  double_dose_[idx] += dose * dose;

}

//-----------------------------------------------------------------------------
void DoseCounter::ScoreCount(int i, int j, int k)
{
  auto tid = G4Threading::G4GetThreadId();
  auto idx = i + j * nx_ + k * nx_ * ny_ + tid * nx_ * ny_ * nz_;
  particle_count_[idx] += 1;
}
//-----------------------------------------------------------------------------
void DoseCounter::SaveDoubleDose(std::string filename = "doubel_dose.csv")
{
  std::ofstream dose_file(filename, std::ios::out);
  std::ofstream count_file("counts.csv", std::ios::out);

  dose_file << "ix,iy,iz,dose" << std::endl;
  count_file << "ix,iy,iz,count" << std::endl;

  auto t_max = G4Threading::GetNumberOfRunningWorkerThreads();
  
  double marged_dose = 0.0;
  double marged_count = 0.0;

  for (int k = 0; k < nz_; k++) {
    for (int j = 0; j < ny_; j++) {
      for (int i = 0; i < nx_; i++) {

        int idx = i + j * nx_ + k * nx_ * ny_;
        
        for (int t = 0; t < t_max; t++) {

          int idx_t = idx + t * nx_ * ny_ * nz_;

          marged_dose += double_dose_[idx_t];
          marged_count += particle_count_[idx_t];

        }
        
        dose_file << i << "," << j << "," << k << "," 
                  << marged_dose << std::endl; 

        count_file << i << "," << j << "," << k << ","
                    << marged_count << std::endl;

        marged_dose = 0.0;
        marged_count = 0.0;

      }
    }
  }


}