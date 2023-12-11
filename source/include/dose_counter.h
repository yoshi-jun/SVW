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
#ifndef DOSECOUNTER_H_
#define DOSECOUNTER_H_

#include <atomic>
#include <vector>
#include <mutex>

class DoseCounter{
private:
  DoseCounter() = default;
public:
  ~DoseCounter() = default;
  void Initialize(int x, int y, int z, int t_max, char* mode);

  DoseCounter(const DoseCounter&) = delete;
  DoseCounter& operator=(const DoseCounter&) = delete;

  static DoseCounter* GetInstance();

  void ScoreDose(int i, int j, int k, double dose);
  void ScoreDoseSpin(int i, int j, int k, double dose);
  void ScoreDoseMutex(int i, int j, int k, double dose);
  void ScoreDoseAtomic(int i, int j, int k, double dose);

  void SaveDose(const char* filename) const;

  void Marge();
  void SaveTotalDose(const char* filename);

  void SetMode(const char* mode);
  char* GetMode() const;

  void SaveRands(std::string rand) const;

  void CountDose(double dose, int tid);
  double GetTotalDose();

  void PrintTests();

  void SpinLock();
  void SpinUnlock();

  void ScoreDoubleDose(int i, int j, int k, double dose);
  void ScoreCount(int i, int j, int k);
  void SaveDoubleDose(std::string filename);
  
private:

  char* mode_;
  int nx_;
  int ny_;
  int nz_;
  int thmax_;

  std::atomic<bool> flag_;

  std::mutex mtx_;
  
  std::vector<double> dose_lis_;
  std::vector<double> total_dose_lis_;
  std::vector<double> total_dose_;
  std::vector<double> double_dose_;
  std::vector<double> particle_count_;
  // std::vector<std::atomic<double>> at_dose_counter_;
  
  std::atomic<double> hoge_;
};

//=============================================================================
inline char* DoseCounter::GetMode() const
{
  return mode_;
}

//-----------------------------------------------------------------------------
inline void DoseCounter::SetMode(const char* mode)
{
  *mode_ = *mode;
}

//-----------------------------------------------------------------------------
inline double DoseCounter::GetTotalDose()
{
  if (mode_ == "mt") {
    double dose = 0;
    // for (int i = 0; i < thmax_; i++) {
    //   std::cout << "th_num = " << i << " " << total_dose_[i] << std::endl;
    //   dose += total_dose_[i];

    // }
    return dose;

  } else {

    return total_dose_[0];
  }
}

//-----------------------------------------------------------------------------
inline void DoseCounter::SpinLock()
{
  while(flag_.exchange(true, std::memory_order_acquire)) {};
}

//-----------------------------------------------------------------------------
inline void DoseCounter::SpinUnlock()
{
  flag_.store(false, std::memory_order_release);
}
#endif

