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
#include <iostream>
#include <time.h>

class StopWatch
{
public:
  ~StopWatch() = default;

  StopWatch(const StopWatch&) = delete;
  StopWatch& operator=(const StopWatch&) = delete;

  static StopWatch* GetInstance();

  void Start();
  void Stop();

  void PrintResult(std::string comment) const;
private:
  StopWatch() = default;
  struct timespec start_time_;
  struct timespec stop_time_;
};

//=============================================================================
inline StopWatch* StopWatch::GetInstance()
{
  static StopWatch timer;

  return &timer;
}

inline void StopWatch::Start()
{
  clock_gettime(CLOCK_REALTIME, &start_time_);

  std::cout << start_time_.tv_sec << std::endl;
}

//-----------------------------------------------------------------------------
inline void StopWatch::Stop()
{
  clock_gettime(CLOCK_REALTIME, &stop_time_);

  std::cout << stop_time_.tv_sec << std::endl;
}

//-----------------------------------------------------------------------------
inline void StopWatch::PrintResult(std::string comment = "Exe time") const
{
  auto exe_time = (stop_time_.tv_sec - start_time_.tv_sec)
                  + (stop_time_.tv_nsec - start_time_.tv_nsec) / 1000000000;
  std::cout << comment << " : " << exe_time << "sec" << std::endl;
}
