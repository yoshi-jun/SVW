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

#include "error_counter.h"
#include "master_run_action.h"
#include "dose_counter.h"
#include "json_reader.h"
#include "stop_watch.h"

//------------------------------------------------------------------------------
MasterRunAction::MasterRunAction()
{
}
//------------------------------------------------------------------------------
MasterRunAction::~MasterRunAction()
{
}

//------------------------------------------------------------------------------
void MasterRunAction::BeginOfRunAction(const G4Run *)
{
  auto timer = StopWatch::GetInstance();
  timer-> Start();

  std::cout << "000000000000000000000000000000000000000000000000000000000000000"
            << std::endl
            << "                             begin run" << std::endl
            << "000000000000000000000000000000000000000000000000000000000000000"
            << std::endl;
}

//------------------------------------------------------------------------------
void MasterRunAction::EndOfRunAction(const G4Run* aRun)
{
  G4cout << "#####coment###########" << G4endl;
  auto timer = StopWatch::GetInstance();
  timer-> GetInstance();
  timer-> Stop();
  timer-> PrintResult("Calculation time");

  auto reader = JsonReader::GetInstance();
  auto filename = reader-> GetFileName();
  auto event_num = reader-> GetEventNum();

  // auto error_counter = ErrorCounter::GetInstance();
  // error_counter-> MargeAllThreads();
  // error_counter-> CalculateStaticalError(event_num);
  // error_counter-> SaveStaticalError();
  // G4cout << "end save error" << G4endl;

  // auto scorer = DoseCounter::GetInstance();
  // double tedep = scorer-> GetTotalDose();
  // scorer-> SaveTotalDose(filename.c_str());
  // scorer-> SaveDoubleDose("test_double_dose.csv");

  // std::cout << "totall edeps =" << tedep << std::endl;

  std::cout << "000000000000000000000000000000000000000000000000000000000000000"
            << std::endl
            << "                            end run" << std::endl
            << "000000000000000000000000000000000000000000000000000000000000000"
            << std::endl;
}
