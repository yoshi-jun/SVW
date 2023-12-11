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
#include <fstream>
#include <getopt.h>
#include <math.h>

#include "G4MTRunManager.hh"
#include "G4RunManager.hh"
#include "G4UIExecutive.hh"
#include "G4VisExecutive.hh"
#include "QGSP_BIC.hh"
#include "QGSP_BIC_HP.hh" // contein option 4 EM phys
#include "Randomize.hh"

#include "action_initialization.h"
#include "em_phys_pcut.h"
#include "emstd_physlist.h"
#include "error_counter.h"
#include "json_reader.h"
#include "dose_counter.h"
#include "json.hpp"
#include "stop_watch.h"
#include "geometry.h"
//-----------------------------------------------------------------------------
namespace {

void show_help()
{
  const char* message =
R"(
usage:
Application_Main [options]

  -h, --help                show this message
  -n, --number              set number of threads
  -c, --conf <filename>     set config file
  -m, --mt                  normal multithread mode
  -a, --atom                use atomic for thread safe
  -x, --mutex               use mutex for thread safe
  -s, --spin                use spin lock for thread safe

config file default
  {
  "thread_num" : 2,
  "beam_flag" : "pencil",
  "particle" : "e-",
  "kine_energy" : 20.0,
  "beam_direc_x": 0.0,
  "beam_direc_y": 0.0,
  "beam_direc_z": 1.0,
  "dim_x" : 61,
  "dim_y" : 61,
  "dim_z" : 150,
  "rand_seed" : 123456789,
  "ssd" : 100,
  "file_name" : "test.csv"
  }
)";

  std::cout << message << std::endl;
}
//-----------------------------------------------------------------------------
  char* mode_name = "mt";

  auto counter = DoseCounter::GetInstance();
  auto reader = JsonReader::GetInstance();
  // default simulation parameter setting
  std::string beam_flag = "pencil";
  std::string particle_name = "geantino";
  std::string file_name = "test.csv";
  auto particle_enegy = 20.0;
  auto beam_direc_x = 0.0;
  auto beam_direc_y = 0.0;
  auto beam_direc_z = 1.0;
  auto dim_nx = 61;
  auto dim_ny = 61;
  auto dim_nz = 150;
  auto seed_rand = 123456789;
  auto ssd = 100;
  auto th_num = 16;
  auto single_flag = false;
  auto vis_mode = false;

} // end of namespace

//=============================================================================
int main(int argc, char** argv)
{
  // setting option
  const char* optstring = "hc:mxasn:r:z:fo:";
  const struct option long_options[] = {
  {"help"  ,  no_argument,       0, 'h'},
  {"conf"  ,  required_argument, 0, 'c'},
  {"multi" ,  no_argument,       0, 'm'},
  {"mutex" ,  no_argument,       0, 'x'},
  {"atomic",  no_argument,       0, 'a'},
  {"spin"  ,  no_argument,       0, 's'},
  {"number",  required_argument, 0, 'n'},
  {"random",  required_argument, 0, 'r'},
  {"dim_z" ,  required_argument, 0, 'z'},
  {"single",  required_argument, 0, 'f'},
  {"dfname",  required_argument, 0, 'o'},
  {"vis"   ,  required_argument, 0, 'v'},
  {0, 0,  0,  0},
  };

  std::string config_name = "";
  while (1) {

    int option_index = -1;
    int c = getopt_long(argc, argv, optstring, long_options, &option_index);

    if (c == -1) {
      break;
    }

    switch (c) {
      case 'h' :
        ::show_help();
        std::exit(EXIT_SUCCESS);
      case 'c' :
        config_name = static_cast<std::string>(optarg);
        break;
      case 'm' :
        ::mode_name = "mt";
        break;
      case 'x' :
        ::mode_name = "mutex";
        break;
      case 'a' :
        ::mode_name = "atomic";
        break;
      case 's' :
        ::mode_name = "spin";
        break;
      case 'n' :
        ::th_num = std::stoi(optarg);
        break;
      case 'r' :
        ::seed_rand = std::stoi(optarg);
        break;
      case 'z' :
        ::dim_nz = std::stoi(optarg);
        break;
      case 'f' :
        ::single_flag = true;
        break;
      case 'o' :
        ::file_name = static_cast<std::string>(optarg);
        break;
      case 'v' :
        ::vis_mode = true;
        break;

    }
  }
  //---------------------------------------------------------------------------
  //setting json element
  std::ifstream fin(config_name);

  if (fin) {
    nlohmann::json js;
    std::cout << "read config file " << std::endl;
    fin >> js;
    ::th_num = js["thread_num"];
    ::beam_flag = js["beam_flag"].get<std::string>();
    ::particle_name = js["particle"].get<std::string>();
    ::particle_enegy = js["kine_energy"];
    // ::beam_direc_x = js["beam_direc_x"];
    // ::beam_direc_y = js["beam_direc_y"];
    // ::beam_direc_z = js["beam_direc_z"];
    ::dim_nx = js["dim_x"];
    ::dim_ny = js["dim_y"];
    ::dim_nz = js["dim_z"];
    ::seed_rand = js["rand_seed"];
    ::ssd = js["ssd"];
    ::file_name = js["file_name"].get<std::string>();
  }

  //  Set boxcell numeric x,y,z
  ::reader-> SetVoxelNum(::dim_nx, ::dim_ny, ::dim_nz);

  //  Set beam name
  ::reader-> SetChooser(::beam_flag);

  // Set particle info
  ::reader-> SetParticleName(::particle_name);
  ::reader-> SetParticleEne(::particle_enegy);

  // Set beam direction
  ::reader-> SetBeamDirec(::beam_direc_x, ::beam_direc_y, ::beam_direc_z);

  // Set ssd [cm] order
  ::reader-> SetSSD(::ssd);

  ::reader-> SetFileName(::file_name);


  //--------------------------------------------------------------------------

  // Initialize counter
  ::counter-> Initialize(::dim_nx, ::dim_ny, ::dim_nz, ::th_num, ::mode_name);

  auto error_counter = ErrorCounter::GetInstance();
  error_counter-> Initialize(::dim_nx, ::dim_ny, ::dim_nz, ::th_num);

  // use MTwistEngine or default RNGEngine
  if(0) {
    auto engine = new CLHEP::MTwistEngine(12345);
    G4Random::setTheEngine(engine);
  }

  G4Random::setTheSeed(::seed_rand);

  // Define the run manager
  auto run_manager = new G4MTRunManager();

  // Set number of thread
  run_manager-> SetNumberOfThreads(::th_num);

  // Set Geometry
  run_manager-> SetUserInitialization(new Geometry);
  std::cout << "Success Initialize Geometry" <<std::endl;
  // Set physics list
    // predefined physics list
  // run_manager-> SetUserInitialization(new QGSP_BIC);
  run_manager-> SetUserInitialization(new EmstdPhysList);
  // run_manager-> SetUserInitialization(new EmPhysPcut);
  // run_manager-> SetUserInitialization(new QGSP_BIC_HP);

  // Set User action initialization
  std::cout << "Success Initialize Physics List" <<std::endl;

  run_manager-> SetUserInitialization(new ActionInitialization);
  std::cout << "Success Initialize Actions" <<std::endl;

  // Initialize run manager
  run_manager-> Initialize();
  std::cout << "Success define RunManager" <<std::endl;

  // UI executive mode
  if (optind < argc) {

    std::cout << "mode = " << ::mode_name << std::endl
              << "Threads = " << ::th_num << std::endl
              << "rand_seed = " << ::seed_rand << std::endl;

    auto event_num = argv[optind];
    reader-> SetEventNum(std::stoi(event_num));
    run_manager-> BeamOn(std::stoi(event_num));

  } else {
    auto vis_manager = new G4VisExecutive{};
    vis_manager->Initialize();

    auto ui_exec = new G4UIExecutive( argc, argv );
    ui_exec-> SessionStart();

    delete ui_exec;
    delete vis_manager;
  }

  delete run_manager;

  std::exit(EXIT_SUCCESS);
}

