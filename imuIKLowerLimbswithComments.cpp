/* ------------------------------------------------------------------------- *
 *             OpenSim:  futureOrientationInverseKinematics.cpp               *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Dimitar Stanev                                                  *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

#include <OpenSim/OpenSim.h>
#include <Simbody.h>
#include <iostream>
#include <fstream>
#include <OpenSim/Common/MarkerData.h>
#include <OpenSim/Analyses/Kinematics.h>

#define DEBUG

//Making an array of strings [copies string into larger string]
char *convert(const std::string & s)
{
   char *pc = new char[s.size()+1];
   std::strcpy(pc, s.c_str());
   return pc; 
}

class OrientationIK
{
  std::string m_model_path; //"C:/opensim3.3/..."
  std::string m_imu_path;
  std::string m_ik_result_path;

  std::vector<std::string> m_body_names;
  std::vector<std::string> m_datafiles;

  int m_num_sensors;


  std::vector<SimTK::OrientationSensors::OSensorIx> m_sensors_mx; //instantiates vector of OSensors
  std::vector<SimTK::OrientationSensors::ObservationIx> m_sensors_ox; //'' but with ObservationIx
    
public:

  OrientationIK(
    std::string model_path,
    std::string imu_path,
    std::string result_path,
    std::vector<std::string> body_names,
    std::vector<std::string> datafiles //input all of these when you create OrientationIK class
    )
    : m_model_path(model_path),
      m_imu_path(imu_path),
      m_ik_result_path(result_path),
      m_body_names(body_names),
      m_datafiles(datafiles)  {
      m_num_sensors = body_names.size();//Number of bodies -- assume one sensor per body?
  }

  ~OrientationIK()
  {

  }

  SimTK::Mat33 readData(std::ifstream* ifile) {
    double a;
    *ifile >> a;

    SimTK::Mat33 Rot;
    *ifile >> Rot[0][0];
    *ifile >> Rot[1][0];
    *ifile >> Rot[2][0];
    *ifile >> Rot[0][1];
    *ifile >> Rot[1][1];
    *ifile >> Rot[2][1];
    *ifile >> Rot[0][2];
    *ifile >> Rot[1][2];
    *ifile >> Rot[2][2];
    return Rot;
  }

  /************************************************************************/
  /* Runs the inverse kinematics                                          */
  /************************************************************************/
  void run()
  {
    OpenSim::Model model(m_model_path);
    model.setUseVisualizer(true);

    // OpenSim::MarkerData imu_trc(m_imu_path);

    SimTK::State& state = model.initSystem(); //lmao classic(instantiate model)

    // setup
    SimTK::Assembler ik(model.updMultibodySystem());
    ik.setAccuracy(1e-5); //probably sets timestep/variance of the algorithm
    SimTK::Markers* markers = new SimTK::Markers(); //turning imus into markers
    SimTK::OrientationSensors* imus = new SimTK::OrientationSensors();

    SimTK::Visualizer& viz = model.updVisualizer().updSimbodyVisualizer();
    viz.setDesiredFrameRate(1000);
    viz.drawFrameNow(state); //makes the solving process visible

    // add markers
    addCustomMarkers(model, *markers, *imus); //markers is an empty array

    // result storage
    OpenSim::Kinematics kinematics(&model);
    kinematics.setRecordAccelerations(true); //flag to record accelerations while solving
 
 /*
   // local reference frame of the right tibia, femur, humerus, radius and hand   
    SimTK::Rotation rot_opensim_imu0(SimTK::BodyOrSpaceType::BodyRotationSequence,
              -SimTK::Pi/2, SimTK::ZAxis,
              0, SimTK::YAxis,
              0, SimTK::XAxis);
   // local reference frame of the left tibia, femur, humerus, radius and hand  
    SimTK::Rotation rot_opensim_imu1(SimTK::BodyOrSpaceType::BodyRotationSequence,   
            SimTK::Pi/2, SimTK::ZAxis,  
             -SimTK::Pi, SimTK::YAxis,
               0, SimTK::XAxis);
  */
   // local reference frame of the pelvis
    SimTK::Rotation rot_opensim_imu2(SimTK::BodyOrSpaceType::BodyRotationSequence, 
              -SimTK::Pi/2, SimTK::ZAxis, 
              SimTK::Pi/2, SimTK::XAxis,
               0, SimTK::XAxis);

    model.getMultibodySystem().realize(state, SimTK::Stage::Report); //tells us the current state of the system (?: details on functionality of realize)

    std::vector<SimTK::Rotation> rotations_osim; // initial rotation for each body part (in the opensim frame) 
    std::vector<SimTK::Rotation> rotations_initial; // initial rotation of each imu (in the xsens frame)
    std::vector<std::ifstream*> ifiles;

    for (int i=0; i < m_num_sensors; i++){
      const SimTK::Transform transform = model.getSimbodyEngine().getTransform(state, model.getBodySet().get(m_body_names[i].c_str()));
      rotations_osim.push_back(transform.R()); //adds rotation transformation to vector of rotations
    }

    for (int i=0; i < m_num_sensors; i++){
      ifiles.push_back(new std::ifstream(m_datafiles[i])); //indexes different datafiles
    }
  

    std::vector<SimTK::Rotation> rotations_opensim_imus = {rot_opensim_imu2}; //Setting Size

    //std::vector<SimTK::Rotation> rotations_opensim_imus = {rot_opensim_imu2, rot_opensim_imu0, rot_opensim_imu0, rot_opensim_imu1, rot_opensim_imu1, 
     // rot_opensim_imu0, rot_opensim_imu0, rot_opensim_imu1, rot_opensim_imu1, rot_opensim_imu0, rot_opensim_imu1};
     
    ik.adoptAssemblyGoal(imus); //solve to match these markers

    // Get the first observation and set things up
    std::string line;
    for (int i=0; i < m_num_sensors; i++){
        for(int j=0; j < 6; j++){
            getline(*ifiles[i],line);
        }
        SimTK::Mat33 rot_matrix_obs = readData(ifiles[i]); //
    
        rotations_initial.push_back(SimTK::Rotation(rot_matrix_obs)); 
        imus->moveOneObservation(m_sensors_ox[i], rotations_osim[i]); //what is msensorsox

        //imus->moveOneObservation(m_sensors_ox[i], rotations_opensim_imus[i] * rotations_initial[i].transpose() * SimTK::Rotation(rot_matrix_obs) * rotations_opensim_imus[i].transpose());  
     // important?
	}

    // setup inverse kinematics
    state.setTime(0.);
    ik.initialize(state);
    ik.assemble(state);
    kinematics.begin(state);

    viz.drawFrameNow(state);

    // Loop through all observations
    int iframe = 0;
    while(!(*ifiles[0]).eof())
      {
   
      iframe++;
       
      for (int i=0; i < m_num_sensors; i++){      
          SimTK::Mat33 rot_matrix_obs = readData(ifiles[i]);
         
          if(i == 0){ 
              imus->moveOneObservation(m_sensors_ox[i], rotations_opensim_imus[i] * rotations_initial[i].transpose() * SimTK::Rotation(rot_matrix_obs) * rotations_opensim_imus[i].transpose() * rotations_osim[i]);
           }else{
              SimTK::Mat33 rot1 = rotations_opensim_imus[0] * rotations_initial[0].transpose() * rotations_initial[i]; //body coordinate from stuff
              SimTK::Mat33 rot_imu_body = rot1.transpose() * rotations_osim[i]; //IMU rotation 
              imus->moveOneObservation(m_sensors_ox[i], rotations_opensim_imus[0] * rotations_initial[0].transpose() * SimTK::Rotation(rot_matrix_obs) * SimTK::Rotation(rot_imu_body));
			  //moveOneObservation is a function of object imus. Reads in rotation data and interates to next time step
           }
  }

  // Visualiztion - drawing the frames
  state.updTime() = iframe * 1./40.;
  ik.track(state.getTime());
  ik.updateFromInternalState(state);

  viz.drawFrameNow(state);
  // report
#ifdef DEBUG
  std::cout << "Frame: " << iframe << " (t=" << state.getTime() << ")\n";
  std::cout << "Error: " << ik.calcCurrentErrorNorm() << "\n";
  std::flush(std::cout);
#endif
  
  kinematics.step(state, iframe);
}//End of loop - repeat

    kinematics.end(state);

	//prints out results
    std::cout << "ik result path: " << m_ik_result_path << std::endl;


    kinematics.printResults(m_ik_result_path, "");

    for (int i=0; i < m_num_sensors; i++)
      delete ifiles[i];//deletes data in temporary ifiles
    
    ifiles.clear(); //Clears paths to ifile

  }

private:

  /************************************************************************/
  /* Adds the markers for the line test problem                           */
  /************************************************************************/
  void addCustomMarkers(OpenSim::Model& model,
      SimTK::Markers& markers, SimTK::OrientationSensors& imus)
  {

    double weight = 1; //??? weights in opt fx?
    // Add sensors
    for(int i=0; i < m_num_sensors; i++){
  SimTK::OrientationSensors::OSensorIx m_mx = imus.addOSensor(m_body_names[i],
               model.updBodySet().get(m_body_names[i]).getIndex(),
               SimTK::Rotation(SimTK::BodyOrSpaceType::BodyRotationSequence,
                   0, SimTK::ZAxis,
                   0, SimTK::YAxis,
                   0, SimTK::XAxis),
               weight);
      m_sensors_mx.push_back(m_mx);
    }

    // Match observations with sensors
    std::vector<char*> names;
    std::transform(m_body_names.begin(), m_body_names.end(), std::back_inserter(names), convert); 
   
	//ensures indexing of sensors and observation matrices match
    imus.defineObservationOrder(m_num_sensors, &names[0]);  
    for(int i=0; i < m_num_sensors; i++){
  m_sensors_ox.push_back(imus.getObservationIxForOSensor(m_sensors_mx[i]));
    }

  }
};


/**
 * Main function
 */
int main()
{
  try {
  
       
    std::vector<std::string> bodies = {
      "pelvis",
      "tibia_r",
      "femur_r",
      "tibia_l",
      "femur_l", 
      "humerus_r",
      "radius_r",
      "humerus_l",
      "radius_l",
      "hand_r",
      "hand_l"
    };
    
    std::vector<std::string> trackingFiles = {  
        "../walkingandturning2/MT_012005D6-000_walking_and_turning-000_00B4226B2.txt",
        "../walkingandturning2/MT_012005D6-000_walking_and_turning-000_00B4227D2.txt",
        "../walkingandturning2/MT_012005D6-000_walking_and_turning-000_00B4227C2.txt",
        "../walkingandturning2/MT_012005D6-000_walking_and_turning-000_00B421ED2.txt",
        "../walkingandturning2/MT_012005D6-000_walking_and_turning-000_00B421EE2.txt",
        "../walkingandturning2/MT_012005D6-000_walking_and_turning-000_00B422632.txt",
        "../walkingandturning2/MT_012005D6-000_walking_and_turning-000_00B4227B2.txt",
        "../walkingandturning2/MT_012005D6-000_walking_and_turning-000_00B4226C2.txt",
        "../walkingandturning2/MT_012005D6-000_walking_and_turning-000_00B422672.txt",
        "../walkingandturning2/MT_012005D6-000_walking_and_turning-000_00B422792.txt",
        "../walkingandturning2/MT_012005D6-000_walking_and_turning-000_00B4226D2.txt",
        "../walkingandturning2/MT_012005D6-000_walking_and_turning-000_00B421F32.txt"
    };

    OrientationIK ik(
         "Full_LegPelvisModel.osim",
         "futureOrientationInverseKinematics.trc",
         "futureOrientationInverseKinematics",
         bodies,
         trackingFiles
         );
    ik.run();

  }
  catch (const std::exception& ex)
    {
      std::cout << "Exception: " << ex.what() << std::endl;
      return 1;
    }
  catch (...)
    {
      std::cout << "Unrecognized exception " << std::endl;
      return 1;
    }

  return 0;
}