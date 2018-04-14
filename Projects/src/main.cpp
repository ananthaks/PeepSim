#include "globalincludes.h"
#include "CrowdSim.h"

#ifndef DISABLE_PARTIO
template <class T, int dim>
void checkIfPartioWorks() {

    std::string particleFile = "test.bgeo";

    Partio::ParticlesDataMutable* parts = Partio::create();
    Partio::ParticleAttribute posH, vH, mH;
    mH = parts->addAttribute("m", Partio::VECTOR, 1);
    posH = parts->addAttribute("position", Partio::VECTOR, 3);
    vH = parts->addAttribute("v", Partio::VECTOR, 3);
    for (int i=0; i<3; i++){
        int idx = parts->addParticle();
        float* m = parts->dataWrite<float>(mH, idx);
        float* p = parts->dataWrite<float>(posH, idx);
        float* v = parts->dataWrite<float>(vH, idx);
        m[0] = (T)(i + 1);
        for (int k = 0; k < 3; k++)
            p[k] = (T)(i + 1);
        for (int k = 0; k < 3; k++)
            v[k] = (T)(i + 100);
    }

    Partio::write(particleFile.c_str(), *parts);
    parts->release();

    std::cout << "<<<< OK PARTIO LOADED SUCCESSFULLY >>>> " << std::endl;

}
#endif


int main() {

#ifdef TEST_PARTIO
    checkIfPartioWorks<float, 3>();
#endif

  PeepSimConfig config;
  config.create();

  CrowdSim simulator = CrowdSim(config);
  simulator.loadSceneFromFile("./scenes/scene_6.json");
  auto results = simulator.evaluate();
  std::cout << "<<<< CROWD SIMULATION FINISHED >>>> " << std::endl;
}
