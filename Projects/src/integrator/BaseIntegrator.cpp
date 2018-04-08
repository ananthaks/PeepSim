#include "BaseIntegrator.h"

BaseIntegrator::BaseIntegrator(std::string name) : mName(name) {}

const std::string& BaseIntegrator::name() const{
    return mName;
}
