#include "simulatedFailures.h"
#include <cmath>
#ifdef CSIM
#include "esp32csim.h"
#endif

struct SimulatedFailureManager::FailSpec {
	const string name;
	float chance, duty, period, start;
};	
SimulatedFailureManager::SimulatedFailureManager() {}

void SimulatedFailureManager::addFailure(const string &name, float chance, float duty, float period, float start/*=0*/) { 
	FailSpec fs = {.name = name, .chance = chance, .duty = duty, .period = period, .start = start};
	failList.push_back(fs);
}
std::vector<std::string> split(const std::string &s, char delim);

void SimulatedFailureManager::addFailure(const string &spec) { 
	float chance = 0, duty = 1.0, period = 60, start = 0;
	float periodMin = 0, startMin = 0;
	vector<string> words = split(spec, '=');
	if (words.size() != 2) { 
		printf("bad fail spec '%s'", spec.c_str());
		return;
	} 
	string name = words[0];
	sscanf(words[1].c_str(),"%f,%f,%f,%f", &chance, &duty, &period, &start);
	sscanf(words[1].c_str(),"%f,%f,%f,%f", &chance, &duty, &periodMin, &startMin);
	if (periodMin > 0) period = periodMin * 60;
	if (startMin > 0) startMin = startMin * 60;
	addFailure(name, chance, duty, period, start);
}
bool SimulatedFailureManager::fail(const string &n) { 
#ifdef CSIM // TODO - figure out some way to share SimulatedFailureManager
            // between csim and esp32 builds 
	float now = (sim().bootTimeUsec + micros()) / 1000000.0;
#else
    float now = 0;
#endif
	for(auto f: failList) { 
		if(f.name == n) {
			if (now < f.start) 
				continue;
			float x = (now - f.start) / f.period;
			if (x - floor(x) > f.duty) 
				continue;
			if (rand() / (RAND_MAX + 1.0) < f.chance) 
				return true;
		}	
	}	
	return false;
}

SimulatedFailureManager &simFailures() { 
	static SimulatedFailureManager *firstStaticUse = new SimulatedFailureManager();
	return *firstStaticUse;
}

#ifdef CSIM
#include "esp32csim.h"
class CsimSimFailuresModule : public Csim_Module {
    void setup() override { 
        sim().simFailureHook = [](const char *f) { 
            return simFailures().fail(f); };
    }
    void parseArg( char **&a, char **) override {
        if (strcmp(*a, "--fail") == 0) simFailures().addFailure(*(++a));
    }
};
#endif
