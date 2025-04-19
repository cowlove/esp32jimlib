#include <vector>
#include <string>

using std::vector;
using std::string;

// TODO figure out how to use this from ESP32 code without cluttering up esp32csim.h
class SimulatedFailureManager {
	struct FailSpec;
	vector<FailSpec> failList;
public:
	SimulatedFailureManager();
	void addFailure(const string &spec);
	void addFailure(const string &name, float chance, float duty, float period, float start=0);
	bool fail(const string &name);
};

SimulatedFailureManager &simFailures();
#define SIMFAILURE(x) simFailures().fail(x)
