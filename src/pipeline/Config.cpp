#include <fstream>
#include "Config.h"

using namespace std;
using namespace student;
using json = nlohmann::json;

Config::Config(string fileName) {
    auto jsonConfig = loadJsonFile(fileName);
    robotSize = jsonConfig["robotSize"];
    mission = jsonConfig["mission"];
    if (missionHasVictimBonus()) {
        victimBonus = jsonConfig["victimBonus"];
    }
}

json Config::loadJsonFile(string fileName) {
    ifstream input(fileName);
    json jsonContent;
    input >> jsonContent;
    return jsonContent;
}
