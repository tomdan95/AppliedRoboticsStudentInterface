#include <fstream>
#include <iostream>
#include "Config.h"

using namespace std;
using namespace student;
using json = nlohmann::json;

Config::Config(string configFolder) {
    auto jsonConfig = loadJsonFile(configFolder);
    robotSize = jsonConfig["robotSize"];
    mission = jsonConfig["mission"];
    if (missionHasVictimBonus()) {
        victimBonus = jsonConfig["victimBonus"];
    }
    numberTemplatesFolder = jsonConfig["numberTemplatesFolder"];
}

json Config::loadJsonFile(string configFolder) {

    string fileName = configFolder + "/config.json";

    cout << "loading config file '" << fileName << "'" << endl;
    ifstream input(fileName);
    json jsonContent;
    input >> jsonContent;
    return jsonContent;
}
