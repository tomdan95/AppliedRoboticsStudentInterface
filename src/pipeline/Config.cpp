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
    autoFindArenaEdges = jsonConfig["autoFindArenaEdges"];
    pruneThreshold = jsonConfig["pruneThreshold"];
}

string getConfigFileName(string configFolder) {
    if (configFolder[configFolder.length() - 1] == '/') {
        configFolder = configFolder.substr(0, configFolder.length() - 1);
    }
    return configFolder + "/config.json";
}

json Config::loadJsonFile(string configFolder) {
    string fileName = getConfigFileName(configFolder);
    cout << "loading config file '" << fileName << "'" << endl;
    ifstream input(fileName);
    json jsonContent;
    input >> jsonContent;
    return jsonContent;
}
