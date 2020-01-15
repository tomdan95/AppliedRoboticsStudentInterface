#ifndef STUDENT_PROJECT_CONFIG_H
#define STUDENT_PROJECT_CONFIG_H

#include <string>
#include <nlohmann/json.hpp>

using namespace std;
using json = nlohmann::json;

namespace student {

    class Config {

    private:
        double robotSize;
        int mission;
        int victimBonus;
        string numberTemplatesFolder;
        bool autoFindArenaEdges;
        double pruneThreshold;
        int bestThetaSteps;
        int maxK;
        double robotSpeed;

        static json loadJsonFile(string configFolder);

        bool missionHasVictimBonus() {return mission == 2; }

    public:
        explicit Config(string configFolder);

        double getRobotSize() { return robotSize; }

        int getMission() { return mission; }

        int getVictimBonus() {
            if(!missionHasVictimBonus()) {
                throw runtime_error("getVictimBonus() can be called only for mission 1");
            }
            return victimBonus;
        }

        string getNumberTemplatesFolder() {
            return numberTemplatesFolder;
        }

        bool getAutoFindArenaEdges() {
            return autoFindArenaEdges;
        }

        double getPruneThreshold () {
            return pruneThreshold;
        }

        int getMaxK(){
            return maxK;
        }

        int getBestThetaSteps(){
            return bestThetaSteps;
        }

        double getRobotSpeed(){
            return robotSpeed;
        }
    };
}


#endif
