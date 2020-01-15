
#include "Mission2.h"
#include "best_theta/BestThetaFinder.h"

using namespace std;

void generate(vector<vector<pair<int, Point>>>& permutations, vector<pair<int, Point>> chosenVictims, vector<pair<int, Point>> victimsToChoose) {
    // choose no other victim
    permutations.push_back(chosenVictims);

    if (victimsToChoose.empty()) {
        return;
    }

    for(auto v : victimsToChoose) {
        vector<pair<int, Point>> chosenVictimCopy;
        chosenVictimCopy.insert(chosenVictimCopy.end(), chosenVictims.begin(), chosenVictims.end());
        chosenVictimCopy.push_back(v);

        vector<pair<int, Point>> victimsToChooseCopy;
        for (auto toInsert : victimsToChoose) {
            if (toInsert.first != v.first) {
                victimsToChooseCopy.push_back(toInsert);
            }
        }

        generate(permutations, chosenVictimCopy, victimsToChooseCopy);
    }
}



namespace student {

    boost::optional<vector<DubinsCurve>> Mission2::solve() {
        auto permutations = generatePermutations();

        double shortestLength = INFINITY;
        boost::optional<vector<DubinsCurve>> shortestSolution;

        for (int i = 0; i < permutations.size(); i++){
            cout << "[MISSION-2] Generating path " << i << " / " << permutations.size() << endl;
            const auto& permutation = permutations[i];
            vector<Point> victimPoints;
            for (const auto v:permutation) {
                victimPoints.push_back(v.second);
            }
            addRobotVictimsAndGateToCleanestPathsGraph(victimPoints);
            computeShortestPath();
            prunePath(&shortestPath, toReach, pruneThreshold);
            auto solution = finder.findBestDubinsCurves(shortestPath);

            if (solution) {
                double length = 0;
                for (const auto curve : *solution) {
                    length += curve.length();
                }
                if(length < shortestLength) {
                    shortestLength = length;
                    shortestSolution = solution;
                }
            }
        }


        return shortestSolution;
    }

    vector<vector<pair<int, Point>>> Mission2::generatePermutations() {
        vector<vector<pair<int, Point>>> permutations;
        vector<pair<int, Point>> chosen;

        generate(permutations, chosen, victims);
        return permutations;
    }
}
