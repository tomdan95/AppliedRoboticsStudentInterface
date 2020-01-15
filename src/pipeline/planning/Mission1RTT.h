#ifndef STUDENT_PROJECT_MISSION1RTT_H
#define STUDENT_PROJECT_MISSION1RTT_H

#include "MissionSolver.h"

namespace student {

    // TODO: Use C++ templates, so we cn use the graph in Graph.cpp
    class GraphD {
    public:
        map<RobotPosition *, vector<RobotPosition *>> edges;
        set<RobotPosition *> points;

        RobotPosition* addPoint(RobotPosition p) {
            /*for (auto *a : points) {
                if (a->x == p.x && a->y == p.y && a->t == p.t) {
                    return;
                }
            }*/
            auto *copy = new RobotPosition(p.x, p.y, p.theta);
            points.insert(copy);
            return copy;
        }

        void addEdge(RobotPosition* a, RobotPosition* b) {
            //TODO: check if edge already exist
            edges[a].push_back(b);
            edges[b].push_back(a);
        }

        double distance(RobotPosition a, RobotPosition b) {
            return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
        }

        RobotPosition* nearestPoint(RobotPosition p) {
            RobotPosition *near = NULL;
            for (auto *a: points) {
                if (near == NULL || distance(*near, p) > distance(*a, p)) {
                    near = a;
                }
            }
            return near;
        }
    };


    class Mission1RTT : public MissionSolver {

    public:
        boost::optional<vector<DubinsCurve>> solve() override;

        Mission1RTT(const student::CollisionDetector *collisionDetector, student::Graph *cleanestPaths,
                    const RobotPosition &start, const Point &gate, const vector<pair<int, Point>>& victims) : MissionSolver(collisionDetector,
                                                                                         cleanestPaths, start, gate,
                                                                                         victims) {}

    private:
        GraphD graph;

        RobotPosition getRandomPoint();
        bool allPointsReached();
    };
}


#endif //STUDENT_PROJECT_MISSION1RTT_H
