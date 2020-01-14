#include "Graph.h"
#include "utils.h"
#include "/dubins/dubins.h"
#include "/dubins/models.h"
#include "collision_detection/ShadowCollisionDetector.h"
using namespace std;

//graph
struct PointD{
    float x,y,t;
    PointD(float x_, float y_, float t_){
        x=x_
        y=y_
        t=t_
    }
}

class GraphD {
    public:
        map<PointD*, vector<PointD*>> edges;
        set<PointD*> points;
        void addPoint(PointD p){
            for (auto *a : points) {
                if (a->x == p.x && a->y == p.y && a->t == p.t) {
                    return;
                }
            }
            auto *copy = new PointD(p.x, p.y, p.t);
            points.insert(copy);
        }
        void addEdge(PointD a, PointD b){
            //TODO: check if edge already exist
            edges[a].push_back(b);
            edges[b].push_back(a);
        }
        double dist(PointD a, PointD b){
            return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
        }
        PointD nearestPoint(PointD p){
            PointD near = NULL;
            for (auto *a: points){
                if(near==NULL){
                    near = a;
                }
                else if (dist(near,p)>dist(a,p)){
                    near=a;
                }
            }
            return near;
        }
}

vector<PointD *> rrt(const student::CollisionDetector *collisionDetector, PointD start, double kMax){
    GraphD graph = new GraphD();
    graph.addPoint(start);
    bool keepGoing = true;
    while(keepGoing){
        //random point
        int t = (rand()%7)+1
        float LOx, HIx, LOy, HIy;
        float x = LOx + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(HIx-LOx)));
        float y = LOy + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(HIy-LOy)));
        PointD currentP = new PointD(x,y,t);
        
        //check if in obstacle
        if (!collisionDetector.isPointInAnyObstacle(new Point::Point(currentP.x,currentP.y))){
            //find near point
            PointD nearP = graph.nearestPoint(currentP);
            //generate dubins path
            vector<DubinsCurve> dubins = dubinsShortestPath(new RobotPosition(new Point::Point(currentP.x,currentP.y),currentP.t), new Point::Point(nearP.x,nearP.y),nearP.t),kMax);
            //check dubbins collison
            if(!collisionDetector.doesCurveCollide(dubins))
                graph.addPoint(currentP);
                graph.addEdge(nearP, currentP);
                //check if is in victim
            }
        }
    }
}