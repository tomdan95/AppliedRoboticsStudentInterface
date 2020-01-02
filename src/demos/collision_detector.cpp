#include <iostream>
#include "../pipeline/planning/collision_detection/ShadowCollisionDetector.h"


using namespace std;
using namespace student;

int main () {
    
    Polygon rectangle;
    rectangle.emplace_back(10, 10);
    rectangle.emplace_back(10, 20);
    rectangle.emplace_back(20, 20);
    rectangle.emplace_back(20, 10);
    
    vector<Polygon> obstacles;
    obstacles.push_back(rectangle);
    CollisionDetector detector(obstacles);

    cout << detector.isPointInAnyObstacle(Point(5, 5)) << endl;
    cout << detector.isPointInAnyObstacle(Point(15, 15)) << endl;
    
    
}