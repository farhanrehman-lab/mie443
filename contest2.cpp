#include <boxes.h>
#include <navigation.h>
#include <robot_pose.h>
#include <imagePipeline.h>

#include <cmath>

double xGoal = 0;
double yGoal = 0;
double phiGoal = 0;
double PI = 1.570795;
double distanceArray[5][5];

int cost = 0;
int completed[5] = {0,0,0,0,0};

// int least (int c) {
//     int nc = 999;
//     int min = 999;
//     int kmin = 0;

//     for (int i = 0; i < 5; i++) {
//         if ((distanceArray[c][i]!=0)&&(completed[i]==0)) {
//             if (distanceArray[c][i] + distanceArray[i][c] < min) {
//                 min = distanceArray[i][0] + distanceArray[c][i];
//                 kmin = distanceArray[c][i];
//                 nc = i;
//             }
//         }
//     }

//     if(min!=999) {
//         cost += kmin;
//     }

//     return nc;
// }

int orderDestArray[7];
// int orderSize = 1;

// void minDistance (int city) {
//     std::cout << "city: " << city << "\n";
//     int ncity = 0;
//     int dest = 0;
//     completed[city] = 1;

//     ncity = least(city);
//     if (ncity = 999) {
//         ncity = 0;
//         dest = ncity + 1;
//         cost += distanceArray[city][ncity];
//         orderDestArray[orderSize] = dest;
//         orderSize ++; 
//         return;
//     }
//     orderDestArray[orderSize] = dest;
//     minDistance(ncity);
// }

// void tsp(int s) {
//     std::vector<int> vertex;
//     for (int i = 0; i < 6; i++) {
//         if (i!=s)
//             vertex.push_back(i);
//     }
    
//     int minpath = 999;
    
//     int current_pathweight = 0;

//     int k = s;
//     for (int i = 0; i < 6; i++) {
//         current_pathweight += distanceArray[k][vertex[i]];
//         k = vertex[i];
//         orderDestArray[orderSize] = k;
//         orderSize++;

//     }
//     current_pathweight += distanceArray[k][s];
//     minpath = std::min(minpath, current_pathweight);
    
// }

float distance(int x1, int y1, int x2, int y2){
    return sqrt(pow(x2-x1, 2)+pow(y2-y1, 2)*1.0);
}
float length;
minDist = 999;

void tsp() {
    for(a=1; a<5; a++){
        length = distance(destination[0][0], destination[0][1], destination[a][0], destination[a][1]);
        
        for(b=1; b<5; b++){
            if b=a { 
                continue; 
            }
            length += distance(destination[a][0], destination[a][1], destination[b][0], destination[b][1]);
            
            for(c=1; c<5; c++){
                if c=a || c=b { 
                    continue; 
                }
                length += distance(destination[b][0], destination[b][1], destination[c][0], destination[c]][1]);
                
                for(d=1; d<5; d++){
                    if d=a || d=b || d=c { 
                        continue; 
                    }
                    length += distance(destination[c][0], destination[c][1], destination[d][0], destination[d]][1]);
                
                    for(e=1; e<5; e++){
                        if e=a || e=b || e=c || e=d { 
                            continue; 
                        }
                        length += distance(destination[d][0], destination[d][1], destination[e][0], destination[e]][1]);
                        length += distance(destination[e][0], destination[e][1], destination[0][0], destination[0]][1]);

                        if length < minDist {
                            minDist = length;
                            // store minDist path and return path
                        }
                    }
                }
            }
        }
    }
}

int main(int argc, char** argv) {
    // Setup ROS.
    ros::init(argc, argv, "contest2");
    ros::NodeHandle n;
    //Robot pose object + subscriber.
    RobotPose robotPose(0,0,0);
    ros::Subscriber amclSub = n.subscribe("/amcl_pose", 1, &RobotPose::poseCallback, &robotPose);
    
    while (robotPose.x==0 && robotPose.y==0 && robotPose.phi==0){
        ros::spinOnce();
    }
    float startX = robotPose.x;
    float startY = robotPose.y;
    float startPhi = robotPose.phi;
    // // Initialize box coordinates and templates
    Boxes boxes; 
    if(!boxes.load_coords() || !boxes.load_templates()) {
        std::cout << "ERROR: could not load coords or templates" << std::endl;
        return -1;
    }
    for(int i = 0; i < boxes.coords.size(); ++i) {
        std::cout << "Box coordinates: " << std::endl;
        std::cout << i << " x: " << boxes.coords[i][0] << " y: " << boxes.coords[i][1] << " z: " 
                  << boxes.coords[i][2] << std::endl;
    }
    // Initialize image objectand subscriber.
    //ImagePipeline imagePipeline(n);
    // Execute strategy.
    
    while(ros::ok()) {
        ros::spinOnce();
        // /***YOUR CODE HERE***/
        
        //Set locations to travel to in order to take image
        float destination[5][3];
        destination[0][0] = startX;
        destination[0][1] = startY;
        destination[0][2] = startPhi;
        
        for(int i = 0; i < 5;i++) {
            phiGoal = boxes.coords[i][2] - 2*PI;
            xGoal = boxes.coords[i][0] - 0.7*cos(phiGoal);
            yGoal = boxes.coords[i][1] - 0.7*sin(phiGoal);
            destination[i+1][0] = xGoal;
            destination[i+1][1] = yGoal;
            destination[i+1][2] = phiGoal;
            //std::cout << destination[i+1][0]<<"," << destination[i+1][1]<<"," <<destination[i+1][2] << "\n";
            completed[i] = 0;
            //Navigation::moveToGoal(xGoal,yGoal,phiGoal);
        // //std::cout << i << " Goal: " << i << std:endl;
        }

        //Create array to store distance between all vertices
        for (int i = 0; i < 6; i++) {
            for (int j = 0; j < 6; j++) {
                distanceArray[i][j] = sqrt(((destination[j][0] - destination[i][0])*(destination[j][0] - destination[i][0])) + ((destination[j][1] - destination[i][1])*(destination[j][1] - destination[i][1])));
                // std::cout << i <<": "<<destination[i][0]<<", "<<destination[i][1]<<", ";
                // std::cout << j <<": "<<destination[j][0]<<", "<<destination[j][1]<<", ";
                // std::cout <<"distance between = " << distanceArray[i][j];
                // std::cout << "\n";
            }
        }

        // Store order to travel according to TSP
        tsp();
        int newDest = 0;
        for (int i = 1; i < 6; i++){
            orderDestArray[i] = newDest;
            //Navigation::moveToGoal(destination[newDest][0],destination[newDest][1],destination[newDest][2]);
            std::cout <<"BOX:" <<newDest<<","<<destination[newDest][0] << ","<< destination[newDest][1] << ","<< destination[newDest][2];
            std::cout << "\n";
            // imagePipeline.getTemplateID(boxes);
        }


        break;
    }

        //Navigation::moveToGoal(startX,startY,startPhi);
        //imagePipeline.getTemplateID(boxes);
        ros::Duration(0.01).sleep();
        
    
    return 0;
}
