#include <boxes.h>
#include <navigation.h>
#include <robot_pose.h>
#include <imagePipeline.h>

#include <cmath>

double xGoal = 0;
double yGoal = 0;
double phiGoal = 0;

double distanceArray[6][6];
int orderDestArray[5];
int orderCount = 0;
int visited[5] = {0,0,0,0,0};

double revxGoal = 0;
double revyGoal = 0;
double revphiGoal = 0;
double PI = 1.570795;

void tsp(int start) {
    double minpath = 999; 
    int nextbox = 0;
    
    for (int i = 1; i < 6; i++) {
        if ((distanceArray[start][i] < minpath) && (distanceArray[start][i] != 0) && (visited[i] == 0)) {
            minpath = distanceArray[start][i];
            nextbox = i;
            //visited[i] = 1;
        }
        
    // if ( nextbox == 0)
    // visited[nextbox] = 0;
    }
    visited[nextbox] = 1;
    orderDestArray[orderCount] = nextbox;
    orderCount++;
    std::cout <<"box"<< nextbox<<std::endl;
    if (orderCount == 5) {
        return;
    }
    
    tsp(nextbox);
    
}

int main(int argc, char** argv) {
    // Setup ROS.
    ros::init(argc, argv, "contest2");
    ros::NodeHandle n;
    // Robot pose object + subscriber.
    RobotPose robotPose(0,0,0); 
    ros::Subscriber amclSub = n.subscribe("/amcl_pose", 1, &RobotPose::poseCallback, &robotPose);
    
    while (robotPose.x==0 && robotPose.y==0 && robotPose.phi==0){
        ros::spinOnce();
    }
    float startX = robotPose.x;
    float startY = robotPose.y;
    float startPhi = robotPose.phi;
    
    // Initialize box coordinates and templates
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
    ImagePipeline imagePipeline(n);
    // Execute strategy.
    
    //Navigation::moveToGoal(-1.404,2.5,-1.606);
    while(ros::ok()) {
        ros::spinOnce();
        /***YOUR CODE HERE***/

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
            std::cout << destination[i+1][0]<<"," << destination[i+1][1]<<"," <<destination[i+1][2] << "\n"<< std::endl;
            
            //Navigation::moveToGoal(xGoal,yGoal,phiGoal);
        // //std::cout << i << " Goal: " << i << std:endl;
        }

        //Create array to store distance between all vertices
        for (int i = 0; i < 6; i++) {
            for (int j = 0; j < 6; j++) {
                distanceArray[i][j] = sqrt(((destination[j][0] - destination[i][0])*(destination[j][0] - destination[i][0])) + ((destination[j][1] - destination[i][1])*(destination[j][1] - destination[i][1])));
                //std::cout << i <<": "<<destination[j][0]<<", "<<destination[j][1]<<", "<< std::endl;
                
                // std::cout << j <<": "<<destination[j][0]<<", "<<destination[j][1]<<", ";
                std::cout <<"distance between = " << distanceArray[i][j] << std::endl;
                
                // std::cout << "\n";
            }
        }

        // Store order to travel according to TSP
        tsp(0);
        int newDest = 0;
        for (int i = 0; i < 5; i++){
            newDest = orderDestArray[i];
            Navigation::moveToGoal(destination[newDest][0],destination[newDest][1],destination[newDest][2]);
            std::cout <<"BOX:" <<newDest<<","<<destination[newDest][0] << ","<< destination[newDest][1] << ","<< destination[newDest][2] <<std::endl;
            std::cout << "\n" << std::endl;
            imagePipeline.getTemplateID(boxes);
        }

        
        
        Navigation::moveToGoal(startX,startY,startPhi);
        
        ros::Duration(0.01).sleep();
        break;
        }

        
    
    return 0;
}
