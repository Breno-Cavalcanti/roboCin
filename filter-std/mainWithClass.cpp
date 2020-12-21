//author  Renato Sousa, 2018
#include <QtNetwork>
#include <stdio.h>
#include "net/robocup_ssl_client.h"
#include "net/grSim_client.h"
#include "util/timer.h"

#include "pb/messages_robocup_ssl_detection.pb.h"
#include "pb/messages_robocup_ssl_geometry.pb.h"
#include "pb/messages_robocup_ssl_wrapper.pb.h"
#include "pb/grSim_Packet.pb.h"
#include "pb/grSim_Commands.pb.h"
#include "pb/grSim_Replacement.pb.h"
#include<stdlib.h>
#include<unistd.h>
#include <time.h>
#include <math.h>

class FilterBall {
private:
    float temp;
    float lastPositionsX[2] = {0,0};
    float lastPositionsY[2] = {0,0};
    float StandardDeviationX;
    float StandardDeviationY;

public:
    float getX(int index) {
        if(index >= 2) {
            return 0;
        }

        return lastPositionsX[index];
    }

    float getY(int index) {
        if(index >= 2) {
            return 0;
        }

        return lastPositionsY[index];
    }

    float getStdX() {
        return StandardDeviationX;
    }

    float getStdY() {
        return StandardDeviationY;
    }

    void setLastX(float newPosition) {

        lastPositionsX[1] = newPosition;

        if(lastPositionsX[1] != 0) {

        temp=lastPositionsX[0];
        lastPositionsX[0]= lastPositionsX[1];
        lastPositionsX[1]=temp;
        }
    }

    void setLastY(float newPosition) {
        lastPositionsY[1] = newPosition;

        if(lastPositionsY[1] != 0) {

        temp=lastPositionsY[0];
        lastPositionsY[0] = lastPositionsY[1];
        lastPositionsY[1] = temp;
        }
    }



    float predictX() {

        float sum = 0;
        float mean = 0;
        float delta = 0;
        mean = (lastPositionsX[0] + lastPositionsX[1]) / 2;

        for(int i= 0; i < 2; i ++) {
            delta = (lastPositionsX[i] - mean);
            delta *= delta;
            sum += delta;
        }

        sum /= 2;

        StandardDeviationX = sqrt(sum);

        return lastPositionsX[0] + StandardDeviationX;
    }

    float predictY() {

        float sum = 0;
        float mean = 0;
        float delta = 0;
        mean = (lastPositionsY[0] + lastPositionsY[1]) / 2;

        for(int i= 0; i < 2; i ++) {
            delta = (lastPositionsY[i] - mean);
            delta *= delta;
            sum += delta;
        }

        sum /= 2;

        StandardDeviationY = sqrt(sum);

        return lastPositionsY[0] + StandardDeviationY;
    }

};


class FilterRobots {
private:
    float temp;
    float lastPositionsX[2] = {0,0};
    float lastPositionsY[2] = {0,0};
    int id;
    float StandardDeviationX;
    float StandardDeviationY;
    bool activate;
public:

    FilterRobots(int robotId) {
        id = robotId;
    }
    float getX(int index) {
        if(index >= 2) {
            return 0;
        }

        return lastPositionsX[index];
    }

    float getY(int index) {
        if(index >= 2) {
            return 0;
        }

        return lastPositionsY[index];
    }

    bool getActivate() {
        return activate;
    }

    float getStdX() {
        return StandardDeviationX;
    }

    float getStdY() {
        return StandardDeviationY;
    }

    void setLastX(float newPosition) { // função responsável por atualizar as posições(X) conhecidas
        activate = true;
        lastPositionsX[1] = newPosition;

        if(lastPositionsX[1] != 0) {

        temp=lastPositionsX[0];
        lastPositionsX[0]= lastPositionsX[1];
        lastPositionsX[1]=temp;
        }
    }

    void setLastY(float newPosition) { // função responsável por atualizar as posições(Y) conhecidas
        lastPositionsY[1] = newPosition;

        if(lastPositionsY[1] != 0) {

        temp=lastPositionsY[0];
        lastPositionsY[0] = lastPositionsY[1];
        lastPositionsY[1] = temp;
        }
    }



    float predictX() {

        float sum = 0;
        float mean = 0;
        float delta = 0;
        mean = (lastPositionsX[0] + lastPositionsX[1]) / 2;

        for(int i= 0; i < 2; i ++) {
            delta = (lastPositionsX[i] - mean);
            delta *= delta;
            sum += delta;
        } // for para calcular a variância

        sum /= 2;

        StandardDeviationX = sqrt(sum); // calculo do desvio padrão

        return lastPositionsX[0] + StandardDeviationX;
    }

    float predictY() {

        float sum = 0;
        float mean = 0;
        float delta = 0;
        mean = (lastPositionsY[0] + lastPositionsY[1]) / 2;

        for(int i= 0; i < 2; i ++) {
            delta = (lastPositionsY[i] - mean);
            delta *= delta;
            sum += delta;
        } // for para calcular a variância


        sum /= 2;

        StandardDeviationY = sqrt(sum); // calculo do desvio padrão


        return lastPositionsY[0] + StandardDeviationY;
    }

};


FilterRobots robots[11] = {0, 1, 2, 3, 4, 5 ,6 ,7 ,8, 9, 10 };
FilterBall BallFilter;

void update_csv_file(float realPosition, float predictPosition, int id){
    FILE *fp;
    fp=fopen("test_std_with_vanishing_class.csv","r+");
    fseek(fp, 0, SEEK_END);
    fprintf(fp,"\n%9.2f,%9.2f,%d",realPosition, predictPosition, id);
    fclose(fp);
}



void printRobotInfo(const SSL_DetectionRobot & robot,int id) {


    float lastPosition = robot.x();
    robots[id].setLastX(lastPosition);

    printf("CONF=%4.2f ", robot.confidence());

    if (robot.has_robot_id()) {
        printf("ID=%3d ",robot.robot_id());
    } else {
        printf("ID=N/A ");
    }
    if(robots[id].getActivate() == true) {
        update_csv_file(robot.x(),robots[id].predictX(), id);

    }

    printf(" HEIGHT=%6.2f POS=<%9.2f,%9.2f> ",robot.height(),robot.x(), robot.y());
    if (robot.has_orientation()) {
        printf("ANGLE=%6.3f ",robot.orientation());
    } else {
        printf("ANGLE=N/A    ");
    }



    printf("RAW=<%8.2f,%8.2f>\n",robot.pixel_x(),robot.pixel_y());



}
int timeFactor;

int main(int argc, char *argv[]){
    (void)argc;
    (void)argv;
    RoboCupSSLClient client;
    client.open(true);
    SSL_WrapperPacket packet;

    GrSim_Client grSim_client;

    while(true) {
        if (client.receive(packet)) {
            printf("-----Received Wrapper Packet---------------------------------------------\n");
            //see if the packet contains a robot detection frame:
            if (packet.has_detection()) {
                SSL_DetectionFrame detection = packet.detection();
                //Display the contents of the robot detection results:
                double t_now = GetTimeSec();

                printf("-[Detection Data]-------\n");
                //Frame info:
                printf("Camera ID=%d FRAME=%d T_CAPTURE=%.4f\n",detection.camera_id(),detection.frame_number(),detection.t_capture());

                printf("SSL-Vision Processing Latency                   %7.3fms\n",(detection.t_sent()-detection.t_capture())*1000.0);
                printf("Network Latency (assuming synched system clock) %7.3fms\n",(t_now-detection.t_sent())*1000.0);
                printf("Total Latency   (assuming synched system clock) %7.3fms\n",(t_now-detection.t_capture())*1000.0);
                int balls_n = detection.balls_size();
                int robots_blue_n =  detection.robots_blue_size();
                //int robots_yellow_n =  detection.robots_yellow_size();

                //Ball info:
                printf("%d\n\n",robots_blue_n);
                for (int i = 0; i < balls_n; i++) {
                    SSL_DetectionBall ball = detection.balls(i);
                    printf("-Ball (%2d/%2d): CONF=%4.2f POS=<%9.2f,%9.2f> ", i+1, balls_n, ball.confidence(),ball.x(),ball.y());

                    BallFilter.setLastX(ball.x());
                    BallFilter.setLastY(ball.y());

                    if (ball.has_z()) {
                        printf("Z=%7.2f ",ball.z());
                    } else {
                        printf("Z=N/A   ");
                    }
                    printf("RAW=<%8.2f,%8.2f>\n",ball.pixel_x(),ball.pixel_y());
                }

                if (balls_n == 0) {
                    timeFactor ++;
                    for(int i = 0; i < 11; i++) {
                        float nextValueX;
                        float nextValueY;
                        if(BallFilter.getX(0) != 0 && BallFilter.getX(0) != 0) {
                            nextValueX = robots[i].getX(0) + (timeFactor * BallFilter.getStdX());
                            BallFilter.setLastX(nextValueX);

                            nextValueY = BallFilter.getY(0) + (timeFactor * BallFilter.getStdY());
                        }



                    }

                }

                //Blue robot info:

                for (int i = 0; i < robots_blue_n; i++) {
                    SSL_DetectionRobot robot = detection.robots_blue(i);
                    timeFactor = 0;
                    printf("-Robot(B) (%2d/%2d): ",i+1, robots_blue_n);
                    printRobotInfo(robot, i);
                    if(robot.x() <= 0){
                        grSim_client.sendCommand(1.0, i);
                    }else{
                        grSim_client.sendCommand(-1.0, i);
                    }
                }

                if (robots_blue_n == 0) {
                    timeFactor ++;
                    for(int i = 0; i < 11; i++) {
                        float nextValueX;
                        float nextValueY;
                        if(robots[i].getX(0) != 0 && robots[i].getX(0) != 0 && robots[i].getActivate()) {
                            nextValueX = robots[i].getX(0) + (timeFactor * robots[i].getStdX()); // aplicação da penalidade em X
                            robots[0].setLastX(nextValueX); // atualizando a posição X com o valor previsto com a penalidade

                            nextValueY = robots[i].getY(0) + (timeFactor * robots[i].getStdY()); // aplicação da penalidade em Y
                            update_csv_file(0, robots[i].predictX(), i); // atualizando a posição Y com o valor previsto com a penalidade
                        }



                    }

                }
                /*
                //Yellow robot info:
                for (int i = 0; i < robots_yellow_n; i++) {
                    SSL_DetectionRobot robot = detection.robots_yellow(i);
                    printf("-Robot(Y) (%2d/%2d): ",i+1, robots_yellow_n);
                    printRobotInfo(robot);
                }
                */
            }



            //see if packet contains geometry data:
            if (packet.has_geometry()) {
                const SSL_GeometryData & geom = packet.geometry();
                printf("-[Geometry Data]-------\n");

                const SSL_GeometryFieldSize & field = geom.field();
                printf("Field Dimensions:\n");
                printf("  -field_length=%d (mm)\n",field.field_length());
                printf("  -field_width=%d (mm)\n",field.field_width());
                printf("  -boundary_width=%d (mm)\n",field.boundary_width());
                printf("  -goal_width=%d (mm)\n",field.goal_width());
                printf("  -goal_depth=%d (mm)\n",field.goal_depth());
                printf("  -field_lines_size=%d\n",field.field_lines_size());
                printf("  -field_arcs_size=%d\n",field.field_arcs_size());

                int calib_n = geom.calib_size();
                for (int i=0; i< calib_n; i++) {
                    const SSL_GeometryCameraCalibration & calib = geom.calib(i);
                    printf("Camera Geometry for Camera ID %d:\n", calib.camera_id());
                    printf("  -focal_length=%.2f\n",calib.focal_length());
                    printf("  -principal_point_x=%.2f\n",calib.principal_point_x());
                    printf("  -principal_point_y=%.2f\n",calib.principal_point_y());
                    printf("  -distortion=%.2f\n",calib.distortion());
                    printf("  -q0=%.2f\n",calib.q0());
                    printf("  -q1=%.2f\n",calib.q1());
                    printf("  -q2=%.2f\n",calib.q2());
                    printf("  -q3=%.2f\n",calib.q3());
                    printf("  -tx=%.2f\n",calib.tx());
                    printf("  -ty=%.2f\n",calib.ty());
                    printf("  -tz=%.2f\n",calib.tz());

                    if (calib.has_derived_camera_world_tx() && calib.has_derived_camera_world_ty() && calib.has_derived_camera_world_tz()) {
                      printf("  -derived_camera_world_tx=%.f\n",calib.derived_camera_world_tx());
                      printf("  -derived_camera_world_ty=%.f\n",calib.derived_camera_world_ty());
                      printf("  -derived_camera_world_tz=%.f\n",calib.derived_camera_world_tz());
                    }

                }
            }
        }


     }

    return 0;
}
