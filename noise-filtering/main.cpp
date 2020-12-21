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
    float lastPositionsX[3] = {0, 0, 0};
    float lastPositionsY[3] = {0, 0, 0};
    float sum = 0;

public:

    void setLastX(float newPosition) { // Função responsável por atualizar os valores de X

        for(int i = 3; i > 0 ; i-- ) {
            lastPositionsX[i] = lastPositionsX[i-1];
        }
        lastPositionsX[0] = newPosition;
    }

    float getX(int index) { // Função responsável por retornar o valor da posição desejada
        if(index >= 3) {
            return NULL;
        }
        return lastPositionsX[index];
    }

    void setLastY(float newPosition) { // Função responsável por atualizar os valores de X
        for(int i = 3; i > 0 ; i-- ) {
            lastPositionsY[i] = lastPositionsY[i-1];
        }
        lastPositionsY[0] = newPosition;
    }

    float getY(int index) { // Função responsável por retornar o valor da posição desejada
        if(index >= 3) {
            return NULL;
        }
        return lastPositionsY[index];
    }

    float noiseFilterX() { //Função que filtra os valores de X
        sum = 0;
        float auxSUm = 0;
        for(int i = 0; i < 3; i++) {
            auxSUm = lastPositionsX[i] * (3 - i); // ele pega a posição e multiplica pelo peso, como i começa com 0, o peso inicial é 3.
            sum += auxSUm;
        }
        sum /= 6;
        return sum;
    }

    float noiseFilterY() { //Função que filtra os valores de Y
        sum = 0;
        float auxSUm = 0;
        for(int i = 0; i < 3; i++) {
            auxSUm = lastPositionsY[i] * (3 - i); // ele pega a posição e multiplica pelo peso, como i começa com 0, o peso inicial é 3.
            sum += auxSUm;
        }
        sum /= 6;
        return sum;
    }


};
// classe responsável por filtrar o ruído das posições do robô.
class FilterRobots {
private:
    float lastPositionsX[3] = {0, 0, 0};
    float lastPositionsY[3] = {0, 0, 0};
    int id;
    float sum = 0;
    bool activate = false;

public:
    FilterRobots(int robotId) {
        id = robotId;
    }
    bool getActivate() {
        return activate;
    }
    void setLastX(float newPosition) { // Função responsável por atualizar os valores de X
        activate = true;
        for(int i = 3; i > 0 ; i-- ) {
            lastPositionsX[i] = lastPositionsX[i-1];
        }
        lastPositionsX[0] = newPosition;
    }

    float getX(int index) { // Função responsável por retornar o valor da posição desejada
        if(index >= 3) {
            return NULL;
        }
        return lastPositionsX[index];
    }

    void setLastY(float newPosition) { // Função responsável por atualizar os valores de X
        for(int i = 3; i > 0 ; i-- ) {
            lastPositionsY[i] = lastPositionsY[i-1];
        }
        lastPositionsY[0] = newPosition;
    }

    float getY(int index) { // Função responsável por retornar o valor da posição desejada
        if(index >= 3) {
            return NULL;
        }
        return lastPositionsY[index];
    }

    float noiseFilterX() { //Função que filtra os valores de X
        sum = 0;
        float auxSUm = 0;
        for(int i = 0; i < 3; i++) {
            auxSUm = lastPositionsX[i] * (3 - i); // ele pega a posição e multiplica pelo peso, como i começa com 0, o peso inicial é 3.
            sum += auxSUm;
        }
        sum /= 6;
        return sum;
    }

    float noiseFilterY() { //Função que filtra os valores de Y
        sum = 0;
        float auxSUm = 0;
        for(int i = 0; i < 3; i++) {
            auxSUm = lastPositionsY[i] * (3 - i); // ele pega a posição e multiplica pelo peso, como i começa com 0, o peso inicial é 3.
            sum += auxSUm;
        }
        sum /= 6;
        return sum;
    }


};


FilterRobots robots[11] = {0, 1, 2, 3, 4, 5 ,6 ,7 ,8, 9, 10 };
FilterBall ballFilter;

void update_csv_file(float positionWithNoise, float positionFiltred){
    FILE *fp;
    fp=fopen("positions_with_noise.csv","r+");
    fseek(fp, 0, SEEK_END);
    fprintf(fp,"\n%9.2f,%9.2f",positionWithNoise ,positionFiltred);
    fclose(fp);
}

void printRobotInfo(const SSL_DetectionRobot & robot, int id) {


    float lastPosition = robot.x();
    robots[id].setLastX(lastPosition);

    printf("CONF=%4.2f ", robot.confidence());

    if (robot.has_robot_id()) {
        printf("ID=%3d ",robot.robot_id());
    } else {
        printf("ID=N/A ");
    }

    update_csv_file(lastPosition, robots[0].noiseFilterX());


    // printf("\n\nO robo está(aprx) na posicao: %9.2f e posicao real e: %9.2f \n\n", lastPositionsX[1] + delta, robot.x() );

    printf(" HEIGHT=%6.2f POS=<%9.2f,%9.2f> ",robot.height(),robot.x(), robot.y());
    if (robot.has_orientation()) {
        printf("ANGLE=%6.3f ",robot.orientation());
    } else {
        printf("ANGLE=N/A    ");
    }



    printf("RAW=<%8.2f,%8.2f>\n",robot.pixel_x(),robot.pixel_y());



}

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
                for (int i = 0; i < balls_n; i++) {
                    SSL_DetectionBall ball = detection.balls(i);

                    ballFilter.setLastX(ball.x());
                    ballFilter.setLastY(ball.y());
                    //printf("-Ball (%2d/%2d): CONF=%4.2f POS=<%9.2f,%9.2f> ", i+1, balls_n, ball.confidence(),ball.x(),ball.y());
                    printf("-Ball (%2d/%2d): CONF=%4.2f POS=<%9.2f,%9.2f> ", i+1, balls_n, ball.confidence(),ballFilter.noiseFilterX(),ballFilter.noiseFilterY());

                    if (ball.has_z()) {
                        printf("Z=%7.2f ",ball.z());
                    } else {
                        printf("Z=N/A   ");
                    }
                    printf("RAW=<%8.2f,%8.2f>\n",ball.pixel_x(),ball.pixel_y());
                }


                //Blue robot info:

                for (int i = 0; i < robots_blue_n; i++) {
                    SSL_DetectionRobot robot = detection.robots_blue(i);
                    printf("-Robot(B) (%2d/%2d): ",i+1, robots_blue_n);
                    printRobotInfo(robot, i);
                    if(robot.x() <= 0){
                        grSim_client.sendCommand(1.0, i);
                    }else{
                        grSim_client.sendCommand(-1.0, i);
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
