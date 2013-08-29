#include "ServoSerial.h"

#define deg2rad(deg) ((deg*M_PI/180.0))

int kbhit(void)
{
    struct termios oldt, newt;
    int ch;
    int oldf;

    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

    ch = getchar();

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);

    if (ch != EOF) {
        ungetc(ch, stdin);
        return 1;
    }

    return 0;
}

static int id = 2;
void usage() {
    printf("servo test program, current id = %d\n", id);
    printf(" 1-9 : change id\n");
    printf("   r : setReset\n");
    printf("   o : setTorqueOn\n");
    printf("   f : setTorqueOff\n");
    printf("   b : setTorqueBreak\n");
    printf("   p : setPosition to 90\n");
    printf("   P : setPosition to -90\n");
    printf("   i : setPositions 90 to id 2 3 4 5 6 6 8\n");
    printf("   I : setPositions -90 to id 2 3 4 5 6 6 8\n");
    printf("   m : setMaxTorque to 100\n");
    printf("   M : setMaxTorque to  50\n");
    printf("   g : getPosition\n");
    printf("   G : getPositions\n");
    printf("   d : getDuration\n");
    printf("   s : getSpeed\n");
    printf("   t : getTorque\n");
    printf("   c : getTemperature\n");
    printf("   v : getVoltage\n");
    printf("   S : getState\n");
}

int main() {
    ServoSerial *serial;

    serial = new ServoSerial("/dev/serusb1");

    char ch = 0;
    usage();
    while (ch!='q') {
        if(kbhit()) {
            ch = getchar();
            switch (ch) {
            case '1':
            case '2':
            case '3':
            case '4':
            case '5':
            case '6':
            case '7':
            case '8':
            case '9':
                id = ch-'0';
                serial->clear_packet();
                break;
            case 'r':
                serial->setReset(id);
                break;
            case 'o':
                serial->setTorqueOn(id);
                break;
            case 'f':
                serial->setTorqueOff(id);
                break;
            case 'b':
                serial->setTorqueBreak(id);
                break;
            case 'p':
                serial->setPosition(id, deg2rad(90), 3);
                break;
            case 'P':
	        serial->setPosition(id, deg2rad(-90), 3);
                break;
            case 'i':
                {int ids[8] = {2,3,4,5,6,7,8,9};
                 double rads[8] = {90,90,90,90,90,90,90,90};
                 double secs[8] = { 3, 3, 3, 3, 3, 3, 3, 3};
	         serial->setPositions(8, ids, rads, secs);}
                break;
            case 'I':
                {int ids[8] = {2,3,4,5,6,7,8,9};
                 double rads[8] = {-90,-90,-90,-90,-90,-90,-90,-90};
                 double secs[8] = { 3, 3, 3, 3, 3, 3, 3, 3};
	         serial->setPositions(8, ids, rads, secs);}
                break;
            case 'm':
                serial->setMaxTorque(id, 100);
                break;
            case 'M':
                serial->setMaxTorque(id, 50);
                break;
            case 'g':
                double angle;
                serial->getPosition(id, &angle);
		fprintf(stderr, "angle = %f [deg]\n", angle);
                break;
            case 'G':
                {int ids[8] = {2,3,4,5,6,7,8,9};
                 double angles[8];
                 for(int i = 0; i < 8; i++ ) {
                        while ( serial->getPosition(ids[i], &angles[i]) < 0 ) 
                                usleep(1000);
                }       
        	 fprintf(stderr, "angles = %f %f %f %f %f %f %f %f [deg]\n", angles[0],angles[1],angles[2],angles[3],angles[4],angles[5],angles[6],angles[7]);}
                break;
            case 'd':
                double duration;
                serial->getDuration(id, &duration);
		fprintf(stderr, "duration = %f [msec]\n", duration);
                break;
            case 's':
                double speed;
                serial->getSpeed(id, &speed);
		fprintf(stderr, "speed = %f [deg/sec]\n", speed);
                break;
            case 't':
                double torque;
                serial->getTorque(id, &torque);
		fprintf(stderr, "torque = %f [mA]\n", torque);
                break;
            case 'c':
                double temperature;
                serial->getTemperature(id, &temperature);
		fprintf(stderr, "temperature = %f [C]\n", temperature);
                break;
            case 'v':
                double voltage;
                serial->getVoltage(id, &voltage);
		fprintf(stderr, "voltage = %f [V]\n", voltage);
                break;
            case 'S':
                unsigned char data[30];
                serial->getState(id, data);
		fprintf(stderr, "state =");
                for(int i = 0; i < 30; i++) {
                        fprintf(stderr, " %02X", data[i]);
                        if ( i % 10 == 9 ) fprintf(stderr, " : ");
                }
		fprintf(stderr, "\n");
                break;
            }
            usage();
        }
    }
    printf("done");

    delete serial;
}
