#include <iostream>
#include <sys/stat.h>
#include <fcntl.h>
#include <cerrno>
#include <string>

using namespace std;

struct Sample {
    timespec ts;
    short data[16];
};

Sample buf[600];

int main(int argc, char *argv[]) {
    int povd;
    int ret;
    long last_ns = 0;
    string dev;
    if (argc != 2) {
        cout << "Usage: testpov <pov_number>" << endl;
        return 1;
    }
    dev = "/dev/pov" + string(argv[1]);
    povd = open(dev.c_str(), O_RDONLY);
    if (povd == -1) {
        cout << "Error (" << errno << ") to open" << dev << endl;
        return 2;
    }
    for (;;) {
        ret = read(povd, buf, sizeof(buf));
        if (ret == -1) {
            cout << "Error (" << errno << ") to read" << dev << endl;
            return 3;
        }                 
        //cout << "Bytes read:" << ret << endl;
        //write(STDOUT_FILENO, buf, sizeof(buf));
#if 0
        for (int i=0; i < sizeof(buf)/sizeof(Sample); i++) {
            long diff = buf[i].ts.tv_nsec - last_ns;
            last_ns =  buf[i].ts.tv_nsec;
            if (diff < 0)
                diff += 1000000000;
            cout << dec << diff << endl;
            cout << dec << buf[i].ts.tv_sec << " " << buf[i].ts.tv_nsec << endl;
#endif	    
#if 0
            for (int j=0; j < 32; j++)
                cout << "0x" << hex << (int)buf[i].data[j] << " ";
            cout << endl;
        }
#endif
        //for(int j=0; j < 1000000; j++);
    }
    close(povd);
    return 0;
}
