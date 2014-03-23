#ifndef DATALOG_H
#define DATALOG_H
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <time.h>
#include <sys/time.h>
#include <string>
#include <streambuf>
#include <unistd.h>
#include <stdio.h>

class DataLogger: public std::ostringstream
{
public:


private:

    std::ofstream fout;
public:


    void flush()
    {

        //////////////////////////////
        //
        // prefixes lines with a time
        // stamp
        // YYYY/MM/DD HH:MM:SS.uuuuuu,
        // where the u are microseconds.
        //
        struct timeval tv;
        gettimeofday(&tv,0);
        time_t now = tv.tv_sec;
        struct tm * now_tm = localtime(&now);
        char buffer[40];
        strftime(buffer,sizeof(buffer),"%Y/%m/%d, %H:%M:%S",now_tm);
        fout << buffer
             << "." << std::setfill('0')
             << std::setw(6) << (tv.tv_usec)<<','
             << std::setfill(' ') << ' '
             << str();
        fout.flush();
        str("");
    }



    template <typename T>
    inline DataLogger & operator<<( const T & t )
    {
        (*(std::ostringstream*)this) << t;
        return *this;
    }

    DataLogger(char *filename)
    {
        fout.open(filename,std::ios::app);
    }

    virtual ~DataLogger()
    {
        fout.close();
    }


    typedef DataLogger & (*DataLogger_manip)(DataLogger &);
    DataLogger & operator<<(DataLogger_manip manip) {
        return manip(*this);
    }

};

namespace std {
inline DataLogger & endl(DataLogger & out) {
    out.put('\n');
    out.flush();
    return out;
}
}

#endif // DATALOG_H ///:~
