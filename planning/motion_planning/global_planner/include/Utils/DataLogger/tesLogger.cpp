#include <iostream>
#include <iomanip>

#include "DataLogger.h"




int main()
{
    DataLogger dlog("log.csv");
	int x=0,y=5;
    dlog << "value 1,"  << x << ", value 2," << y <<std::endl;
    dlog << "value 2,"  << y << ", value 2," << x <<std::endl;
  //  dlog << "3testing:"  <<std::setw(30)<< "error!1"  << "error!2" <<  "error!2" <<std::endl;
 //   dlog << "t4esting:"  << "error!1"  << "error!\n\n2" <<  "error!2" <<std::endl;
    return 0;
}
