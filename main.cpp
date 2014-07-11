/* 
 * File:   main.cpp
 * Author: evan
 *
 * Created on June 23, 2014, 3:11 PM
 */

#include <cstdlib>
#include "SimpleVisionProcessor.hpp"

using namespace std;

/*
 * 
 */
int main(int argc, char** argv) {

  SimpleVisionProcessor svp;
  svp.run();
  
  while (true) {
    boost::this_thread::sleep(boost::posix_time::seconds(1));
  }
  
  svp.stop();
  return 0;
}

