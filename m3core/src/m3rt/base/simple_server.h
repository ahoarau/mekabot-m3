/* 
M3 -- Meka Robotics Real-Time Control System
Copyright (c) 2010 Meka Robotics
Author: edsinger@mekabot.com (Aaron Edsinger)

M3 is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

M3 is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License
along with M3.  If not, see <http://www.gnu.org/licenses/>.
*/


#ifndef  M3RT_SIMPLE_SERVER_H
#define  M3RT_SIMPLE_SERVER_H

#include <string>
#include <vector>
#include <sys/select.h>

namespace m3rt
{
/**
 * @brief
 *
 */
class M3SimpleServer
{
public:
	M3SimpleServer():nb_rx(0),nb_tx(0),buf_rx(0),buf_tx(0),socket_fd(-1),listener(-1){}
    /**
     * @brief
     *
     */
    ~M3SimpleServer();
    /**
     * @brief
     *
     * @param port The port number (default is 10000)
     * @return bool True if startup was successfull (no binding / already assign port errors)
     */
    bool Startup(int port);//Non-blocking
    /**
     * @brief
     *
     */
    void Shutdown();
    /**
     * @brief
     *
     * @param s
     * @return int
     */
    int WriteStringToPort(std::string & s); //Non-blocking
    /**
     * @brief
     *
     * @param s
     * @param size
     * @return int
     */
    int  ReadStringFromPort(std::string & s, int & size);//Non-blocking
protected:
    /**
     * @brief
     *
     * @return bool
     */
    bool HandleNewConnection();//Non-blocking
    /**
     * @brief
     *
     * @return bool
     */
    bool IsActiveSocket(){return socket_fd!=-1;}
    int portno; 
    int nb_rx; 
    int nb_tx; 
    unsigned char * buf_rx; 
    unsigned char * buf_tx; 
    fd_set master;   // master file descriptor list 
    fd_set read_fds; // temp file descriptor list for select() 
    fd_set write_fds; // temp file descriptor list for select() 
    int fdmax;        // maximum file descriptor number 
    int socket_fd; 
    int listener;     // listening socket descriptor 
    struct timeval tv; 
};
}
#endif


