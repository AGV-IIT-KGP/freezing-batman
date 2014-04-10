/* 
 * File:   Server.hpp
 * Author: samuel
 *
 * Created on 14 December, 2013, 3:06 AM
 */

#ifndef SERVER_HPP
#define	SERVER_HPP

#include <Representation.hpp>

namespace environment {

    class Server {
    public:
        Server();
        Server(const Server& orig);
        virtual ~Server();
    private:
        Representation representation;
    };
}

#endif	/* SERVER_HPP */

