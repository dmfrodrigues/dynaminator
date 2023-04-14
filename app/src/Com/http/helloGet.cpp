#include "Com/HTTPServer.hpp"

using namespace std;
using namespace Com;

/**yaml GET /hello
 * summary: Ping server.
 * tags:
 *   - Global
 */
void HTTPServer::helloGet(const httplib::Request &, httplib::Response &res) {
    res.set_content("Hello world!\n", "text/plain");
}
