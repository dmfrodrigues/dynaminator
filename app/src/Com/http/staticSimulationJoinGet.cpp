#include <spdlog/spdlog.h>

#include <nlohmann/json.hpp>

#include "Com/HTTPServer.hpp"
#include "GlobalState.hpp"

using namespace std;
using namespace Com;

using json = nlohmann::json;

/**yaml GET /static/simulation/{id}/join
 * summary: Wait for simulation to finish.
 * tags:
 *   - Static
 * parameters:
 *   - name: id
 *     in: path
 *     required: true
 *     description: ID of simulation
 *     schema:
 *       type: string
 *       pattern: '^[\w\-]+$'
 * responses:
 *   '200':
 *     description: Successfully waited for simulation
 */
void HTTPServer::staticSimulationJoinGet(const httplib::Request &req, httplib::Response &res) {
    const string &resourceID = req.matches[1];
    const string &taskID     = "task://" + resourceID;

    try {
        shared_future<GlobalState::TaskReturn> &future = GlobalState::tasks.get(taskID);

        GlobalState::TaskReturn ret = future.get();

        res.status = ret.status;
        res.set_content(ret.content, ret.content_type);
    } catch(const GlobalState::ResourceException &e) {
        res.status = 400;
        res.set_content("what(): "s + e.what(), "text/plain");
    } catch(const exception &e) {
        res.status = 500;
        res.set_content("what(): "s + e.what(), "text/plain");
        spdlog::error("Exception, what(): {}", e.what());
    }
}
