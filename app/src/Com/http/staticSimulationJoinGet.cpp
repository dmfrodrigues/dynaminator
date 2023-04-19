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
    // TODO: implement this.
    /*
    To implement this, I need:
    - To store the thread where the simulation is running
    - Protect the map of threads with a mutex
    - TO either join the thread, or have a condition variable to notify that operation is over.
    */
    const string &resourceID = req.matches[1];
    const string &taskID     = "task://" + resourceID;

    try {
        shared_ptr<shared_future<GlobalState::TaskReturn>> future;
        try {
            lock_guard<mutex> lock(GlobalState::tasks);
            future = GlobalState::tasks->at(taskID);
        } catch(const std::out_of_range &e) {
            res.status = 404;
            res.set_content("No such task " + taskID, "text/plain");
            return;
        }

        GlobalState::TaskReturn ret = future->get();

        res.status = ret.status;
        res.set_content(ret.content, ret.content_type);

    } catch(const exception &e) {
        res.status = 500;
        res.set_content("what(): "s + e.what(), "text/plain");
        cerr << "what(): " << e.what() << endl;
    }
}