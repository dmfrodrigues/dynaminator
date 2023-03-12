#pragma once

#include <string>
#include <unordered_map>
#include <vector>

/**
 * @brief Demand in O-format file.
 *
 * See:
 * - http://www.chinautc.com/information/manage/UNCC_Editor/uploadfile/20081105144806983.pdf
 * - https://sumo.dlr.de/docs/Demand/Importing_O/D_Matrices.html#the_o-format_visumvissim
 * for more info on the VISUM O-format for matrices.
 */
class OFormatDemand {
   public:
    typedef std::string Node;
    typedef double Flow;
    typedef double Time;

   private:
    Time from, to;
    double factor;
    std::unordered_map<
        Node,
        std::unordered_map<
            Node,
            Flow> >
        flows;

   public:
    void addDemand(Node u, Node v, Flow f);
    std::vector<Node> getStartNodes() const;
    std::vector<Node> getDestinations(Node u) const;
    Flow getDemand(Node u, Node v) const;

    static OFormatDemand loadFromFile(const std::string &path);
};
