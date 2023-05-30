#include "data/SUMO/EdgeData.hpp"

#include <filesystem>
#include <fstream>
#include <iostream>

#include "data/SUMO/Network.hpp"
#include "utils/stringify.hpp"

using namespace std;
using namespace SUMO;
using namespace rapidxml;

using utils::stringify::stringify;

namespace fs = std::filesystem;

typedef Network::Edge Edge;

EdgeData::Interval::Edge::Edge(Network::Edge::ID id) {
    attributes.setAttribute("id", id);
}

EdgeData::Interval::Interval(Time begin, Time end) {
    attributes.setAttribute("begin", begin);
    attributes.setAttribute("end", end);
}

EdgeData::Interval::Interval(Time begin, Time end, Interval::ID id) {
    attributes.setAttribute("begin", begin);
    attributes.setAttribute("end", end);
    attributes.setAttribute("id", id);
}

EdgeData::Interval::Edge &EdgeData::Interval::createEdge(Network::Edge::ID id) {
    edges.emplace(id, Edge(id));
    return edges.at(id);
}

EdgeData::Interval &EdgeData::createInterval(Time begin, Time end) {
    intervals.emplace_back(Interval(begin, end));
    return intervals.back();
}

EdgeData::Interval &EdgeData::createInterval(Time begin, Time end, EdgeData::Interval::ID id) {
    intervals.emplace_back(Interval(begin, end, id));
    idToInterval.emplace(id, &intervals.back());
    return intervals.back();
}

template<typename T>
void add_attr(xml_node<> &xmlNode, const string &key, const T &val) {
    xml_document<> &doc  = *xmlNode.document();
    auto            attr = doc.allocate_attribute(
        doc.allocate_string(
            key.c_str()
        ),
        doc.allocate_string(
            stringify<T>::toString(val).c_str()
        )
    );
    xmlNode.append_attribute(attr);
}

void EdgeData::saveToFile(
    const string &filePath
) const {
    xml_document<> doc;
    xml_node<>    &meandata = *doc.allocate_node(node_element, "meandata");
    doc.append_node(&meandata);

    for(const Interval &interval: intervals) {
        xml_node<> &intervalEl = *doc.allocate_node(node_element, "interval");
        meandata.append_node(&intervalEl);

        for(const auto &[name, value]: interval.attributes.attributes) {
            add_attr(intervalEl, name, value);
        }

        for(const auto &[edgeID, edge]: interval.edges) {
            xml_node<> &edgeEl = *doc.allocate_node(node_element, "edge");
            intervalEl.append_node(&edgeEl);

            for(const auto &[name, value]: edge.attributes.attributes) {
                add_attr(edgeEl, name, value);
            }
        }

        ofstream os;
        os.exceptions(ios_base::failbit | ios_base::badbit);
        fs::path p = fs::path(filePath).parent_path();
        if(!fs::is_directory(p)) {
            cerr << "Creating directory " << p << endl;
            if(!fs::create_directory(p)) {
                throw ios_base::failure("Could not create directory " + p.string());
            }
        }
        try {
            os.open(filePath);
        } catch(const ios_base::failure &ex) {
            throw ios_base::failure("Could not open file " + filePath);
        }
        os << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
        os << doc;
    }
}
