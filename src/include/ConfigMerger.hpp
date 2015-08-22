//
// Created by owain on 8/22/15.
//

#ifndef QUADTARGETFSM_CONFIGMERGER_H
#define QUADTARGETFSM_CONFIGMERGER_H

#include <boost/property_tree/ptree.hpp>

class ConfigMerger {
    public:
        static boost::property_tree::ptree mergePropertyTrees(
            const boost::property_tree::ptree &rptFirst,
            const boost::property_tree::ptree &rptSecond
        );
};


#endif //QUADTARGETFSM_CONFIGMERGER_H
