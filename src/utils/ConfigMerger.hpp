//
// Created by owain on 8/22/15.
//

#ifndef QUADTARGETFSM_CONFIGMERGER_H
#define QUADTARGETFSM_CONFIGMERGER_H

#include <boost/property_tree/ptree.hpp>

class ConfigMerger {
    /**
    * Merge two Boost property trees.
    * Code snippet lifted from http://paste.tbee-clan.de/TX2Vm
    * Original author unknown!
    * Used whilst loading multiple configuration files.
    * The values in the second tree simply override those in the first.
    **/
    public:
        static boost::property_tree::ptree mergePropertyTrees(
            const boost::property_tree::ptree &rptFirst,
            const boost::property_tree::ptree &rptSecond
        );
};


#endif //QUADTARGETFSM_CONFIGMERGER_H
