//
// Created by owain on 8/22/15.
//

#include <queue>
#include <boost/foreach.hpp>
#include "ConfigMerger.hpp"

boost::property_tree::ptree ConfigMerger::mergePropertyTrees(
        const boost::property_tree::ptree &rptFirst,
        const boost::property_tree::ptree &rptSecond) {
    // Take over first property tree
    boost::property_tree::ptree ptMerged = rptFirst;

    // Keep track of keys and values (subtrees) in second property tree
    std::queue<std::string> qKeys;
    std::queue<boost::property_tree::ptree> qValues;
    qValues.push( rptSecond );

    // Iterate over second property tree
    while( !qValues.empty() )
    {
        // Setup keys and corresponding values
        boost::property_tree::ptree ptree = qValues.front();
        qValues.pop();
        std::string keychain = "";
        if( !qKeys.empty() )
        {
            keychain = qKeys.front();
            qKeys.pop();
        }

        // Iterate over keys level-wise
        BOOST_FOREACH( const boost::property_tree::ptree::value_type& child, ptree )
        {
            // Leaf
            if( child.second.size() == 0 )
            {
                // No "." for first level entries
                std::string s;
                if( keychain != "" )
                    s = keychain + "." + child.first.data();
                else
                    s = child.first.data();

                // Put into combined property tree
                ptMerged.put( s, child.second.data() );
            }
                // Subtree
            else
            {
                // Put keys (identifiers of subtrees) and all of its parents (where present)
                // aside for later iteration. Keys on first level have no parents
                if( keychain != "" )
                    qKeys.push( keychain + "." + child.first.data() );
                else
                    qKeys.push( child.first.data() );

                // Put values (the subtrees) aside, too
                qValues.push( child.second );
            }
        }  // -- End of BOOST_FOREACH
    }  // --- End of while

    return ptMerged;
}
