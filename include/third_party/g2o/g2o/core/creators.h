//
// Created by yc_qian on 19-12-20.
//

#ifndef ORB_SLAM2_KINETIC_CREATORS_H
#define ORB_SLAM2_KINETIC_CREATORS_H

#include "hyper_graph.h"

#include <string>
#include <typeinfo>

namespace g2o
{

    /**
     * \brief Abstract interface for allocating HyperGraphElement
     */
    class  AbstractHyperGraphElementCreator
    {
    public:
        /**
         * create a hyper graph element. Has to implemented in derived class.
         */
        virtual HyperGraph::HyperGraphElement* construct() = 0;
        /**
         * name of the class to be created. Has to implemented in derived class.
         */
        virtual const std::string& name() const = 0;

        virtual ~AbstractHyperGraphElementCreator() { }
    };

    /**
     * \brief templatized creator class which creates graph elements
     */
    template <typename T>
    class HyperGraphElementCreator : public AbstractHyperGraphElementCreator
    {
    public:
        HyperGraphElementCreator() : _name(typeid(T).name()) {}
#if defined (WINDOWS) && defined(__GNUC__) // force stack alignment on Windows with GCC
        __attribute__((force_align_arg_pointer))
#endif
        HyperGraph::HyperGraphElement* construct() { return new T;}
        virtual const std::string& name() const { return _name;}
    protected:
        std::string _name;
    };

} // end namespace

#endif //ORB_SLAM2_KINETIC_CREATORS_H
